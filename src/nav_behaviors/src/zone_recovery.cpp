// ZoneRecovery: see zone_recovery.hpp.
#include <memory>
#include <cstdint>
#include <string>
#include <sstream>
#include <iomanip>

#include "nav_behaviors/zone_recovery.hpp"

namespace nav_behaviors
{

namespace
{
// child order of ZoneRecovery
enum Child { kNav = 0, kClear = 1, kSpin = 2, kBackUp = 3, kGiveUp = 4, kChildren = 5 };

int childOf(RecoveryAction a)
{
  switch (a) {
    case RecoveryAction::ClearCostmaps: return kClear;
    case RecoveryAction::Spin: return kSpin;
    case RecoveryAction::BackUp: return kBackUp;
    case RecoveryAction::GiveUp: return kGiveUp;
  }
  return kGiveUp;
}
}  // namespace

ZoneRecovery::ZoneRecovery(const std::string & name, const BT::NodeConfiguration & conf)
: BT::ControlNode(name, conf)
{
  // Subscribe right away: a test or a manual drive may poll the zone state before the first tick,
  // and the first failure must already see a fresh phase.
  readPorts();
}

void ZoneRecovery::readPorts()
{
  std::string topic = "/nav_zone_state";
  getInput("retries", cfg_.retries);
  getInput("state_topic", topic);
  getInput("zone_timeout", zone_timeout_);
  getInput("spin_after", cfg_.spin_after);
  getInput("backup_after", cfg_.backup_after);
  getInput("spin", cfg_.spin);
  getInput("backup", cfg_.backup);
  std::string file;
  getInput("zones_file", file);
  getInput("global_frame", global_frame_);
  getInput("base_frame", base_frame_);
  getInput("zone_margin", zone_margin_);
  if (file != zones_file_) {
    zones_ = loadZones(file, 50, 50,
      {"tunnel", "slope", "step_l1", "step_l2", "step_high", "fly_slope", "undulation"}).zones;
    zones_file_ = file;
    tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }
  if (!monitor_.ready() || monitor_.topic() != topic) {
    const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    monitor_.initialize(node, topic);
    logger_ = node->get_logger();
  }
}

bool ZoneRecovery::motionBlocked()
{
  // A stopped controller no longer publishes phase heartbeats. Elapsed time does
  // not prove that the chassis has left the tunnel: check physical occupancy.
  if (!zones_.empty()) {
    try {
      const auto pose = tf_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      const auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      const rclcpp::Time stamp(pose.header.stamp, node->get_clock()->get_clock_type());
      const double age = (node->now() - stamp).seconds();
      if (stamp.nanoseconds() != 0 && (age < 0.0 || age > 0.5)) {return true;}
      for (const auto & z : zones_) {
        double u, v;
        z.toLocal(pose.transform.translation.x, pose.transform.translation.y, u, v);
        if (std::abs(u) <= 0.5 * z.depth + zone_margin_ &&
          std::abs(v) <= 0.5 * z.width + zone_margin_) {return true;}
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_,
        "ZoneRecovery: no robot transform, motion recovery inhibited: %s", e.what());
      return true;
    }
  }
  return monitor_.blockedRecently(zone_timeout_);
}

bool ZoneRecovery::goalChanged()
{
  // A new target also interrupts a running recovery and starts its own ladder.
  // For through-poses, the final pose/stamp is stable when RemovePassedGoals
  // removes reached waypoints from the front of the blackboard vector.
  geometry_msgs::msg::PoseStamped goal;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (config().blackboard->get("goals", goals) && !goals.empty()) {
    goal = goals.back();
  } else if (!config().blackboard->get("goal", goal)) {
    return false;
  }
  std::ostringstream stream;
  const auto & p = goal.pose.position;
  const auto & q = goal.pose.orientation;
  stream << std::setprecision(17) << goal.header.frame_id << ' ' << goal.header.stamp.sec << ' '
    << goal.header.stamp.nanosec << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' '
    << q.x << ' ' << q.y << ' ' << q.z << ' ' << q.w;
  const auto signature = stream.str();
  const bool changed = !goal_signature_.empty() && signature != goal_signature_;
  goal_signature_ = signature;
  return changed;
}

BT::NodeStatus ZoneRecovery::tick()
{
  if (childrenCount() != static_cast<std::size_t>(kChildren)) {
    throw BT::BehaviorTreeException(
            "ZoneRecovery '" + name() + "' needs 5 children: navigation, clear, spin, back-up, give-up");
  }
  readPorts();
  monitor_.spin();
  if (goalChanged()) {
    haltChildren();
    attempt_ = 0;
    recovering_ = false;
    zone_hold_ = false;
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (recovering_) {
    const int idx = childOf(action_);
    if ((action_ == RecoveryAction::Spin || action_ == RecoveryAction::BackUp) && motionBlocked()) {
      haltChild(static_cast<std::size_t>(idx));
      recovering_ = false;
      zone_hold_ = true;
      ++attempt_;
      return BT::NodeStatus::RUNNING;
    }
    const BT::NodeStatus s = children_nodes_[idx]->executeTick();
    if (s == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    haltChild(static_cast<std::size_t>(idx));
    recovering_ = false;
    ++attempt_;
    // one recovery action per tick cycle: give the next plan / follow attempt a fresh BT tick
    return BT::NodeStatus::RUNNING;
  }

  const BT::NodeStatus nav = children_nodes_[kNav]->executeTick();
  if (nav == BT::NodeStatus::SUCCESS) {
    // Nav2 aggregates these blackboard values into the action result. A
    // recovered intermediate failure must not survive a completed navigation.
    for (const auto * key : {"compute_path_error_code", "follow_path_error_code", "smoother_error_code"}) {
      if (config().blackboard->getEntry(key)) {
        config().blackboard->set<std::uint16_t>(key, 0);
      }
    }
    halt();   // resets the ladder for the next goal
    return BT::NodeStatus::SUCCESS;
  }
  if (nav == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  zone_hold_ = zone_hold_ || motionBlocked();
  const bool in_zone = zone_hold_;
  action_ = recoveryAction(cfg_, attempt_, in_zone);
  if (action_ == RecoveryAction::GiveUp) {
    RCLCPP_WARN(logger_, "ZoneRecovery: navigation failed %d times%s, giving up",
      attempt_, in_zone ? " (in a zone)" : "");
    halt();
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_WARN(logger_, "ZoneRecovery: attempt %d/%d -> %s%s", attempt_ + 1, cfg_.retries,
    recoveryActionName(action_), in_zone ? " (in a zone, no motion)" : "");
  const int idx = childOf(action_);
  const BT::NodeStatus rec = children_nodes_[idx]->executeTick();
  if (rec == BT::NodeStatus::RUNNING) {
    recovering_ = true;
    return BT::NodeStatus::RUNNING;
  }
  // finished synchronously (costmap clear): one cycle is over, retry navigation on the next tick
  haltChild(static_cast<std::size_t>(idx));
  ++attempt_;
  return BT::NodeStatus::RUNNING;
}

void ZoneRecovery::halt()
{
  ControlNode::halt();
  attempt_ = 0;
  recovering_ = false;
  zone_hold_ = false;
  goal_signature_.clear();
  action_ = RecoveryAction::ClearCostmaps;
}

}  // namespace nav_behaviors
