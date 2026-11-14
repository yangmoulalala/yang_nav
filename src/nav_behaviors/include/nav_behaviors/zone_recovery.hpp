// ZoneRecovery (BT control): the navigation retry loop with an escalating, zone-aware recovery ladder.
//
// Children, in order:
//   0  the navigation subtree (plan / shape / follow)
//   1  clear the costmaps and wait
//   2  spin in place
//   3  back up
//   4  give up (the child is expected to fail, e.g. <AlwaysFailure/>)
//
// While child 0 succeeds the node is transparent.  On its failure the node picks one recovery child
// with nav_behaviors::recoveryAction (pure policy, unit-tested) and ticks only that one, so retries
// escalate instead of repeating the same action.  After the recovery child finishes, child 0 is
// ticked again.  When the policy returns GiveUp the whole node returns FAILURE and the navigation
// task ends - the caller re-tasks, which is the only safe move inside a zone.
//
// `attempt` counts recovery cycles of the current navigation task and is reset by halt(), which the
// action server sends between goals and whenever child 0 succeeds.
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/control_node.h"
#include "nav_behaviors/recovery_policy.hpp"
#include "nav_behaviors/zone_phase_monitor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_behaviors/zones_yaml.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_behaviors
{

class ZoneRecovery : public BT::ControlNode
{
public:
  ZoneRecovery(const std::string & name, const BT::NodeConfiguration & conf);
  ZoneRecovery() = delete;

  BT::NodeStatus tick() override;
  void halt() override;

  // Drain /nav_zone_state without ticking the tree (tests, manual drives).
  void pollZoneState() {monitor_.spin();}
  bool zoneFresh(double timeout) const {return monitor_.fresh(timeout);}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("retries", 5, "recovery cycles before giving up"),
      BT::InputPort<std::string>("state_topic", "/nav_zone_state", "zone phase topic"),
      BT::InputPort<std::string>("zones_file", "", "physical regions checked before motion recovery"),
      BT::InputPort<std::string>("global_frame", "map", "zone geometry frame"),
      BT::InputPort<std::string>("base_frame", "base_link", "robot frame"),
      BT::InputPort<double>("zone_margin", 0.55, "footprint radius plus clearance around each region"),
      BT::InputPort<double>("zone_timeout", 4.0,
        "ALIGN/TRAVERSE/EXIT reported within this -> no spin, no back-up"),
      BT::InputPort<int>("spin_after", 1, "first attempt that spins"),
      BT::InputPort<int>("backup_after", 2, "first attempt that backs up"),
      BT::InputPort<bool>("spin", true, "spin is allowed at all"),
      BT::InputPort<bool>("backup", true, "back-up is allowed at all"),
    };
  }

private:
  void readPorts();
  bool motionBlocked();
  bool goalChanged();

  ZonePhaseMonitor monitor_;
  RecoveryConfig cfg_;
  double zone_timeout_{4.0};
  int attempt_{0};
  bool recovering_{false};
  bool zone_hold_{false};
  std::string goal_signature_;
  std::string zones_file_, global_frame_{"map"}, base_frame_{"base_link"};
  double zone_margin_{0.55};
  std::vector<Zone> zones_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  RecoveryAction action_{RecoveryAction::ClearCostmaps};
  rclcpp::Logger logger_{rclcpp::get_logger("ZoneRecovery")};
};

}  // namespace nav_behaviors
