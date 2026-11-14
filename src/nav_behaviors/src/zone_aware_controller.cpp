// ZoneAwareController: nav2_core::Controller wrapping an inner controller (MPPI) with the ZoneTracker.
//
// Shared direction-constrained-zone state machine, reported on /nav_zone_state and used by the BT:
//
//   none      outside every window; the inner controller drives alone
//   approach  window entered, weight ramps 0 -> 1; MPPI and the tracker are mixed, a replan is still
//             allowed (the shaper re-shapes the new path)
//   align     heading locked on the zone axis, the tracker owns the robot (weight 1, MPPI not called);
//             replanning is paused from here so the shaped path is kept
//   traverse  inside the zone body
//   exit      past the far mouth, weight ramps back to 0
//
// Open directional terrain uses the same state machine with the ramp tuning profile. Its YAML policy
// selects the tracked direction and nose-first alignment where required; descent remains with MPPI.
//
// In the blend ramps an inner-controller failure falls back to the tracker.  The inner controller is
// configured with this plugin's name, so its parameters stay under <name>.*; the tracker's live under
// <name>.zone.*.  /nav_mode (latched, tunnel / ramp / normal) is the cmd_gate speed backstop and is
// published only here.
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_behaviors/zone_state.hpp"
#include "nav_behaviors/costmap_collision.hpp"
#include "nav_behaviors/overlap_escape.hpp"
#include "nav_behaviors/zone_tracker.hpp"
#include "nav_behaviors/zones_yaml.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav_behaviors
{

class ZoneAwareController : public nav2_core::Controller
{
public:
  ZoneAwareController()
  : loader_("nav2_core", "nav2_core::Controller") {}

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    using nav2_util::declare_parameter_if_not_declared;
    using rclcpp::ParameterValue;
    const std::string p = name + ".zone.";
    nav2_util::declare_parameter_if_not_declared(node, p + "speed_limit_reference", rclcpp::ParameterValue(1.5));
    node->get_parameter(p + "speed_limit_reference", speed_limit_reference_);
    TrackerParams tp;
    auto dbl = [&](const std::string & k, double & v) {
        declare_parameter_if_not_declared(node, p + k, ParameterValue(v));
        node->get_parameter(p + k, v);
      };
    // declared before the inner controller exists, so its parameter callback never sees them
    declare_parameter_if_not_declared(node, p + "zones_file", ParameterValue(std::string()));
    declare_parameter_if_not_declared(node, p + "min_cells", ParameterValue(50));
    declare_parameter_if_not_declared(node, p + "slope_min_cells", ParameterValue(50));
    const std::vector<std::string> default_zone_types{
      "tunnel", "slope", "step_l1", "step_l2", "step_high", "fly_slope", "undulation"};
    declare_parameter_if_not_declared(node, p + "types", ParameterValue(default_zone_types));
    declare_parameter_if_not_declared(node, p + "inner_plugin",
      ParameterValue(std::string("nav2_mppi_controller::MPPIController")));
    // from this weight on the tracker alone drives (no MPPI rollouts); below it the two are mixed
    declare_parameter_if_not_declared(node, p + "align_weight", ParameterValue(1.0));
    // /nav_zone_state is re-published at this rate so the BT condition never expires mid-zone
    declare_parameter_if_not_declared(node, p + "phase_heartbeat", ParameterValue(0.5));
    dbl("escape_speed", escape_speed_);
    dbl("escape_max_distance", escape_max_distance_);
    dbl("window_before", tp.window_before);
    dbl("window_after", tp.window_after);
    dbl("blend", tp.blend);
    dbl("exit_turn_span", tp.exit_turn_span);
    dbl("align_before", tp.align_before);
    dbl("tunnel_speed", tp.tunnel_speed);
    dbl("approach_speed", tp.approach_speed);
    dbl("decel", tp.decel);
    dbl("gate_dist", tp.gate_dist);
    dbl("front_extent", tp.front_extent);
    dbl("rear_extent", tp.rear_extent);
    dbl("gate_stop", tp.gate_stop);
    dbl("lat_tol", tp.lat_tol);
    double yaw_tol_deg = tp.yaw_tol * 180.0 / M_PI;
    dbl("yaw_tol_deg", yaw_tol_deg);
    tp.yaw_tol = yaw_tol_deg * M_PI / 180.0;
    dbl("k_lat", tp.k_lat);
    dbl("max_lat_speed", tp.max_lat_speed);
    dbl("k_yaw", tp.k_yaw);
    dbl("max_wz", tp.max_wz);
    dbl("min_turn_span", tp.min_turn_span);
    double wedge_deg = tp.wedge_yaw * 180.0 / M_PI;
    dbl("wedge_yaw_deg", wedge_deg);
    tp.wedge_yaw = wedge_deg * M_PI / 180.0;
    dbl("wedge_lat", tp.wedge_lat);
    dbl("align_speed", tp.align_speed);
    dbl("backoff_speed", tp.backoff_speed);
    dbl("search_dist", tp.search_dist);
    // Open directional terrain tuning profile
    dbl("ramp_speed", tp.ramp_speed);
    dbl("ramp_approach_speed", tp.ramp_approach_speed);
    dbl("ramp_decel", tp.ramp_decel);
    dbl("ramp_align_before", tp.ramp_align_before);
    dbl("ramp_lat_tol", tp.ramp_lat_tol);
    dbl("ramp_window_before", tp.ramp_window_before);
    dbl("ramp_window_after", tp.ramp_window_after);
    // chaining: the tracker must group the zones exactly as the shaper does, or it would look for a
    // tunnel body the shaped path no longer has
    dbl("chain_gap", tp.chain.gap);
    dbl("chain_lat", tp.chain.lat);
    double chain_angle_deg = tp.chain.angle * 180.0 / M_PI;
    dbl("chain_angle_deg", chain_angle_deg);
    tp.chain.angle = chain_angle_deg * M_PI / 180.0;
    tracker_.setParams(tp);
    std::string zones_file, inner_plugin;
    int min_cells = 50, slope_min_cells = 50;
    std::vector<std::string> types = default_zone_types;
    node->get_parameter(p + "zones_file", zones_file);
    node->get_parameter(p + "min_cells", min_cells);
    node->get_parameter(p + "slope_min_cells", slope_min_cells);
    node->get_parameter(p + "types", types);
    node->get_parameter(p + "inner_plugin", inner_plugin);
    node->get_parameter(p + "align_weight", align_weight_);
    node->get_parameter(p + "phase_heartbeat", heartbeat_);
    // Check the complete padded footprint over stopping distance plus check_margin.
    // The shaper and controller both reject nonzero footprint-shrinking tolerance.
    declare_parameter_if_not_declared(node, p + "check_margin", ParameterValue(0.3));
    declare_parameter_if_not_declared(node, p + "collision_tolerance", ParameterValue(0.0));
    declare_parameter_if_not_declared(node, p + "block_timeout", ParameterValue(5.0));
    double tol = 0.0;
    node->get_parameter(p + "check_margin", check_margin_);
    node->get_parameter(p + "collision_tolerance", tol);
    node->get_parameter(p + "block_timeout", block_timeout_);
    // per-cycle log inside the window (debugging handovers in Gazebo)
    declare_parameter_if_not_declared(node, p + "trace", ParameterValue(false));
    node->get_parameter(p + "trace", trace_);
    costmap_ros_ = costmap_ros;
    check_footprint_ = costmap_ros->getRobotFootprint();
    if (escape_speed_ <= 0.0 || escape_speed_ > 0.25 || escape_max_distance_ <= 0.0 || escape_max_distance_ > 2.0 ||
      tol != 0.0 || check_margin_ < 0.0 || block_timeout_ <= 0.0 ||
      tp.decel <= 0.0 || tp.ramp_decel <= 0.0 || tp.blend <= 0.0 || tp.max_wz <= 0.0 ||
      align_weight_ != 1.0 || heartbeat_ <= 0.0 || heartbeat_ >= 1.5) {
      throw std::runtime_error("ZoneAwareController: invalid safety, blending or heartbeat parameters");
    }
    if (zones_file.empty()) {
      RCLCPP_WARN(logger_, "ZoneAwareController: zone.zones_file not set, inner controller only");
    } else {
      const ZoneLoad l = loadZones(zones_file, min_cells, slope_min_cells, types);
      for (const auto & d : l.dropped) {
        RCLCPP_WARN(logger_, "ZoneAwareController: dropping %s, label fragment", d.c_str());
      }
      // chained zones are one stretch: the tracker must see the same bodies the shaper writes
      const auto grouped = groupZones(l.zones, tp.chain);
      tracker_.setZones(grouped);
      zones_ = grouped;
      RCLCPP_INFO(logger_, "ZoneAwareController: %d narrow zones, %d directional zones from %s -> %zu stretches, tunnel %.2f m/s, "
        "ramp %.2f m/s", l.tunnels, l.directional, zones_file.c_str(), grouped.size(), tp.tunnel_speed, tp.ramp_speed);
    }
    inner_ = loader_.createSharedInstance(inner_plugin);
    inner_->configure(parent, name, tf, costmap_ros);
    RCLCPP_INFO(logger_, "ZoneAwareController: inner controller %s", inner_plugin.c_str());
    mode_pub_ = node->create_publisher<std_msgs::msg::String>("/nav_mode", rclcpp::QoS(1).reliable().transient_local());
    phase_pub_ = node->create_publisher<std_msgs::msg::String>("/nav_zone_state",
        rclcpp::QoS(1).reliable().transient_local());
    // RViz: zone boxes + axes (latched), tracker state at the robot (window phase, errors, gate)
    zones_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/nav_zones/markers",
        rclcpp::QoS(1).reliable().transient_local());
    viz_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/nav_zones/tracker", 5);
    frame_ = costmap_ros->getGlobalFrameID();
  }

  void cleanup() override
  {
    if (inner_) {
      inner_->cleanup();
    }
    inner_.reset();
    mode_pub_.reset();
    phase_pub_.reset();
    zones_pub_.reset();
    viz_pub_.reset();
  }

  void activate() override
  {
    inner_->activate();
    mode_pub_->on_activate();
    phase_pub_->on_activate();
    zones_pub_->on_activate();
    viz_pub_->on_activate();
    publishMode(ZoneProfile::NarrowPassage, false, true);
    publishPhase(ZonePhase::None, std::string(), true);
    publishZones();
  }

  void deactivate() override
  {
    publishMode(ZoneProfile::NarrowPassage, false, true);
    publishPhase(ZonePhase::None, std::string(), true);
    inner_->deactivate();
    mode_pub_->on_deactivate();
    phase_pub_->on_deactivate();
    zones_pub_->on_deactivate();
    viz_pub_->on_deactivate();
  }

  void reset() override
  {
    inner_->reset();
    tracker_.resetProgress();
    escape_target_.reset();
    blocked_since_.reset();
    publishPhase(ZonePhase::None, std::string(), true);
    publishMode(ZoneProfile::NarrowPassage, false, true);
  }

  bool cancel() override
  {
    publishPhase(ZonePhase::None, std::string(), true);
    publishMode(ZoneProfile::NarrowPassage, false, true);
    return inner_->cancel();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if (path.header.frame_id != frame_ || path.poses.empty()) {
      throw nav2_core::ControllerException("ZoneAwareController: path frame or size is invalid");
    }
    inner_->setPlan(path);
    std::vector<Pose2> poses;
    poses.reserve(path.poses.size());
    for (const auto & ps : path.poses) {
      const auto & o = ps.pose.orientation;
      poses.push_back({ps.pose.position.x, ps.pose.position.y,
          std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))});
    }
    if (plan_goal_ && (std::hypot(poses.back().x - plan_goal_->x, poses.back().y - plan_goal_->y) > 0.01 ||
      std::abs(normAngle(poses.back().yaw - plan_goal_->yaw)) > 0.01)) {tracker_.resetProgress();}
    // A newly committed, shaped path may carry a different escape from the
    // current footprint. Do not keep a stale target/allowance from its predecessor.
    escape_target_.reset();
    plan_goal_ = poses.back();
    tracker_.setPath(poses);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    const auto & q = pose.pose.orientation;
    const Pose2 robot{pose.pose.position.x, pose.pose.position.y,
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    const TrackerOutput o = tracker_.compute(robot, v_cap_);
    track(o);
    publishMode(o.profile, o.weight > 0.0 && o.zone_mode);
    publishTracker(robot, o, pose.header.stamp);
    try {
      if (auto escape = escapeCommand(robot, velocity, pose.header)) {
        publishPhase(o.phase, o.zone_id);
        capSpeed(escape->twist);
        return *escape;
      }
    } catch (const nav2_core::ControllerException &) {
      // Escape failures need a new plan too. Publishing the nominal TRAVERSE
      // heartbeat here otherwise leaves the BT holding an unusable path.
      publishPhase(ZonePhase::Blocked, o.zone_id);
      throw;
    }
    if (o.weight <= 0.0) {
      publishPhase(o.phase, o.zone_id);
      blocked_since_.reset();
      auto cmd = inner_->computeVelocityCommands(pose, velocity, goal_checker);
      capSpeed(cmd.twist);
      return cmd;
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.twist.linear.x = cmd.twist.linear.y = cmd.twist.angular.z = 0.0;
    double w = o.weight;
    // ALIGN and on: the tracker alone (MPPI rollouts would hit the 1-cm walls)
    if (w < align_weight_) {
      try {
        cmd = inner_->computeVelocityCommands(pose, velocity, goal_checker);
      } catch (const nav2_core::ControllerException & e) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "ZoneAwareController: inner controller failed in the "
          "blend zone (%s), tracker only", e.what());
        w = 1.0;
        // cmd.twist already zeroed, safe to blend
      }
    }
    auto & t = cmd.twist;
    t.linear.x = (1.0 - w) * t.linear.x + w * o.vx;
    t.linear.y = (1.0 - w) * t.linear.y + w * o.vy;
    t.angular.z = (1.0 - w) * t.angular.z + w * o.wz;
    if (trace_) {
      RCLCPP_INFO(logger_, "ZTRACE %s ph %s s %+.3f w %.2f pose (%.3f %.3f %.1f) ref %.1f e_lat %+.3f e_yaw %+.1f "
        "trk (%+.2f %+.2f %+.2f) out (%+.2f %+.2f %+.2f)%s", o.zone_id.c_str(), zonePhaseName(o.phase), o.s_rel, w,
        robot.x, robot.y, robot.yaw * 180.0 / M_PI, o.yaw_ref * 180.0 / M_PI, o.e_lat, o.e_yaw * 180.0 / M_PI,
        o.vx, o.vy, o.wz, t.linear.x, t.linear.y, t.angular.z, o.gated ? " GATED" : "");
    }
    // the tracker does not see the costmap: check the path ahead (stopping distance + margin) with the
    // footprint the shaper used, and stop in front of anything lethal.  A long block fails the controller so
    // the behaviour tree recovers (replan from where the robot stands).
    bool blocked = w > 0.0 && obstacleAhead(robot, cmd.twist, velocity, o.v_ref);
    if (blocked) {
      // A full-speed lateral/turning correction can overshoot a narrow corridor
      // in the braking model even though a slower correction is collision-free.
      // Recheck the same complete footprint, present momentum and reference path
      // for every candidate; an obstacle in the required corridor still stops us.
      const auto requested = cmd.twist;
      for (double scale : {0.75, 0.5, 0.25, 0.125}) {
        auto candidate = requested;
        candidate.linear.x *= scale;
        candidate.linear.y *= scale;
        candidate.angular.z *= scale;
        if (!obstacleAhead(robot, candidate, velocity, o.v_ref)) {
          cmd.twist = candidate;
          blocked = false;
          break;
        }
      }
      // Scaling forward and lateral motion together preserves an unsafe
      // approach angle. Hold longitudinal advance and align to the valid path
      // first; otherwise a small tracking error parks forever before a wall
      // corner even though a collision-free sideways correction is available.
      if (blocked) {
        for (double scale : {1.0, 0.5, 0.25, 0.125}) {
          auto candidate = requested;
          candidate.linear.x = w * o.align_vx * scale;
          candidate.linear.y = w * o.align_vy * scale;
          candidate.angular.z *= scale;
          if (std::hypot(candidate.linear.x, candidate.linear.y) < .005 &&
            std::abs(candidate.angular.z) < .01) {continue;}
          if (!obstacleAhead(robot, candidate, velocity, o.v_ref)) {
            cmd.twist = candidate;
            blocked = false;
            break;
          }
        }
      }
    }
    if (blocked) {
      publishPhase(ZonePhase::Blocked, o.zone_id);
      t.linear.x = t.linear.y = t.angular.z = 0.0;
      const rclcpp::Time now = clock_->now();
      if (!blocked_since_) {
        blocked_since_ = now;
        RCLCPP_WARN(logger_, "ZoneAwareController: %s obstacle on the path ahead, stopping", o.zone_id.c_str());
      } else if ((now - *blocked_since_).seconds() > block_timeout_) {
        blocked_since_.reset();
        throw nav2_core::NoValidControl("ZoneAwareController: zone path blocked by an obstacle");
      }
    } else {
      publishPhase(o.phase, o.zone_id);
      blocked_since_.reset();
    }
    capSpeed(cmd.twist);
    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_->setSpeedLimit(speed_limit, percentage);
    if (speed_limit == 0.0) {
      v_cap_ = std::numeric_limits<double>::infinity();   // NO_SPEED_LIMIT
    } else {
      v_cap_ = std::isfinite(speed_limit) && speed_limit > 0.0 ?
        (percentage ? speed_limit_reference_ * speed_limit / 100.0 : speed_limit) : 0.0;
    }
  }

private:
  std::optional<geometry_msgs::msg::TwistStamped> escapeCommand(
    const Pose2 & robot, const geometry_msgs::msg::Twist & measured, const std_msgs::msg::Header & header)
  {
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto current = footprintContacts(*map, check_footprint_, robot);
    unsigned center_x, center_y;
    const bool center_free = map->worldToMap(robot.x, robot.y, center_x, center_y) &&
      map->getCost(center_x, center_y) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    if (current.valid && current.cells.empty() && center_free && !escape_target_) {return std::nullopt;}
    if (!current.valid) {throw nav2_core::NoValidControl("Escape footprint contains unknown/out-of-map space");}
    if (!escape_target_) {
      const auto exits = escapePaths(*map, check_footprint_, robot, plan_goal_.value_or(robot), escape_max_distance_);
      if (exits.empty()) {throw nav2_core::NoValidControl("No collision-decreasing translation escape");}
      escape_target_ = exits.front().poses.back();
      escape_contacts_ = escapeAllowance(*map, current, robot, escape_max_distance_);
      RCLCPP_INFO(logger_, "ZoneAwareController: occupied footprint, escaping at %.2f m/s without rotation", escape_speed_);
    }
    if (!contactsDecrease(escape_contacts_, current)) {
      throw nav2_core::NoValidControl("Escape stopped: new obstacle contact or growing overlap");
    }
    escape_contacts_.area = current.area;
    const auto target = *escape_target_;
    const double dx = target.x - robot.x, dy = target.y - robot.y, distance = std::hypot(dx, dy);
    if (current.cells.empty() && distance < 0.03) {
      escape_target_.reset();
      inner_->reset();
      RCLCPP_INFO(logger_, "ZoneAwareController: footprint clear, resuming normal path tracking");
      return std::nullopt;
    }
    // Recheck the entire remaining escape against the live map on every cycle.
    // The original component allowance is frozen across cycles and rolling-map shifts.
    if (std::hypot(target.x - robot.x, target.y - robot.y) > escape_max_distance_ + 0.05 ||
      !escapeMotionFree(*map, check_footprint_, robot, {target.x, target.y, robot.yaw}, &escape_contacts_)) {
      throw nav2_core::NoValidControl("Escape corridor changed or became blocked");
    }
    geometry_msgs::msg::TwistStamped cmd; cmd.header = header;
    // First brake old rotational/cross-track momentum instead of applying another
    // motion while already overlapping. Collision monitor and cmd_gate remain in series.
    const double cy = std::cos(robot.yaw), sy = std::sin(robot.yaw);
    const double mx = cy * measured.linear.x - sy * measured.linear.y;
    const double my = sy * measured.linear.x + cy * measured.linear.y;
    const double along = (mx * dx + my * dy) / std::max(distance, 1e-6);
    const double across = std::abs(mx * dy - my * dx) / std::max(distance, 1e-6);
    if (std::abs(measured.angular.z) > 0.05 || along < -0.02 || across > 0.03 ||
      std::hypot(mx, my) > escape_speed_ + 0.05) {return cmd;}
    const double speed = std::min(escape_speed_, 1.5 * distance);
    cmd.twist.linear.x = speed * (cy * dx + sy * dy) / std::max(distance, 1e-6);
    cmd.twist.linear.y = speed * (-sy * dx + cy * dy) / std::max(distance, 1e-6);
    return cmd;
  }

  void capSpeed(geometry_msgs::msg::Twist & cmd) const
  {
    const double speed = std::hypot(cmd.linear.x, cmd.linear.y);
    if (speed > v_cap_) {
      cmd.linear.x *= v_cap_ / speed;
      cmd.linear.y *= v_cap_ / speed;
    }
  }

  // Complete footprint checks along the reference path and both braking trajectories.
  bool obstacleAhead(const Pose2 & robot, const geometry_msgs::msg::Twist & command,
    const geometry_msgs::msg::Twist & measured, double reference)
  {
    if (!costmap_ros_) {return true;}
    const double decel = std::min(tracker_.params().decel, tracker_.params().ramp_decel);
    const double speed = std::max(std::hypot(command.linear.x, command.linear.y),
      std::hypot(measured.linear.x, measured.linear.y));
    if (!std::isfinite(speed)) {return true;}
    const double dist = speed * speed / (2.0 * decel) + speed * 0.2 + check_margin_;
    const auto poses = tracker_.ahead(reference < 0.0 ? -dist : dist);
    auto * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    auto motionFree = [&](const Pose2 & a, const Pose2 & b, const char * source) {
        Pose2 hit;
        if (footprintMotionFree(*costmap, check_footprint_, a, b, &hit)) {return true;}
        const auto contacts = footprintContacts(*costmap, check_footprint_, hit);
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
          "Zone collision: %s at (%.3f %.3f %.1f deg), robot (%.3f %.3f), occupied cells %zu",
          source, hit.x, hit.y, hit.yaw * 180.0 / M_PI, robot.x, robot.y, contacts.cells.size());
        return false;
      };
    Pose2 previous = robot;
    for (const auto & q : poses) {
      if (!motionFree(previous, q, "reference path")) {return true;}
      previous = q;
    }
    // Check the actual lateral/rotating/backing command as well as the nominal
    // path. Check both present momentum and the new command through braking.
    size_t trajectory = 0;
    for (const auto & twist : {measured, command}) {
      const char * source = trajectory++ == 0 ? "measured braking" : "command braking";
      double vx = twist.linear.x, vy = twist.linear.y, wz = twist.angular.z;
      if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {return true;}
      Pose2 q = robot;
      constexpr double dt = 0.05;
      const double horizon = 0.2 + std::max(std::hypot(vx, vy) / decel, std::abs(wz) / 4.0) + dt;
      for (double time = 0; time < horizon; time += dt) {
        previous = q;
        const double mid = q.yaw + 0.5 * wz * dt;
        q.x += (std::cos(mid) * vx - std::sin(mid) * vy) * dt;
        q.y += (std::sin(mid) * vx + std::cos(mid) * vy) * dt;
        q.yaw = normAngle(q.yaw + wz * dt);
        if (!motionFree(previous, q, source)) {return true;}
        if (time >= 0.2) {
          const double v = std::hypot(vx, vy);
          const double scale = v > 0.0 ? std::max(0.0, v - decel * dt) / v : 0.0;
          vx *= scale; vy *= scale;
          wz = std::copysign(std::max(0.0, std::abs(wz) - 4.0 * dt), wz);
        }
      }
    }
    return false;
  }

  // zone boxes (body), mouth lines and axis arrows; latched, sent once on activation
  void publishZones()
  {
    if (!zones_pub_) {
      return;
    }
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    int id = 0;
    auto pt = [](const Pose2 & q) {
        geometry_msgs::msg::Point p;
        p.x = q.x;
        p.y = q.y;
        p.z = 0.02;
        return p;
      };
    for (const auto & t : zones_) {
      const double hd = 0.5 * t.depth, hw = 0.5 * t.width;
      const bool open = t.profile == ZoneProfile::OpenDirectional;
      visualization_msgs::msg::Marker box;
      box.header.frame_id = frame_;
      box.ns = open ? "directional_body" : "narrow_body";
      box.id = id++;
      box.type = visualization_msgs::msg::Marker::LINE_STRIP;
      box.scale.x = 0.02;
      box.color.r = open ? 1.0f : 1.0f;
      box.color.g = open ? 1.0f : 0.6f;
      box.color.b = open ? 0.2f : 0.0f;
      box.color.a = 1.0f;
      box.pose.orientation.w = 1.0;
      for (const auto & uv : {std::pair{-hd, -hw}, {hd, -hw}, {hd, hw}, {-hd, hw}, {-hd, -hw}}) {
        box.points.push_back(pt(t.toMap(uv.first, uv.second, 0.0)));
      }
      arr.markers.push_back(box);
      // axis through the zone, 1.5 m beyond each mouth (where the shaper puts the path); a slope arrow
      // points uphill
      visualization_msgs::msg::Marker ax = box;
      ax.ns = open ? "directional_axis" : "narrow_axis";
      ax.id = id++;
      ax.type = visualization_msgs::msg::Marker::LINE_LIST;
      ax.scale.x = 0.01;
      ax.color.r = 0.2f;
      ax.color.g = 0.8f;
      ax.color.b = 1.0f;
      if (open) {
        ax.points = {pt(t.toMap(0.0, 0.0, 0.0)), pt(t.toMap(hd + 0.6, 0.0, 0.0))};
      } else {
        ax.points = {pt(t.toMap(-hd - 1.5, 0.0, 0.0)), pt(t.toMap(hd + 1.5, 0.0, 0.0))};
      }
      arr.markers.push_back(ax);
      visualization_msgs::msg::Marker label = box;
      label.ns = open ? "directional_id" : "narrow_id";
      label.id = id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.points.clear();
      label.pose.position = pt(t.toMap(0.0, hw + 0.15, 0.0));
      label.pose.position.z = 0.3;
      label.scale.z = 0.15;
      label.color.r = label.color.g = label.color.b = 1.0f;
      label.text = t.id;
      arr.markers.push_back(label);
    }
    zones_pub_->publish(arr);
  }

  // at the robot: tracker phase / weight / errors as text, the heading reference as an arrow, red when gated
  void publishTracker(const Pose2 & r, const TrackerOutput & o, const builtin_interfaces::msg::Time & stamp)
  {
    if (!viz_pub_ || !viz_pub_->is_activated()) {
      return;
    }
    const bool on = o.weight > 0.0;
    if (!on && !viz_on_) {
      return;   // nothing shown, nothing to clear
    }
    viz_on_ = on;
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker text;
    text.header.frame_id = frame_;
    text.header.stamp = stamp;
    text.ns = "tracker_state";
    text.id = 0;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = on ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    text.pose.position.x = r.x;
    text.pose.position.y = r.y + 0.55;
    text.pose.position.z = 0.5;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.12;
    text.color.a = 1.0f;
    text.color.r = 1.0f;
    text.color.g = o.gated ? 0.3f : 1.0f;
    text.color.b = o.gated ? 0.3f : 1.0f;
    char buf[200];
    std::snprintf(buf, sizeof(buf), "%s %s %s  w %.2f  s %+.2f\nlat %+.3f m  yaw %+.1f deg  v %.2f%s%s",
      o.zone_id.c_str(), zoneProfileName(o.profile), zonePhaseName(o.phase), o.weight, o.s_rel, o.e_lat,
      o.e_yaw * 180.0 / M_PI, o.v_ref, o.gated ? "  GATE" : "", o.backing_off ? "  BACK" : "");
    text.text = buf;
    arr.markers.push_back(text);
    visualization_msgs::msg::Marker ref = text;
    ref.ns = "tracker_yaw_ref";
    ref.type = visualization_msgs::msg::Marker::ARROW;
    ref.text.clear();
    ref.pose.position.x = r.x;
    ref.pose.position.y = r.y;
    ref.pose.position.z = 0.05;
    ref.pose.orientation.z = std::sin(0.5 * o.yaw_ref);
    ref.pose.orientation.w = std::cos(0.5 * o.yaw_ref);
    ref.scale.x = 0.5;
    ref.scale.y = 0.04;
    ref.scale.z = 0.04;
    ref.color.r = o.gated ? 1.0f : 0.1f;
    ref.color.g = o.gated ? 0.2f : 1.0f;
    ref.color.b = 0.2f;
    arr.markers.push_back(ref);
    viz_pub_->publish(arr);
  }

  void publishPhase(ZonePhase p, const std::string & zone, bool force = false)
  {
    if (!phase_pub_) {
      return;
    }
    const rclcpp::Time now = clock_->now();
    const bool changed = p != phase_ || zone != phase_zone_;
    // heartbeat: the BT condition expires without a fresh message, and a phase that never changes
    // (a long traverse) must not look like a stopped controller
    if (!force && !changed && (now - phase_stamp_).seconds() < heartbeat_) {
      return;
    }
    phase_ = p;
    phase_zone_ = zone;
    phase_stamp_ = now;
    std_msgs::msg::String msg;
    char st[32];
    std::snprintf(st, sizeof(st), " @%.3f", now.seconds());
    msg.data = std::string(zonePhaseName(p)) + (zone.empty() ? "" : " " + zone) + st;
    phase_pub_->publish(msg);
  }

  // "tunnel" / "ramp" / "normal": cmd_gate applies the matching speed limit
  void publishMode(ZoneProfile profile, bool in_zone, bool force = false)
  {
    const std::string m = in_zone ? zoneModeName(profile) : "normal";
    if (!mode_pub_ || (!force && m == mode_)) {
      return;
    }
    mode_ = m;
    std_msgs::msg::String msg;
    msg.data = m;
    mode_pub_->publish(msg);
  }

  // per-crossing log: window entry, gate holds, body statistics at the window exit
  void track(const TrackerOutput & o)
  {
    if (o.weight > 0.0 && cur_ != o.zone_id) {
      cur_ = o.zone_id;
      max_lat_ = max_yaw_ = 0.0;
      min_v_ = std::numeric_limits<double>::infinity();
      gate_logged_ = false;
      RCLCPP_INFO(logger_, "ZoneAwareController: %s (%s) window, heading locked at %.1f deg, lateral %+.3f m",
        cur_.c_str(), zoneProfileName(o.profile), o.lock * 180.0 / M_PI, o.e_lat);
    }
    if (o.weight <= 0.0 && !cur_.empty()) {
      RCLCPP_INFO(logger_, "ZoneAwareController: %s done, in body max |lateral| %.3f m, max |heading| %.2f deg, "
        "min speed %.2f m/s", cur_.c_str(), max_lat_, max_yaw_ * 180.0 / M_PI, min_v_);
      cur_.clear();
    }
    if (o.gated && !gate_logged_) {
      gate_logged_ = true;
      RCLCPP_WARN(logger_, "ZoneAwareController: %s gate: lateral %+.3f m, heading %+.2f deg, slowing down",
        o.zone_id.c_str(), o.e_lat, o.e_yaw * 180.0 / M_PI);
    }
    if (o.in_body) {
      max_lat_ = std::max(max_lat_, std::abs(o.e_lat));
      max_yaw_ = std::max(max_yaw_, std::abs(o.e_yaw));
      min_v_ = std::min(min_v_, o.v_ref);
    }
  }

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr inner_;
  ZoneTracker tracker_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_pub_, phase_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_, viz_pub_;
  std::vector<Zone> zones_;
  std::string frame_{"map"};
  bool viz_on_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("ZoneAwareController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string mode_, cur_, phase_zone_;
  ZonePhase phase_{ZonePhase::None};
  rclcpp::Time phase_stamp_{0, 0, RCL_ROS_TIME};
  double heartbeat_{0.5};
  double align_weight_{1.0};
  double v_cap_{std::numeric_limits<double>::infinity()};
  double speed_limit_reference_{1.5};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::vector<geometry_msgs::msg::Point> check_footprint_;
  std::optional<Pose2> plan_goal_, escape_target_;
  ContactState escape_contacts_;
  double escape_speed_{0.15}, escape_max_distance_{1.5};
  double check_margin_{0.3}, block_timeout_{5.0};
  bool trace_{false};
  std::optional<rclcpp::Time> blocked_since_;
  double max_lat_{0.0}, max_yaw_{0.0}, min_v_{0.0};
  bool gate_logged_{false};
};

}  // namespace nav_behaviors

PLUGINLIB_EXPORT_CLASS(nav_behaviors::ZoneAwareController, nav2_core::Controller)
