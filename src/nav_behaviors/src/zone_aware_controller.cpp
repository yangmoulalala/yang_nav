// ZoneAwareController: nav2_core::Controller wrapping an inner controller (MPPI) with the TunnelTracker.
//
// Outside tunnel windows the inner controller drives alone.  In a window the command is
// (1 - w) * inner + w * tracker; at w = 1 the inner controller is not called (its rollouts would hit
// the 1-cm walls), in the blend ramps an inner failure falls back to the tracker.  The inner
// controller is configured with this plugin's name, so its parameters stay under <name>.*; the
// tracker's live under <name>.zone.*.  /nav_mode (latched) is 'tunnel' near and inside the body.
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_behaviors/tunnel_tracker.hpp"
#include "nav_behaviors/zones_yaml.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

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
    TrackerParams tp;
    auto dbl = [&](const std::string & k, double & v) {
        declare_parameter_if_not_declared(node, p + k, ParameterValue(v));
        node->get_parameter(p + k, v);
      };
    // declared before the inner controller exists, so its parameter callback never sees them
    declare_parameter_if_not_declared(node, p + "zones_file", ParameterValue(std::string()));
    declare_parameter_if_not_declared(node, p + "min_cells", ParameterValue(50));
    declare_parameter_if_not_declared(node, p + "inner_plugin",
      ParameterValue(std::string("nav2_mppi_controller::MPPIController")));
    dbl("window_before", tp.window_before);
    dbl("window_after", tp.window_after);
    dbl("blend", tp.blend);
    dbl("align_before", tp.align_before);
    dbl("tunnel_speed", tp.tunnel_speed);
    dbl("approach_speed", tp.approach_speed);
    dbl("decel", tp.decel);
    dbl("gate_dist", tp.gate_dist);
    dbl("front_extent", tp.front_extent);
    dbl("gate_stop", tp.gate_stop);
    dbl("lat_tol", tp.lat_tol);
    double yaw_tol_deg = tp.yaw_tol * 180.0 / M_PI;
    dbl("yaw_tol_deg", yaw_tol_deg);
    tp.yaw_tol = yaw_tol_deg * M_PI / 180.0;
    dbl("k_lat", tp.k_lat);
    dbl("max_lat_speed", tp.max_lat_speed);
    dbl("k_yaw", tp.k_yaw);
    dbl("max_wz", tp.max_wz);
    dbl("search_dist", tp.search_dist);
    tracker_.setParams(tp);
    std::string zones_file, inner_plugin;
    int min_cells = 50;
    node->get_parameter(p + "zones_file", zones_file);
    node->get_parameter(p + "min_cells", min_cells);
    node->get_parameter(p + "inner_plugin", inner_plugin);
    if (zones_file.empty()) {
      RCLCPP_WARN(logger_, "ZoneAwareController: zone.zones_file not set, inner controller only");
    } else {
      std::vector<std::string> dropped;
      const auto tunnels = loadTunnels(zones_file, min_cells, &dropped);
      for (const auto & d : dropped) {
        RCLCPP_WARN(logger_, "ZoneAwareController: dropping %s, label fragment", d.c_str());
      }
      tracker_.setTunnels(tunnels);
      RCLCPP_INFO(logger_, "ZoneAwareController: %zu tunnels from %s, tunnel speed %.2f m/s", tunnels.size(),
        zones_file.c_str(), tp.tunnel_speed);
    }
    inner_ = loader_.createSharedInstance(inner_plugin);
    inner_->configure(parent, name, tf, costmap_ros);
    RCLCPP_INFO(logger_, "ZoneAwareController: inner controller %s", inner_plugin.c_str());
    mode_pub_ = node->create_publisher<std_msgs::msg::String>("/nav_mode", rclcpp::QoS(1).reliable().transient_local());
  }

  void cleanup() override
  {
    if (inner_) {
      inner_->cleanup();
    }
    inner_.reset();
    mode_pub_.reset();
  }

  void activate() override
  {
    inner_->activate();
    mode_pub_->on_activate();
    publishMode(false, true);
  }

  void deactivate() override
  {
    publishMode(false, true);
    inner_->deactivate();
    mode_pub_->on_deactivate();
  }

  void reset() override
  {
    inner_->reset();
    tracker_.resetProgress();
  }

  bool cancel() override
  {
    return inner_->cancel();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    inner_->setPlan(path);
    std::vector<Pose2> poses;
    poses.reserve(path.poses.size());
    for (const auto & ps : path.poses) {
      const auto & o = ps.pose.orientation;
      poses.push_back({ps.pose.position.x, ps.pose.position.y,
          std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))});
    }
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
    publishMode(o.weight > 0.0 && o.tunnel_mode);
    if (o.weight <= 0.0) {
      return inner_->computeVelocityCommands(pose, velocity, goal_checker);
    }
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    double w = o.weight;
    if (w < 1.0) {
      try {
        cmd = inner_->computeVelocityCommands(pose, velocity, goal_checker);
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "ZoneAwareController: inner controller failed in the "
          "blend zone (%s), tracker only", e.what());
        w = 1.0;
      }
    }
    auto & t = cmd.twist;
    t.linear.x = (1.0 - w) * t.linear.x + w * o.vx;
    t.linear.y = (1.0 - w) * t.linear.y + w * o.vy;
    t.angular.z = (1.0 - w) * t.angular.z + w * o.wz;
    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  {
    inner_->setSpeedLimit(speed_limit, percentage);
    if (speed_limit <= 0.0) {
      v_cap_ = std::numeric_limits<double>::infinity();   // NO_SPEED_LIMIT
    } else {
      v_cap_ = percentage ? tracker_.params().approach_speed * speed_limit / 100.0 : speed_limit;
    }
  }

private:
  void publishMode(bool tunnel, bool force = false)
  {
    const std::string m = tunnel ? "tunnel" : "normal";
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
    if (o.weight > 0.0 && cur_ != o.tunnel_id) {
      cur_ = o.tunnel_id;
      max_lat_ = max_yaw_ = 0.0;
      min_v_ = std::numeric_limits<double>::infinity();
      gate_logged_ = false;
      RCLCPP_INFO(logger_, "ZoneAwareController: %s window, heading locked at %.1f deg, lateral %+.3f m",
        cur_.c_str(), o.lock * 180.0 / M_PI, o.e_lat);
    }
    if (o.weight <= 0.0 && !cur_.empty()) {
      RCLCPP_INFO(logger_, "ZoneAwareController: %s done, in body max |lateral| %.3f m, max |heading| %.2f deg, "
        "min speed %.2f m/s", cur_.c_str(), max_lat_, max_yaw_ * 180.0 / M_PI, min_v_);
      cur_.clear();
    }
    if (o.gated && !gate_logged_) {
      gate_logged_ = true;
      RCLCPP_WARN(logger_, "ZoneAwareController: %s gate: lateral %+.3f m, heading %+.2f deg, slowing down",
        o.tunnel_id.c_str(), o.e_lat, o.e_yaw * 180.0 / M_PI);
    }
    if (o.in_body) {
      max_lat_ = std::max(max_lat_, std::abs(o.e_lat));
      max_yaw_ = std::max(max_yaw_, std::abs(o.e_yaw));
      min_v_ = std::min(min_v_, o.v_ref);
    }
  }

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr inner_;
  TunnelTracker tracker_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Logger logger_{rclcpp::get_logger("ZoneAwareController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string mode_, cur_;
  double v_cap_{std::numeric_limits<double>::infinity()};
  double max_lat_{0.0}, max_yaw_{0.0}, min_v_{0.0};
  bool gate_logged_{false};
};

}  // namespace nav_behaviors

PLUGINLIB_EXPORT_CLASS(nav_behaviors::ZoneAwareController, nav2_core::Controller)
