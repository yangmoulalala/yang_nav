// Last stage before the chassis.
//
//   input   (remap, Twist)   navigation command (velocity_smoother / collision_monitor output)
//   output  (remap, Twist)   /cmd_vel
//   /backend_node/localization_state (String, transient local)  pose_trusted gate
//   /nav_mode (String, transient local)  normal|tunnel|ramp from ZoneAwareController; recovery reserved
//
// * Hold (zero) unless localization reports pose_trusted=true and the state is fresh.
// * Watchdog: MecanumDrive keeps executing the last command, so stale input becomes zero.
// * Per-mode speed limits.
// * When idle it sends zeros for idle_stop_time and then stays quiet, so teleop publishing
//   on the same output topic keeps working.
#include <memory>
#include <limits>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav2_msgs/msg/speed_limit.hpp>

#include "nav_safety/gate_logic.hpp"

namespace nav_safety
{

class CmdGate : public rclcpp::Node
{
public:
  CmdGate()
  : Node("cmd_gate")
  {
    const double rate = declare_parameter<double>("rate", 30.0);
    cfg_.input_timeout = declare_parameter<double>("input_timeout", cfg_.input_timeout);
    cfg_.state_timeout = declare_parameter<double>("state_timeout", cfg_.state_timeout);
    cfg_.require_trusted = declare_parameter<bool>("require_trusted", cfg_.require_trusted);
    cfg_.require_observation = declare_parameter<bool>("require_observation", false);
    cfg_.observation_timeout = declare_parameter<double>("observation_timeout", 0.5);
    const double reference_speed = declare_parameter<double>("speed_limit_reference", 1.5);
    observation_sub_ = create_subscription<std_msgs::msg::Header>(
      "/nav/observation", 5, [this](std_msgs::msg::Header::ConstSharedPtr m) {
        observation_stamp_ = rclcpp::Time(m->stamp);
        have_observation_ = true;
      });
    // SpeedFilter (nav2_costmap_2d) publishes /speed_limit as plain QoS(10): reliable, volatile.
    // A transient_local subscription here never matches it and the cap is silently dropped.
    speed_sub_ = create_subscription<nav2_msgs::msg::SpeedLimit>(
      "/speed_limit", rclcpp::QoS(10).reliable(),
      [this, reference_speed](nav2_msgs::msg::SpeedLimit::ConstSharedPtr m) {
        speed_limit_ = m->speed_limit == 0.0 ? std::numeric_limits<double>::infinity() :
          (m->percentage ? reference_speed * m->speed_limit / 100.0 : m->speed_limit);
        if (std::isnan(speed_limit_) || speed_limit_ < 0.0) {speed_limit_ = 0.0;}
      });
    idle_stop_time_ = declare_parameter<double>("idle_stop_time", 0.5);
    for (std::size_t i = 0; i < kModeCount; ++i) {
      const std::string base = std::string("limits.") + mode_name(static_cast<Mode>(i));
      cfg_.limits[i].max_linear =
        declare_parameter<double>(base + ".max_linear", cfg_.limits[i].max_linear);
      cfg_.limits[i].max_angular =
        declare_parameter<double>(base + ".max_angular", cfg_.limits[i].max_angular);
    }

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    state_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("state_topic", "/backend_node/localization_state"), latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        trusted_ = parse_trusted(m->data);
        state_stamp_ = now();
        have_state_ = true;
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("mode_topic", "/nav_mode"), latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        const auto mode = parse_mode(m->data);
        if (!mode) {
          RCLCPP_WARN(get_logger(), "unknown nav mode '%s', keeping %s", m->data.c_str(),
          mode_name(mode_));
          return;
        }
        if (*mode != mode_) {
          RCLCPP_INFO(get_logger(), "mode %s -> %s", mode_name(mode_), mode_name(*mode));
        }
        mode_ = *mode;
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "input", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        cmd_ = *m;
        cmd_stamp_ = now();
        have_cmd_ = true;
      });
    pub_ = create_publisher<geometry_msgs::msg::Twist>("output", 10);
    idle_ticks_limit_ = std::max(1, static_cast<int>(idle_stop_time_ * rate));
    idle_ticks_ = idle_ticks_limit_;  // quiet at startup
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
  }

  ~CmdGate() override
  {
    if (pub_) {pub_->publish(geometry_msgs::msg::Twist());}
  }

private:
  void tick()
  {
    const rclcpp::Time t = now();
    GateInput in;
    in.have_cmd = have_cmd_;
    in.cmd_age = have_cmd_ ? (t - cmd_stamp_).seconds() : 0.0;
    in.have_state = have_state_;
    in.state_age = have_state_ ? (t - state_stamp_).seconds() : 0.0;
    in.trusted = trusted_;
    in.have_observation = have_observation_;
    in.observation_age = have_observation_ ? (t - observation_stamp_).seconds() : 0.0;

    std::string reason;
    const Decision d = decide(cfg_, in, reason);
    geometry_msgs::msg::Twist out;
    if (d == Decision::Idle) {
      if (idle_ticks_ < idle_ticks_limit_) {
        pub_->publish(out);
        ++idle_ticks_;
      }
      report("");
      return;
    }
    idle_ticks_ = 0;
    if (d == Decision::Pass) {
      auto limits = cfg_.limits[static_cast<int>(mode_)];
      limits.max_linear = std::min(limits.max_linear, speed_limit_);
      const Cmd c = limit({cmd_.linear.x, cmd_.linear.y, cmd_.angular.z}, limits);
      out.linear.x = c.vx;
      out.linear.y = c.vy;
      out.angular.z = c.wz;
    }
    report(reason);
    pub_->publish(out);
  }

  void report(const std::string & reason)
  {
    if (reason == last_reason_) {return;}
    if (!reason.empty()) {
      RCLCPP_WARN(get_logger(), "holding: %s", reason.c_str());
    } else if (!last_reason_.empty()) {
      RCLCPP_INFO(get_logger(), "released");
    }
    last_reason_ = reason;
  }

  GateConfig cfg_;
  double idle_stop_time_{0.5};
  int idle_ticks_limit_{15};
  int idle_ticks_{15};
  Mode mode_{Mode::Normal};
  bool trusted_{false}, have_state_{false}, have_cmd_{false};
  bool have_observation_{false};
  double speed_limit_{std::numeric_limits<double>::infinity()};
  rclcpp::Time observation_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr observation_sub_;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_sub_;
  rclcpp::Time state_stamp_, cmd_stamp_;
  geometry_msgs::msg::Twist cmd_;
  std::string last_reason_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_, mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_safety::CmdGate>());
  rclcpp::shutdown();
  return 0;
}
