#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_behaviors/fixed_frame_slew.hpp"

namespace nav_behaviors
{

class FixedFrameVelocitySmoother : public nav2_util::LifecycleNode
{
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using CallbackReturn = nav2_util::CallbackReturn;

public:
  FixedFrameVelocitySmoother()
  : nav2_util::LifecycleNode("velocity_smoother", "")
  {
    declare_parameter("smoothing_frequency", 100.0);
    declare_parameter("velocity_timeout", 0.5);
    declare_parameter("heading_timeout", 0.1);
    declare_parameter("odom_topic", "/localization/odometry");
    declare_parameter("odom_frame", "map");
    declare_parameter("base_frame", "gimbal_nav");
    declare_parameter("max_velocity", std::vector<double>{1.5, 1.5, 2.5});
    declare_parameter("min_velocity", std::vector<double>{-1.5, -1.5, -2.5});
    declare_parameter("max_accel", std::vector<double>{0.9, 0.9, 4.0});
    declare_parameter("max_decel", std::vector<double>{-1.0, -1.0, -4.0});
  }

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    frequency_ = get_parameter("smoothing_frequency").as_double();
    velocity_timeout_ = get_parameter("velocity_timeout").as_double();
    heading_timeout_ = get_parameter("heading_timeout").as_double();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    const auto max_vel = get_parameter("max_velocity").as_double_array();
    const auto min_vel = get_parameter("min_velocity").as_double_array();
    const auto max_accel = get_parameter("max_accel").as_double_array();
    const auto max_decel = get_parameter("max_decel").as_double_array();
    const auto valid = [](const auto & values, double sign) {
        return values.size() == 3 && std::all_of(values.begin(), values.end(), [sign](double v) {
          return std::isfinite(v) && sign * v > 0.0;
        });
      };
    if (!std::isfinite(frequency_) || frequency_ <= 0.0 ||
      !std::isfinite(velocity_timeout_) || velocity_timeout_ <= 0.0 ||
      !std::isfinite(heading_timeout_) || heading_timeout_ <= 0.0 ||
      !valid(max_vel, 1.0) || !valid(min_vel, -1.0) ||
      !valid(max_accel, 1.0) || !valid(max_decel, -1.0) ||
      odom_frame_.empty() || base_frame_.empty())
    {
      RCLCPP_ERROR(get_logger(), "Invalid fixed-frame velocity smoother parameters");
      return CallbackReturn::FAILURE;
    }
    linear_limit_ = std::min({max_vel[0], max_vel[1], -min_vel[0], -min_vel[1]});
    accel_ = std::min(max_accel[0], max_accel[1]);
    decel_ = std::min(-max_decel[0], -max_decel[1]);
    angular_accel_ = max_accel[2];
    angular_decel_ = -max_decel[2];
    angular_min_ = min_vel[2];
    angular_max_ = max_vel[2];
    output_ = create_publisher<Twist>("cmd_vel_smoothed", 10);
    odometry_ = create_subscription<Odometry>(get_parameter("odom_topic").as_string(),
      rclcpp::SensorDataQoS().keep_last(1), [this](Odometry::ConstSharedPtr msg) {updateHeading(*msg);});
    input_ = create_subscription<Twist>("cmd_vel", rclcpp::QoS(1),
      [this](Twist::ConstSharedPtr msg) {updateCommand(*msg);});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    output_->on_activate();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), [this]() {tick();});
    createBond();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    stop();
    output_->on_deactivate();
    destroyBond();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    input_.reset();
    odometry_.reset();
    output_.reset();
    heading_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    stop();
    return CallbackReturn::SUCCESS;
  }

  struct Heading {double yaw, rate, stamp;};

  void updateHeading(const Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const double rate = msg.twist.twist.angular.z;
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id != base_frame_ ||
      !std::isfinite(norm) || norm < 1e-12 || !std::isfinite(rate))
    {
      heading_.reset();
      return;
    }
    heading_ = Heading{tf2::getYaw(q), rate, rclcpp::Time(msg.header.stamp).seconds()};
  }

  std::optional<double> yawAt(double now) const
  {
    if (!heading_) {return std::nullopt;}
    const double age = now - heading_->stamp;
    if (age < 0.0 || age > heading_timeout_) {return std::nullopt;}
    const double yaw = heading_->yaw + heading_->rate * age;
    return std::isfinite(yaw) ? std::optional<double>(yaw) : std::nullopt;
  }

  void updateCommand(const Twist & msg)
  {
    if (!output_->is_activated()) {return;}
    const double now = this->now().seconds();
    const auto yaw = yawAt(now);
    if (!yaw || !std::isfinite(msg.linear.x) || !std::isfinite(msg.linear.y) ||
      !std::isfinite(msg.angular.z))
    {
      stop(true);
      return;
    }
    target_ = rotateVelocity(limitVelocityNorm({msg.linear.x, msg.linear.y}, linear_limit_), *yaw);
    angular_target_ = std::clamp(msg.angular.z, angular_min_, angular_max_);
    command_time_ = now;
  }

  void tick()
  {
    if (!command_time_) {return;}
    const double now = this->now().seconds();
    const auto yaw = yawAt(now);
    const double age = now - *command_time_;
    if (!yaw || age < 0.0) {stop(); return;}
    const bool expired = age > velocity_timeout_;
    current_ = slewVelocity(current_, expired ? XY{} : target_, accel_, decel_, 1.0 / frequency_);
    angular_ = slewVelocity({angular_, 0.0}, {expired ? 0.0 : angular_target_, 0.0},
      angular_accel_, angular_decel_, 1.0 / frequency_).x;
    const auto local = rotateVelocity(current_, -*yaw);
    Twist msg;
    msg.linear.x = local.x;
    msg.linear.y = local.y;
    msg.angular.z = angular_;
    output_->publish(msg);
    if (expired && current_.x == 0.0 && current_.y == 0.0 && angular_ == 0.0) {
      command_time_.reset();
    }
  }

  void stop(bool force = false)
  {
    if ((force || command_time_) && output_ && output_->is_activated()) {output_->publish(Twist{});}
    command_time_.reset();
    current_ = {};
    angular_ = 0.0;
  }

  std::string odom_frame_, base_frame_;
  double frequency_, velocity_timeout_, heading_timeout_;
  double linear_limit_, accel_, decel_, angular_accel_, angular_decel_, angular_min_, angular_max_;
  double angular_{0.0}, angular_target_{0.0};
  XY current_, target_;
  std::optional<double> command_time_;
  std::optional<Heading> heading_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_;
  rclcpp::Subscription<Twist>::SharedPtr input_;
  rclcpp_lifecycle::LifecyclePublisher<Twist>::SharedPtr output_;
};

}  // namespace nav_behaviors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav_behaviors::FixedFrameVelocitySmoother>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
