// Ordinary navigation completes at rest in XY; action yaw is independent.
#pragma once
#include <cmath>
#include <limits>
#include <stdexcept>
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav_behaviors
{
class TranslationGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    const auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".xy_goal_tolerance", rclcpp::ParameterValue(.15));
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".trans_stopped_velocity", rclcpp::ParameterValue(.05));
    node->get_parameter(name + ".xy_goal_tolerance", xy_tolerance_);
    node->get_parameter(name + ".trans_stopped_velocity", stopped_velocity_);
    if (!std::isfinite(xy_tolerance_) || xy_tolerance_ <= 0.0 ||
      !std::isfinite(stopped_velocity_) || stopped_velocity_ <= 0.0)
    {
      throw std::invalid_argument("TranslationGoalChecker requires positive finite tolerances");
    }
  }

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query,
    const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist & velocity) override
  {
    const double dx = query.position.x - goal.position.x;
    const double dy = query.position.y - goal.position.y;
    // Recheck position on every tick: entering tolerance while still moving
    // must not latch success and permit completion after drifting outside it.
    return std::isfinite(dx) && std::isfinite(dy) &&
           std::isfinite(velocity.linear.x) && std::isfinite(velocity.linear.y) &&
           std::hypot(dx, dy) <= xy_tolerance_ &&
           std::hypot(velocity.linear.x, velocity.linear.y) <= stopped_velocity_;
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose,
    geometry_msgs::msg::Twist & velocity) override
  {
    const double unused = std::numeric_limits<double>::lowest();
    pose.position.x = pose.position.y = xy_tolerance_;
    pose.position.z = unused;
    pose.orientation.x = pose.orientation.y = pose.orientation.z = pose.orientation.w = unused;
    velocity.linear.x = velocity.linear.y = stopped_velocity_;
    velocity.linear.z = unused;
    velocity.angular.x = velocity.angular.y = velocity.angular.z = unused;
    return true;
  }

private:
  double xy_tolerance_{.15};
  double stopped_velocity_{.05};
};
}  // namespace nav_behaviors
