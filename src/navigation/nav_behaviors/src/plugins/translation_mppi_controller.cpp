// MPPI lifecycle adapter using the chassis' two controlled translation axes.
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav_behaviors/translation_motion_model.hpp"
#include "nav_behaviors/translation_response.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav_behaviors
{
class TranslationOptimizer : public mppi::Optimizer
{
public:
  void useTranslationModel()
  {
    // Dynamic parameter updates may replace the stock motion model. Restore
    // our model under the parameter lock before each optimization cycle.
    if (!std::dynamic_pointer_cast<TranslationMotionModel>(motion_model_)) {
      motion_model_ = std::make_shared<TranslationMotionModel>();
    }
    motion_model_->initialize(settings_.constraints, settings_.model_dt);
    critics_data_.motion_model = motion_model_;
    control_sequence_.wz.fill(0.0f);
    if (settings_.open_loop) {
      throw nav2_core::ControllerException(
        "TranslationMPPI requires open_loop=false to observe passive reference yaw");
    }
  }
  void setResponse(double real, double imag)
  {
    std::static_pointer_cast<TranslationMotionModel>(motion_model_)->setResponse(real, imag);
  }
};

class TranslationMPPIController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    costmap_ = costmap_ros;
    parameters_ = std::make_unique<mppi::ParametersHandler>(parent);
    optimizer_.initialize(parent, name, costmap_ros, parameters_.get());
    path_.initialize(parent, name, costmap_ros, tf, parameters_.get());
    optimizer_.useTranslationModel();
    node_ = parent;
    auto node = parent.lock();
    auto get = parameters_->getParamGetter(name);
    get(response_feedback_, "response_feedback", false);
    command_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(5), [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        final_command_ = {msg->linear.x, msg->linear.y};
        if (auto node = node_.lock()) {command_time_ = node->now().seconds();}
      });
    RCLCPP_INFO(parent.lock()->get_logger(),
      "TranslationMPPI: controls reference X/Y; predicts measured passive yaw; action yaw is external");
  }

  void cleanup() override
  {
    command_subscription_.reset();
    optimizer_.shutdown();
    parameters_.reset();
  }
  void activate() override {parameters_->start();}
  void deactivate() override {}
  void reset() override {optimizer_.reset(); response_.reset();}

  void setPlan(const nav_msgs::msg::Path & plan) override {path_.setPath(plan);}
  void setSpeedLimit(const double & speed, const bool & percentage) override
  {
    optimizer_.setSpeedLimit(speed, percentage);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * goal_checker) override
  {
    std::lock_guard<std::mutex> params_lock(*parameters_->getLock());
    if (!std::isfinite(speed.angular.z) || !std::isfinite(speed.linear.x) ||
      !std::isfinite(speed.linear.y)) {
      throw nav2_core::NoValidControl("Non-finite measured reference velocity");
    }
    auto plan = path_.transformPath(pose);
    const auto goal = path_.getTransformedGoal(pose.header.stamp);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(
      *costmap_->getCostmap()->getMutex());
    optimizer_.useTranslationModel();
    if (response_feedback_) {
      auto node = node_.lock();
      const double now = node->now().seconds();
      XY final_command;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (now-command_time_ < .2) {final_command = final_command_;}
      }
      response_.observe(now, final_command, {speed.linear.x, speed.linear.y},
        tf2::getYaw(pose.pose.orientation));
      optimizer_.setResponse(response_.real, response_.imag);
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
        "Translation response gain=%.3f angle=%.3f", std::hypot(response_.real, response_.imag),
        std::atan2(response_.imag, response_.real));
    } else {
      optimizer_.setResponse(1.0, 0.0);
    }
    auto cmd = optimizer_.evalControl(pose, speed, plan, goal.pose, goal_checker);
    // Also guard the final filter/output, independently of model constraints.
    cmd.twist.angular.z = 0.0;
    return cmd;
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::unique_ptr<mppi::ParametersHandler> parameters_;
  TranslationOptimizer optimizer_;
  mppi::PathHandler path_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  std::mutex command_mutex_;
  XY final_command_;
  double command_time_{-1.0};
  bool response_feedback_{false};
  TranslationResponse response_;
};
}  // namespace nav_behaviors
PLUGINLIB_EXPORT_CLASS(nav_behaviors::TranslationMPPIController, nav2_core::Controller)
