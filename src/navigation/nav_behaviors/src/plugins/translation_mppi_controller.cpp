// MPPI lifecycle adapter using the chassis' two controlled translation axes.
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav_behaviors/costmap_collision.hpp"
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
  void useTranslationModel(bool fixed_frame_translation)
  {
    // Dynamic parameter updates may replace the stock motion model. Restore
    // our model under the parameter lock before each optimization cycle.
    if (!std::dynamic_pointer_cast<TranslationMotionModel>(motion_model_)) {
      motion_model_ = std::make_shared<TranslationMotionModel>();
    }
    motion_model_->initialize(settings_.constraints, settings_.model_dt);
    std::static_pointer_cast<TranslationMotionModel>(motion_model_)->setFixedFrameTranslation(
      fixed_frame_translation);
    critics_data_.motion_model = motion_model_;
    control_sequence_.wz.fill(0.0f);
    if (settings_.open_loop) {
      throw nav2_core::ControllerException(
        "TranslationMPPI requires open_loop=false to observe passive reference yaw");
    }
  }
  void setResponse(double real, double imag, std::optional<XY> command_start)
  {
    auto model = std::static_pointer_cast<TranslationMotionModel>(motion_model_);
    model->setResponse(real, imag);
    model->setCommandStart(command_start);
  }

  bool permits(XY command) const
  {
    const auto & bounds = settings_.constraints;
    return command.x >= bounds.vx_min && command.x <= bounds.vx_max &&
           std::abs(command.y) <= bounds.vy;
  }

  void seedPathVelocity(XY world_target, const geometry_msgs::msg::Twist & measured,
    double yaw, double yaw_rate, double real, double imag, std::optional<XY> command_start)
  {
    const double norm_squared = real * real + imag * imag;
    const auto to_command = [&](XY velocity) {
        return XY{(real * velocity.x + imag * velocity.y) / norm_squared,
          (real * velocity.y - imag * velocity.x) / norm_squared};
      };
    const auto slew = [this](double current, double target, double accel, double decel) {
        const double lower = current >= 0.0 ? -decel : -accel;
        const double upper = current >= 0.0 ? accel : decel;
        return current + std::clamp(target - current,
          lower * settings_.model_dt, upper * settings_.model_dt);
      };
    XY current = command_start.value_or(to_command({measured.linear.x, measured.linear.y}));
    const auto & limits = settings_.constraints;
    for (size_t step = 0; step < control_sequence_.vx.size(); ++step) {
      const double angle = yaw + (step + 1) * settings_.model_dt * yaw_rate;
      const double cosine = std::cos(angle), sine = std::sin(angle);
      const auto target = to_command({cosine * world_target.x + sine * world_target.y,
        -sine * world_target.x + cosine * world_target.y});
      current.x = slew(current.x, std::clamp(target.x, double(limits.vx_min), double(limits.vx_max)),
        limits.ax_max, -limits.ax_min);
      current.y = slew(current.y, std::clamp(target.y, -double(limits.vy), double(limits.vy)),
        limits.ay_max, -limits.ay_min);
      control_sequence_.vx(step) = current.x;
      control_sequence_.vy(step) = current.y;
    }
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
    node_ = parent;
    auto node = parent.lock();
    auto get = parameters_->getParamGetter(name);
    get(response_feedback_, "response_feedback", false);
    get(fixed_frame_translation_, "fixed_frame_translation", false, mppi::ParameterType::Static);
    optimizer_.useTranslationModel(fixed_frame_translation_);
    get(path_departure_limit_, "path_departure_limit", 0.0, mppi::ParameterType::Static);
    get(curve_accel_, "reference_curve_accel", 0.0, mppi::ParameterType::Static);
    get(clear_reentry_, "clear_space_reentry", false, mppi::ParameterType::Static);
    get(reentry_timeout_, "reentry_timeout", 0.0, mppi::ParameterType::Static);
    if (!std::isfinite(curve_accel_) || curve_accel_ < 0.0) {
      throw std::invalid_argument("TranslationMPPI reference_curve_accel must be nonnegative and finite");
    }
    if (!std::isfinite(path_departure_limit_) || path_departure_limit_ < 0.0) {
      throw std::invalid_argument("TranslationMPPI path_departure_limit must be nonnegative and finite");
    }
    const auto reference_prefix = name + ".TranslationVelocityCritic.";
    reference_parameters_ = {reference_prefix + "lookahead", reference_prefix + "gain",
      reference_prefix + "max_speed", reference_prefix + "direction_lookahead"};
    std::string feedback_topic;
    get(feedback_topic, "executed_velocity_topic", std::string("/cmd_vel"), mppi::ParameterType::Static);
    command_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      feedback_topic, rclcpp::QoS(1), [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
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
    const auto node = node_.lock();
    const auto reference = path_departure_limit_ > 0.0 ?
      node->get_parameters(reference_parameters_) : std::vector<rclcpp::Parameter>();
    std::lock_guard<std::mutex> params_lock(*parameters_->getLock());
    if (!std::isfinite(speed.angular.z) || !std::isfinite(speed.linear.x) ||
      !std::isfinite(speed.linear.y)) {
      throw nav2_core::NoValidControl("Non-finite measured reference velocity");
    }
    auto plan = path_.transformPath(pose);
    const auto goal = path_.getTransformedGoal(pose.header.stamp);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(
      *costmap_->getCostmap()->getMutex());
    optimizer_.useTranslationModel(fixed_frame_translation_);
    std::optional<XY> command_start;
    if (response_feedback_ || path_departure_limit_ > 0.0 || fixed_frame_translation_) {
      const double now = node->now().seconds();
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        const double age = now - command_time_;
        if (age >= 0.0 && age < .10 && std::isfinite(final_command_.x) &&
          std::isfinite(final_command_.y))
        {
          command_start = final_command_;
        }
      }
      if (response_feedback_) {
        response_.observe(now, command_start.value_or(XY{}), {speed.linear.x, speed.linear.y},
          tf2::getYaw(pose.pose.orientation));
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
          "Translation response gain=%.3f angle=%.3f", std::hypot(response_.real, response_.imag),
          std::atan2(response_.imag, response_.real));
      }
    }
    std::vector<XY> path;
    const XY robot{pose.pose.position.x, pose.pose.position.y};
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double real = response_feedback_ ? response_.real : 1.0;
    const double imag = response_feedback_ ? response_.imag : 0.0;
    if (path_departure_limit_ <= 0.0 && !fixed_frame_translation_) {command_start.reset();}
    optimizer_.setResponse(real, imag, command_start);
    XY desired{};
    if (path_departure_limit_ > 0.0) {
      path.reserve(plan.poses.size());
      for (const auto & point : plan.poses) {
        path.push_back({point.pose.position.x, point.pose.position.y});
      }
      const double lookahead = reference[0].as_double();
      const double gain = reference[1].as_double();
      const double max_speed = reference[2].as_double();
      const double direction_lookahead = reference[3].as_double();
      if (!std::isfinite(lookahead) || lookahead <= 0.0 || !std::isfinite(gain) || gain <= 0.0 ||
        !std::isfinite(max_speed) || max_speed <= 0.0 || !std::isfinite(direction_lookahead) ||
        direction_lookahead < 0.0)
      {
        throw nav2_core::NoValidControl("TranslationMPPI: invalid path reference parameters");
      }
      desired = translationReference(path, robot, lookahead, gain, max_speed, direction_lookahead,
        fixed_frame_translation_ ? pathDeparturePreview : 0.0, curve_accel_);
      optimizer_.seedPathVelocity(desired,
        speed, yaw, fixed_frame_translation_ ? 0.0 : speed.angular.z, real, imag, command_start);
    }
    auto cmd = optimizer_.evalControl(pose, speed, plan, goal.pose, goal_checker);
    if (path_departure_limit_ > 0.0) {
      const double cosine = std::cos(yaw), sine = std::sin(yaw);
      const double vx = real * cmd.twist.linear.x - imag * cmd.twist.linear.y;
      const double vy = imag * cmd.twist.linear.x + real * cmd.twist.linear.y;
      if (movesAwayFromPath(path, robot,
        {cosine * vx - sine * vy, sine * vx + cosine * vy}, path_departure_limit_))
      {
        const double desired_norm = std::hypot(desired.x, desired.y);
        const double response_norm = std::hypot(real, imag);
        if (clear_reentry_ && fixed_frame_translation_ && command_start &&
          pose.header.frame_id == costmap_->getGlobalFrameID() && costmap_->isCurrent() &&
          desired_norm > .005 && response_norm >= .65 && response_norm <= 1.35)
        {
          const XY direction{desired.x / desired_norm, desired.y / desired_norm};
          const double forward = direction.x * (cosine * vx - sine * vy) +
            direction.y * (sine * vx + cosine * vy);
          const double retained = std::min(desired_norm, forward);
          const XY corrected{direction.x * retained, direction.y * retained};
          const XY local{cosine * corrected.x + sine * corrected.y, -sine * corrected.x + cosine * corrected.y};
          const double gain_squared = response_norm * response_norm;
          const XY output{(real * local.x + imag * local.y) / gain_squared,
            (real * local.y - imag * local.x) / gain_squared};
          const double bound = std::max({std::hypot(speed.linear.x, speed.linear.y),
            std::hypot(command_start->x, command_start->y) * std::max(1.0, response_norm),
            std::hypot(output.x, output.y) * std::max(1.0, response_norm)});
          if (retained > .005 && optimizer_.permits(output) &&
            !movesAwayFromPath(path, robot, corrected, path_departure_limit_) &&
            clearBrakingEnvelope(*costmap_->getCostmap(), costmap_->getRobotFootprint(),
            {robot.x, robot.y, yaw}, bound, curve_accel_, reentry_timeout_))
          {
            cmd.twist.linear.x = output.x;
            cmd.twist.linear.y = output.y;
            cmd.twist.angular.z = 0.0;
            RCLCPP_DEBUG(node->get_logger(), "TranslationMPPI: clear-space path correction");
            return cmd;
          }
        }
        optimizer_.reset(false);
        throw nav2_core::NoValidControl("TranslationMPPI: command would continue to depart from path");
      }
    }
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
  bool fixed_frame_translation_{false};
  double path_departure_limit_{0.0};
  double curve_accel_{0.0};
  bool clear_reentry_{false};
  double reentry_timeout_{0.0};
  std::vector<std::string> reference_parameters_;
  TranslationResponse response_;
};
}  // namespace nav_behaviors
PLUGINLIB_EXPORT_CLASS(nav_behaviors::TranslationMPPIController, nav2_core::Controller)
