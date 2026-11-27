// Anchor imminent XY commands to local path feedback. Horizon-average path
// costs alone can postpone lateral corrections under an unmodelled response.
#include <cmath>
#include <stdexcept>
#include <vector>
#include "nav_behaviors/translation_motion_model.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{
class TranslationVelocityCritic : public CriticFunction
{
public:
  void initialize() override
  {
    auto get = parameters_handler_->getParamGetter(name_);
    get(weight_, "cost_weight", 200.0f);
    get(lookahead_, "lookahead", 0.25f);
    get(gain_, "gain", 1.0f);
    get(max_speed_, "max_speed", 0.25f);
    get(direction_lookahead_, "direction_lookahead", 0.0f);
    get(velocity_score_time_, "velocity_score_time", 0.0f);
    parameters_handler_->getParamGetter(parent_name_)(departure_limit_, "path_departure_limit", 0.0,
      ParameterType::Static);
    parameters_handler_->getParamGetter(parent_name_)(curve_accel_, "reference_curve_accel", 0.0,
      ParameterType::Static);
    for (const auto v : {weight_, lookahead_, gain_, max_speed_}) {
      if (!std::isfinite(v) || v <= 0.0f) {
        throw std::invalid_argument("TranslationVelocityCritic parameters must be positive and finite");
      }
    }
    if (!std::isfinite(direction_lookahead_) || direction_lookahead_ < 0.0f ||
      !std::isfinite(velocity_score_time_) || velocity_score_time_ < 0.0f)
    {
      throw std::invalid_argument("TranslationVelocityCritic direction and score horizons must be nonnegative and finite");
    }
    RCLCPP_INFO(logger_, "TranslationVelocityCritic: anchors imminent XY commands to local path feedback");
  }

  void score(CriticData & data) override
  {
    if (!enabled_ || data.path.x.size() == 0 || data.state.vx.shape(1) < 2) {return;}
    std::vector<nav_behaviors::XY> path;
    path.reserve(data.path.x.size());
    for (size_t i = 0; i < data.path.x.size(); ++i) {
      path.push_back({data.path.x(i), data.path.y(i)});
    }
    const auto & pose = data.state.pose.pose;
    const double yaw = tf2::getYaw(pose.orientation);
    const auto model = std::dynamic_pointer_cast<nav_behaviors::TranslationMotionModel>(data.motion_model);
    const auto desired = nav_behaviors::translationReference(
      path, {pose.position.x, pose.position.y}, lookahead_, gain_, max_speed_, direction_lookahead_,
      model && model->fixedFrameTranslation() && departure_limit_ > 0.0 ?
      nav_behaviors::pathDeparturePreview : 0.0, curve_accel_);
    const double step_offset = model && model->fixedFrameTranslation() ? 0.0 : 0.5;
    // Cost the first *predicted* commands, excluding the shared measured
    // velocity at index zero. Future passive turret yaw remains observable.
    const double steps = velocity_score_time_ > 0.0f ?
      std::round(velocity_score_time_ / data.model_dt) : 2.0;
    const size_t count = static_cast<size_t>(std::clamp(
        steps, 1.0, static_cast<double>(data.state.vx.shape(1) - 1)));
    for (size_t i = 0; i < data.state.vx.shape(0); ++i) {
      double cost = 0.0;
      for (size_t j = 1; j <= count; ++j) {
        const double angle = yaw + data.state.speed.angular.z * (j - step_offset) * data.model_dt;
        const double c = std::cos(angle), s = std::sin(angle);
        const double vx = data.state.vx(i, j), vy = data.state.vy(i, j);
        const double ex = c * vx - s * vy - desired.x;
        const double ey = s * vx + c * vy - desired.y;
        cost += ex * ex + ey * ey;
      }
      data.costs(i) += weight_ * cost / count;
    }
  }
private:
  double departure_limit_{0.0};
  double curve_accel_{0.0};
  float weight_{200.0f}, lookahead_{0.25f}, gain_{1.0f}, max_speed_{0.25f};
  float direction_lookahead_{0.0f}, velocity_score_time_{0.0f};
};
}  // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::TranslationVelocityCritic, mppi::critics::CriticFunction)
