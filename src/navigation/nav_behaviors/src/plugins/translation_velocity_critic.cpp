// Anchor imminent XY commands to local path feedback. Horizon-average path
// costs alone can postpone lateral corrections under an unmodelled response.
#include <cmath>
#include <stdexcept>
#include <vector>
#include "nav_behaviors/translation_reference.hpp"
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
    get(max_path_deviation_, "max_path_deviation", 0.0f);
    get(gain_, "gain", 1.0f);
    get(max_speed_, "max_speed", 0.25f);
    for (const auto v : {weight_, lookahead_, gain_, max_speed_}) {
      if (!std::isfinite(v) || v <= 0.0f) {
        throw std::invalid_argument("TranslationVelocityCritic parameters must be positive and finite");
      }
    }
    if (!std::isfinite(max_path_deviation_) || max_path_deviation_ < 0.0f) {
      throw std::invalid_argument("TranslationVelocityCritic max_path_deviation must be finite and nonnegative");
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
    const auto desired = nav_behaviors::translationReference(
      path, {pose.position.x, pose.position.y}, lookahead_, gain_, max_speed_, max_path_deviation_);
    const double yaw = tf2::getYaw(pose.orientation);
    // Cost the first two *predicted* commands, excluding the shared measured
    // velocity at index zero. Future passive turret yaw remains observable.
    const size_t count = std::min<size_t>(2, data.state.vx.shape(1) - 1);
    for (size_t i = 0; i < data.state.vx.shape(0); ++i) {
      double cost = 0.0;
      for (size_t j = 1; j <= count; ++j) {
        const double angle = yaw + data.state.speed.angular.z * (j - 0.5) * data.model_dt;
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
  float weight_{200.0f}, lookahead_{0.25f}, max_path_deviation_{0.0f}, gain_{1.0f}, max_speed_{0.25f};
};
}  // namespace mppi::critics
PLUGINLIB_EXPORT_CLASS(mppi::critics::TranslationVelocityCritic, mppi::critics::CriticFunction)
