// Ordinary chassis navigation controls reference X/Y only. Yaw is an observed
// disturbance, not a sampled actuator; action yaw is handled by chassis_comm.
#pragma once
#include <optional>
#include "nav_behaviors/translation_reference.hpp"
#include "nav2_mppi_controller/motion_models.hpp"

namespace nav_behaviors
{
class TranslationMotionModel : public mppi::OmniMotionModel
{
public:
  void setResponse(double real, double imag) {real_ = real; imag_ = imag;}
  void setCommandStart(std::optional<XY> command) {command_start_ = command;}
  void setFixedFrameTranslation(bool enabled) {fixed_frame_translation_ = enabled;}
  bool fixedFrameTranslation() const {return fixed_frame_translation_;}
  void predict(mppi::models::State & state) override
  {
    const double norm2 = real_*real_ + imag_*imag_;
    const double yaw_step = fixed_frame_translation_ ? state.speed.angular.z * model_dt_ : 0.0;
    for (size_t i = 0; i < state.vx.shape(0); ++i) {
      const double x = state.speed.linear.x, y = state.speed.linear.y;
      state.vx(i, 0) = command_start_ ? command_start_->x : (real_*x + imag_*y)/norm2;
      state.vy(i, 0) = command_start_ ? command_start_->y : (real_*y - imag_*x)/norm2;
    }
    mppi::OmniMotionModel::predict(state);
    const auto steps = state.vx.shape(1);
    for (size_t j = 1; j < steps; ++j) {
      const double c = std::cos(j * yaw_step), s = std::sin(j * yaw_step);
      const double real = c * real_ + s * imag_, imag = c * imag_ - s * real_;
      for (size_t i = 0; i < state.vx.shape(0); ++i) {
        const double x = state.vx(i, j), y = state.vy(i, j);
        state.vx(i, j) = real * x - imag * y;
        state.vy(i, j) = imag * x + real * y;
      }
    }
    for (size_t i = 0; i < state.vx.shape(0); ++i) {
      state.vx(i, 0) = state.speed.linear.x;
      state.vy(i, 0) = state.speed.linear.y;
    }
    // Every candidate sees the same measured yaw rate over the horizon. In
    // particular, random cwz samples cannot invent a collision-free rotation.
    state.wz.fill(state.speed.angular.z);
    state.cwz.fill(0.0f);
  }

  void applyConstraints(mppi::models::ControlSequence & sequence) override
  {
    sequence.wz.fill(0.0f);
  }
private:
  double real_{1.0}, imag_{0.0};
  std::optional<XY> command_start_;
  bool fixed_frame_translation_{false};
};
}  // namespace nav_behaviors
