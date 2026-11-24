// Ordinary chassis navigation controls reference X/Y only. Yaw is an observed
// disturbance, not a sampled actuator; action yaw is handled by chassis_comm.
#pragma once
#include "nav2_mppi_controller/motion_models.hpp"

namespace nav_behaviors
{
class TranslationMotionModel : public mppi::OmniMotionModel
{
public:
  void setResponse(double real, double imag) {real_ = real; imag_ = imag;}
  void predict(mppi::models::State & state) override
  {
    // Acceleration limits act on command axes. Convert the shared measured
    // initial velocity back to those axes, then predict actual XY for ALL
    // rollout costs, including footprint collision checks.
    const double norm2 = real_*real_ + imag_*imag_;
    for (size_t i = 0; i < state.vx.shape(0); ++i) {
      const double x = state.vx(i, 0), y = state.vy(i, 0);
      state.vx(i, 0) = (real_*x + imag_*y)/norm2;
      state.vy(i, 0) = (real_*y - imag_*x)/norm2;
    }
    mppi::OmniMotionModel::predict(state);
    for (size_t i = 0; i < state.vx.shape(0); ++i) {
      for (size_t j = 0; j < state.vx.shape(1); ++j) {
        const double x = state.vx(i, j), y = state.vy(i, j);
        state.vx(i, j) = real_*x - imag_*y;
        state.vy(i, j) = imag_*x + real_*y;
      }
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
};
}  // namespace nav_behaviors
