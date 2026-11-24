// Estimate a bounded planar gain/rotation from final commands and odometry.
// This is transient model feedback, never an installation/joint calibration.
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include "nav_behaviors/translation_reference.hpp"

namespace nav_behaviors
{
class TranslationResponse
{
public:
  void reset() {samples_.clear(); real = 1.0; imag = 0.0; last_ = -1.0;}
  void observe(double now, XY final_command, XY measured, double yaw)
  {
    const double dt = last_ < 0.0 ? 0.0 : now - last_;
    last_ = now;
    if (dt < 0.0 || dt > 0.3) {reset(); last_ = now;}
    const double c = std::cos(yaw), s = std::sin(yaw);
    auto world = [c, s](XY v) {return XY{c*v.x-s*v.y, s*v.x+c*v.y};};
    samples_.push_back({now, world(final_command)});
    while (!samples_.empty() && now - samples_.front().time > .5) {samples_.pop_front();}
    // Use a settled command window, spanning the serial/smoother delay and
    // odometry filtering. Do not identify acceleration or changing commands.
    XY mean{}; size_t count = 0;
    for (const auto & sample : samples_) {
      const double age = now - sample.time;
      if (age < .1 || age > .4) {continue;}
      mean.x += sample.velocity.x; mean.y += sample.velocity.y; ++count;
    }
    if (count < 4 || dt <= 0.0 || dt > .3) {return;}
    mean.x /= count; mean.y /= count;
    const double norm2 = mean.x*mean.x + mean.y*mean.y;
    if (norm2 < .08*.08) {return;}
    for (const auto & sample : samples_) {
      if (now - sample.time > .4) {continue;}
      if (std::hypot(sample.velocity.x-mean.x, sample.velocity.y-mean.y) > .035) {return;}
    }
    measured = world(measured);
    const double r = (measured.x*mean.x + measured.y*mean.y) / norm2;
    const double i = (measured.y*mean.x - measured.x*mean.y) / norm2;
    const double gain = std::hypot(r, i), angle = std::atan2(i, r);
    // Stopped/disconnected output, reversals and large discrepancies are not
    // evidence of an axis correction. Never learn those as an inverse model.
    if (!std::isfinite(gain) || gain < .7 || gain > 1.3 || std::abs(angle) > .524) {return;}
    const double alpha = 1.0 - std::exp(-dt / .35);
    real += alpha*(r-real); imag += alpha*(i-imag);
  }
  double real{1.0}, imag{0.0};
private:
  struct Sample {double time; XY velocity;};
  std::deque<Sample> samples_;
  double last_{-1.0};
};
}  // namespace nav_behaviors
