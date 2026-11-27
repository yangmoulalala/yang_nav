#include <gtest/gtest.h>
#include "nav_behaviors/fixed_frame_slew.hpp"

TEST(FixedFrameSlew, RotationAloneDoesNotConsumeAccelerationBudget)
{
  using namespace nav_behaviors;
  const XY forward{.8, .2};
  for (double rate : {-3.15, 3.15}) {
    XY current{};
    const double dt = .01;
    for (int step = 1; step <= 200; ++step) {
      const XY previous = current;
      const double angle = rate * dt * step;
      current = slewVelocity(current, forward, .9, 1., dt);
      EXPECT_LE(std::hypot(current.x - previous.x, current.y - previous.y), .9 * dt + 1e-9);
      const auto recovered = rotateVelocity(rotateVelocity(current, -angle), angle);
      EXPECT_NEAR(recovered.x, current.x, 1e-9);
      EXPECT_NEAR(recovered.y, current.y, 1e-9);
    }
    EXPECT_DOUBLE_EQ(current.x, forward.x);
    EXPECT_DOUBLE_EQ(current.y, forward.y);
  }
}

TEST(FixedFrameSlew, UsesDecelerationUntilReversalThenAcceleration)
{
  using namespace nav_behaviors;
  EXPECT_NEAR(slewVelocity({.1, 0.}, {-.8, 0.}, .9, 1., .2).x, -.09, 1e-9);
  const auto stop = slewVelocity({.3, .4}, {}, .9, 1., .52);
  EXPECT_DOUBLE_EQ(stop.x, 0.);
  EXPECT_DOUBLE_EQ(stop.y, 0.);
  const auto limited = limitVelocityNorm({1.5, 1.5}, 1.5);
  EXPECT_NEAR(std::hypot(limited.x, limited.y), 1.5, 1e-9);
}
