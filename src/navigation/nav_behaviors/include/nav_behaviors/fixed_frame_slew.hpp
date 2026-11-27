#pragma once

#include <cmath>
#include "nav_behaviors/translation_reference.hpp"

namespace nav_behaviors
{

inline XY rotateVelocity(XY velocity, double yaw)
{
  const double cosine = std::cos(yaw), sine = std::sin(yaw);
  return {cosine * velocity.x - sine * velocity.y, sine * velocity.x + cosine * velocity.y};
}

inline XY limitVelocityNorm(XY velocity, double limit)
{
  const double speed = std::hypot(velocity.x, velocity.y);
  if (speed <= limit) {return velocity;}
  return {velocity.x * (limit / speed), velocity.y * (limit / speed)};
}

inline XY slewVelocity(XY current, XY target, double accel, double decel, double dt)
{
  const XY delta{target.x - current.x, target.y - current.y};
  const double dot = delta.x * current.x + delta.y * current.y;
  const double travel2 = delta.x * delta.x + delta.y * delta.y;
  if (dt <= 0.0 || travel2 == 0.0) {return current;}
  const double distance = std::sqrt(travel2);
  const double braking_distance = dot < 0.0 ? std::min(distance, -dot / distance) : 0.0;
  const double brake_time = braking_distance / decel;
  const double step = dt <= brake_time ? decel * dt : braking_distance + accel * (dt - brake_time);
  if (step >= distance) {return target;}
  return {current.x + delta.x * (step / distance), current.y + delta.y * (step / distance)};
}

}  // namespace nav_behaviors
