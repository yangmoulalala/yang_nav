// Common math utilities for navigation geometry (no ROS dependency).
#pragma once

#include <cmath>
#include <algorithm>

namespace nav_behaviors
{

// Normalize angle to [-pi, pi]
inline double normAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Smoothstep function: maps [0, 1] -> [0, 1] with zero derivatives at ends
// https://en.wikipedia.org/wiki/Smoothstep
inline double smooth01(double w)
{
  w = std::clamp(w, 0.0, 1.0);
  return w * w * (3.0 - 2.0 * w);
}

// Derivative of smooth01
inline double dsmooth01(double w)
{
  return (w <= 0.0 || w >= 1.0) ? 0.0 : 6.0 * w * (1.0 - w);
}

// 2D vector length
inline double hypot2(double x, double y)
{
  return std::hypot(x, y);
}

// Clamp a value to [min, max]
template<typename T>
inline T clamp(T value, T min_val, T max_val)
{
  return std::clamp(value, min_val, max_val);
}

}  // namespace nav_behaviors
