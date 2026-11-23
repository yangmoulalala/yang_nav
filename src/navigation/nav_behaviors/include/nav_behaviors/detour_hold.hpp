#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include "nav_msgs/msg/path.hpp"

namespace nav_behaviors
{
inline double pathDistance(const nav_msgs::msg::Path & path, size_t first = 0)
{
  double length = 0.;
  for (size_t i = first + 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i - 1].pose.position;
    const auto & b = path.poses[i].pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

// Keep a valid current route briefly when a replan suddenly makes a large detour.
// The live controller still stops for obstacles; persistent blockage can reroute.
class DetourHold
{
public:
  bool keepCurrent(const nav_msgs::msg::Path & current, const nav_msgs::msg::Path & candidate,
    double now)
  {
    if (current.poses.empty() || candidate.poses.empty() || current.header.frame_id != candidate.header.frame_id) {
      reset(); return false;
    }
    const auto & robot = candidate.poses.front().pose.position;
    size_t nearest = 0;
    double distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < current.poses.size(); ++i) {
      const auto & p = current.poses[i].pose.position;
      const double d = std::hypot(p.x - robot.x, p.y - robot.y);
      if (d < distance) {distance = d; nearest = i;}
    }
    const double remaining = distance + pathDistance(current, nearest);
    const double replacement = pathDistance(candidate);
    if (distance > .5 || replacement <= 2. * remaining || replacement - remaining <= 5.) {
      reset(); return false;
    }
    if (!since_ || now < *since_) {since_ = now;}
    return now - *since_ < 1.5;
  }
  void reset() {since_.reset();}
private:
  std::optional<double> since_;
};
}  // namespace nav_behaviors
