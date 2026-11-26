// A local XY path reference, independent of chassis/goal orientation.
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace nav_behaviors
{
struct XY {double x{0.0}, y{0.0};};
inline XY translationReference(
  const std::vector<XY> & path, XY robot, double lookahead, double gain, double max_speed,
  double max_path_deviation = 0.0)
{
  if (path.empty()) {return {};}
  XY target = path.back();
  double best = std::numeric_limits<double>::infinity();
  size_t segment = 0;
  double fraction = 0.0;
  XY projected = path.front();
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const auto a = path[i], b = path[i + 1];
    const double dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
    const double u = length2 > 1e-12 ?
      std::clamp(((robot.x - a.x) * dx + (robot.y - a.y) * dy) / length2, 0.0, 1.0) : 0.0;
    const double ex = a.x + u * dx - robot.x, ey = a.y + u * dy - robot.y;
    if (ex * ex + ey * ey < best) {
      best = ex * ex + ey * ey; segment = i; fraction = u;
      projected = {a.x + u * dx, a.y + u * dy};
    }
  }
  const auto chordFollowsPath = [&](XY endpoint, size_t last) {
    const double dx = endpoint.x - projected.x, dy = endpoint.y - projected.y;
    const double length2 = dx * dx + dy * dy;
    const double limit2 = max_path_deviation * max_path_deviation;
    for (size_t index = segment + 1; index <= last; ++index) {
      const double px = path[index].x - projected.x, py = path[index].y - projected.y;
      const double ratio = length2 > 1e-12 ?
        std::clamp((px * dx + py * dy) / length2, 0.0, 1.0) : 0.0;
      const double ex = px - ratio * dx, ey = py - ratio * dy;
      if (ex * ex + ey * ey > limit2) {return false;}
    }
    return true;
  };
  for (size_t i = segment; i + 1 < path.size(); ++i) {
    const auto a = path[i], b = path[i + 1];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double available = (1.0 - fraction) * length;
    if (length <= 1e-12) {fraction = 0.0; continue;}
    const auto pointAt = [&](double ratio) {
      return XY{a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y)};
    };
    const double end = fraction + std::min(lookahead, available) / length;
    target = pointAt(end);
    if (max_path_deviation > 0.0 && !chordFollowsPath(target, i)) {
      double low = fraction, high = end;
      for (int iteration = 0; iteration < 8; ++iteration) {
        const double mid = (low + high) * 0.5;
        if (chordFollowsPath(pointAt(mid), i)) {low = mid;} else {high = mid;}
      }
      target = pointAt(low);
      break;
    }
    if (lookahead <= available) {break;}
    lookahead -= available; fraction = 0.0;
  }
  XY velocity{gain * (target.x - robot.x), gain * (target.y - robot.y)};
  const double scale = std::min(1.0, max_speed / std::max(1e-12, std::hypot(velocity.x, velocity.y)));
  return {velocity.x * scale, velocity.y * scale};
}
}  // namespace nav_behaviors
