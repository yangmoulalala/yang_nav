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
  const std::vector<XY> & path, XY robot, double lookahead, double gain, double max_speed)
{
  if (path.empty()) {return {};}
  XY target = path.back();
  double best = std::numeric_limits<double>::infinity();
  size_t segment = 0;
  double fraction = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const auto a = path[i], b = path[i + 1];
    const double dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
    const double u = length2 > 1e-12 ?
      std::clamp(((robot.x - a.x) * dx + (robot.y - a.y) * dy) / length2, 0.0, 1.0) : 0.0;
    const double ex = a.x + u * dx - robot.x, ey = a.y + u * dy - robot.y;
    if (ex * ex + ey * ey < best) {
      best = ex * ex + ey * ey; segment = i; fraction = u;
    }
  }
  for (size_t i = segment; i + 1 < path.size(); ++i) {
    const auto a = path[i], b = path[i + 1];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double available = (1.0 - fraction) * length;
    if (lookahead <= available && length > 1e-12) {
      const double u = fraction + lookahead / length;
      target = {a.x + u * (b.x - a.x), a.y + u * (b.y - a.y)};
      break;
    }
    lookahead -= available; fraction = 0.0;
  }
  XY velocity{gain * (target.x - robot.x), gain * (target.y - robot.y)};
  const double scale = std::min(1.0, max_speed / std::max(1e-12, std::hypot(velocity.x, velocity.y)));
  return {velocity.x * scale, velocity.y * scale};
}
}  // namespace nav_behaviors
