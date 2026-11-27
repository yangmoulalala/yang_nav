// A local XY path reference, independent of chassis/goal orientation.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace nav_behaviors
{
struct XY {double x{0.0}, y{0.0};};
struct PathProjection
{
  double distance_squared{std::numeric_limits<double>::infinity()};
  size_t segment{0};
  double fraction{0.0};
};

inline PathProjection projectOntoPath(const std::vector<XY> & path, XY robot)
{
  PathProjection closest;
  if (path.size() == 1) {
    const double dx = path.front().x - robot.x, dy = path.front().y - robot.y;
    closest.distance_squared = dx * dx + dy * dy;
  }
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const auto a = path[i], b = path[i + 1];
    const double dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
    const double u = length2 > 1e-12 ?
      std::clamp(((robot.x - a.x) * dx + (robot.y - a.y) * dy) / length2, 0.0, 1.0) : 0.0;
    const double ex = a.x + u * dx - robot.x, ey = a.y + u * dy - robot.y;
    const double distance = ex * ex + ey * ey;
    if (distance < closest.distance_squared) {
      closest = {distance, i, u};
    }
  }
  return closest;
}

inline XY pathLookahead(const std::vector<XY> & path, PathProjection closest, double lookahead)
{
  double fraction = closest.fraction;
  for (size_t i = closest.segment; i + 1 < path.size(); ++i) {
    const auto a = path[i], b = path[i + 1];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double available = (1.0 - fraction) * length;
    if (lookahead <= available && length > 1e-12) {
      const double u = fraction + lookahead / length;
      return {a.x + u * (b.x - a.x), a.y + u * (b.y - a.y)};
    }
    lookahead -= available; fraction = 0.0;
  }
  return path.back();
}

inline XY translationReference(
  const std::vector<XY> & path, XY robot, double lookahead, double gain, double max_speed,
  double direction_lookahead = 0.0, double preview_seconds = 0.0, double lateral_accel = 0.0)
{
  if (path.empty()) {return {};}
  const auto closest = projectOntoPath(path, robot);
  const XY target = pathLookahead(path, closest, lookahead);
  double speed = std::min(max_speed, gain * std::hypot(target.x - robot.x, target.y - robot.y));
  const double span = std::min(lookahead, speed * preview_seconds);
  if (span > 1e-6 && lateral_accel > 0.0) {
    const XY start = pathLookahead(path, closest, 0.0);
    const XY mid = pathLookahead(path, closest, .5 * span);
    const XY end = pathLookahead(path, closest, span);
    const XY first{mid.x - start.x, mid.y - start.y}, second{end.x - mid.x, end.y - mid.y};
    const double bend = std::abs(std::atan2(first.x * second.y - first.y * second.x,
      first.x * second.x + first.y * second.y));
    const double separation = .5 * (std::hypot(first.x, first.y) + std::hypot(second.x, second.y));
    if (bend > 1e-6 && separation > 1e-6) {
      speed = std::min(speed, std::sqrt(lateral_accel * separation / bend));
    }
  }
  const double direction_distance = direction_lookahead > 0.0 ?
    std::min(lookahead, std::max(direction_lookahead, speed * preview_seconds)) : lookahead;
  const XY direction = pathLookahead(path, closest, direction_distance);
  const XY delta{direction.x - robot.x, direction.y - robot.y};
  const double norm = std::hypot(delta.x, delta.y);
  if (norm < 1e-12) {return {};}
  if (preview_seconds > 0.0) {speed = std::min(speed, norm / preview_seconds);}
  return {delta.x * speed / norm, delta.y * speed / norm};
}

inline constexpr double pathDeparturePreview = 0.35;

inline bool movesAwayFromPath(const std::vector<XY> & path, XY robot, XY world_velocity, double limit)
{
  if (path.empty() || limit <= 0.0) {return false;}
  // Deadband on the predicted offset growth. A flat 5 mm over the 0.35 s preview is
  // an effective 14 mm/s outward-speed limit, so any lateral component tripped it and
  // the controller threw NoValidControl instead of steering back. Scale it with the
  // offset so a robot that is already off-path can still converge without tripping.
  const double current = std::sqrt(projectOntoPath(path, robot).distance_squared);
  const double min_increase = std::max(0.02, 0.25 * current);
  const XY projected{robot.x + pathDeparturePreview * world_velocity.x,
    robot.y + pathDeparturePreview * world_velocity.y};
  const double future = std::sqrt(projectOntoPath(path, projected).distance_squared);
  return future > std::max(limit, current + min_increase);
}
}  // namespace nav_behaviors
