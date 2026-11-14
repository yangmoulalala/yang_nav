// Conservative convex-footprint checks, including cells wholly inside the footprint.
// Callers own the costmap mutex. Unknown and out-of-map space are collisions.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav_behaviors/math_utils.hpp"
#include "nav_behaviors/path_shaper.hpp"

namespace nav_behaviors
{

template<class Visitor>
inline bool visitFootprintCells(
  const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2 & pose, Visitor visit)
{
  if (footprint.size() < 3 || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
    !std::isfinite(pose.yaw)) {return false;}
  std::vector<geometry_msgs::msg::Point> polygon;
  nav2_costmap_2d::transformFootprint(pose.x, pose.y, pose.yaw, footprint, polygon);
  double xmin = std::numeric_limits<double>::infinity(), ymin = xmin;
  double xmax = -xmin, ymax = -xmin;
  for (const auto & p : polygon) {
    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
  }
  unsigned int x0, y0, x1, y1;
  if (!map.worldToMap(xmin, ymin, x0, y0) || !map.worldToMap(xmax, ymax, x1, y1)) {
    return false;
  }
  const double half = 0.5 * map.getResolution();
  for (unsigned int y = y0; y <= y1; ++y) {
    for (unsigned int x = x0; x <= x1; ++x) {
      if (map.getCost(x, y) < nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
      double cx, cy;
      map.mapToWorld(x, y, cx, cy);
      // Separating-axis test of the convex polygon against this occupied grid cell.
      bool separated = false;
      for (size_t i = 0; i < polygon.size(); ++i) {
        const auto & a = polygon[i];
        const auto & b = polygon[(i + 1) % polygon.size()];
        const double nx = a.y - b.y, ny = b.x - a.x;
        if (std::hypot(nx, ny) < 1e-12) {continue;}
        double lo = std::numeric_limits<double>::infinity(), hi = -lo;
        for (const auto & p : polygon) {
          const double v = nx * p.x + ny * p.y;
          lo = std::min(lo, v); hi = std::max(hi, v);
        }
        const double center = nx * cx + ny * cy, reach = half * (std::abs(nx) + std::abs(ny));
        if (hi < center - reach || lo > center + reach) {separated = true; break;}
      }
      if (!separated && cx + half >= xmin && cx - half <= xmax &&
        cy + half >= ymin && cy - half <= ymax && !visit(x, y, polygon)) {return false;}
    }
  }
  return true;
}

inline bool footprintFree(
  const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2 & pose)
{
  return visitFootprintCells(map, footprint, pose, [&](unsigned x, unsigned y, const auto &) {
      return map.getCost(x, y) < nav2_costmap_2d::LETHAL_OBSTACLE;
    });
}

inline bool footprintMotionFree(
  const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2 & a, const Pose2 & b,
  Pose2 * collision_pose = nullptr)
{
  double radius = 0.0;
  for (const auto & p : footprint) {radius = std::max(radius, std::hypot(p.x, p.y));}
  const double turn = normAngle(b.yaw - a.yaw);
  const double travel = std::hypot(b.x - a.x, b.y - a.y) + radius * std::abs(turn);
  if (!std::isfinite(travel)) {return false;}
  const int steps = std::max(1, static_cast<int>(std::ceil(travel / (0.5 * map.getResolution()))));
  for (int i = 0; i <= steps; ++i) {
    const double f = static_cast<double>(i) / steps;
    const Pose2 pose{a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), normAngle(a.yaw + f * turn)};
    if (!footprintFree(map, footprint, pose)) {
      if (collision_pose) {*collision_pose = pose;}
      return false;
    }
  }
  return true;
}

}  // namespace nav_behaviors
