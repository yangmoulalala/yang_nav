#pragma once

#include "nav_behaviors/costmap_collision.hpp"

namespace nav_behaviors
{

inline bool needsObservedApproach(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2 & goal)
{
  unsigned mx, my;
  if (!map.worldToMap(goal.x, goal.y, mx, my)) {return true;}
  const auto cost = map.getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {return true;}
  if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {return false;}
  return !visitFootprintCells(map, footprint, goal, [&](unsigned x, unsigned y, const auto &) {
      return map.getCost(x, y) != nav2_costmap_2d::NO_INFORMATION;
    });
}

inline std::vector<Pose2> observedApproach(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2 & start, const Pose2 & goal,
  double clearance, bool require_unknown = false)
{
  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  constexpr double goal_separation = 0.20;
  if (!std::isfinite(distance) || distance <= goal_separation ||
    !footprintFree(map, footprint, start)) {return {};}

  auto stop_footprint = footprint;
  nav2_costmap_2d::padFootprint(stop_footprint, clearance);
  const double step = 0.5 * map.getResolution();
  std::vector<Pose2> path{start};
  size_t safe_end = 0;
  bool blocked_by_unknown = false;
  for (double travel = step; travel < distance; travel += step) {
    const double fraction = travel / distance;
    const Pose2 pose{start.x + fraction * (goal.x - start.x),
      start.y + fraction * (goal.y - start.y), start.yaw};
    unsigned mx, my;
    if (!map.worldToMap(pose.x, pose.y, mx, my)) {break;}
    const auto cost = map.getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE || !footprintFree(map, footprint, pose)) {
      blocked_by_unknown = needsObservedApproach(map, footprint, pose);
      break;
    }
    if (travel < distance - goal_separation) {
      path.push_back(pose);
      if (footprintFree(map, stop_footprint, pose)) {safe_end = path.size();}
    }
  }
  if (require_unknown && !blocked_by_unknown) {return {};}
  path.resize(safe_end);
  return path;
}

}  // namespace nav_behaviors
