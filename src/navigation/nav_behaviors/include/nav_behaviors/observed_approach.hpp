#pragma once

#include <array>
#include <functional>
#include <map>
#include <queue>
#include "nav_msgs/msg/path.hpp"
#include "nav_behaviors/costmap_collision.hpp"

namespace nav_behaviors
{

inline bool observedTranslationFree(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2 & start, const Pose2 & goal)
{
  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  if (!std::isfinite(distance)) {return false;}
  const auto steps = std::max(1, static_cast<int>(std::ceil(distance / (0.5 * map.getResolution()))));
  for (int step = 0; step <= steps; ++step) {
    const double fraction = static_cast<double>(step) / steps;
    const Pose2 pose{start.x + fraction * (goal.x - start.x),
      start.y + fraction * (goal.y - start.y), start.yaw};
    unsigned col, row;
    if (!map.worldToMap(pose.x, pose.y, col, row) ||
      map.getCost(col, row) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      !footprintFree(map, footprint, pose)) {return false;}
  }
  return true;
}

inline bool observedPathFree(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const nav_msgs::msg::Path & path,
  const Pose2 & start, const Pose2 & goal)
{
  if (path.poses.empty()) {return false;}
  auto previous = start;
  for (const auto & point : path.poses) {
    const Pose2 pose{point.pose.position.x, point.pose.position.y, start.yaw};
    if (!footprintMotionFree(map, footprint, previous, pose)) {return false;}
    previous = pose;
  }
  return std::hypot(previous.x - goal.x, previous.y - goal.y) <=
         std::sqrt(2.0) * map.getResolution() &&
         footprintMotionFree(map, footprint, previous, {goal.x, goal.y, start.yaw});
}

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
    if (!map.worldToMap(pose.x, pose.y, mx, my)) {
      blocked_by_unknown = true;
      break;
    }
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

inline std::vector<Pose2> observedDetour(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const Pose2 & start, const Pose2 & goal,
  double clearance, const std::function<bool()> & stop)
{
  if (!observedTranslationFree(map, footprint, start, start)) {return {};}
  const double stride = std::max(0.10, 2.0 * map.getResolution());
  auto distance = [&](const Pose2 & pose) {return std::hypot(pose.x - goal.x, pose.y - goal.y);};
  const double initial = distance(start);
  if (!std::isfinite(initial) || initial < stride) {return {};}
  const bool can_finish = observedTranslationFree(map, footprint, goal, goal);
  const bool partial_allowed = needsObservedApproach(map, footprint, goal);
  if (!can_finish && !partial_allowed) {return {};}
  auto stopping_footprint = footprint;
  nav2_costmap_2d::padFootprint(stopping_footprint, clearance);
  using Key = std::pair<int, int>;
  using Entry = std::pair<double, Key>;
  struct Record {Key parent; double travel; bool closed;};
  const Key origin{0, 0};
  std::map<Key, Record> records{{origin, {origin, 0.0, false}}};
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pending;
  pending.emplace(initial, origin);
  const std::array<Key, 8> directions{{{1,0},{0,1},{-1,0},{0,-1},{1,1},{-1,1},{-1,-1},{1,-1}}};
  auto pose_at = [&](Key key) {return Pose2{start.x + stride * key.first,
      start.y + stride * key.second, start.yaw};};
  auto path_to = [&](Key key) {
      std::vector<Pose2> path;
      while (key != origin) {path.push_back(pose_at(key)); key = records.at(key).parent;}
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    };
  Key best = origin;
  double best_remaining = initial - 0.15;
  for (size_t expanded = 0; !pending.empty() && expanded < 800;) {
    if (stop && stop()) {break;}
    const auto key = pending.top().second;
    pending.pop();
    auto & record = records.at(key);
    if (record.closed) {continue;}
    record.closed = true;
    ++expanded;
    const auto current = pose_at(key);
    const double remaining = distance(current);
    if (can_finish && remaining <= 3.0 * stride && observedTranslationFree(map, footprint, current, goal)) {
      auto path = path_to(key);
      if (remaining > 1e-8) {path.push_back(goal);} else {path.back() = goal;}
      return path;
    }
    if (partial_allowed && remaining > 0.20 && remaining < best_remaining &&
      footprintFree(map, stopping_footprint, current)) {best = key; best_remaining = remaining;}
    for (const auto & direction : directions) {
      const Key next{key.first + direction.first, key.second + direction.second};
      const double travel = record.travel + stride * std::hypot(direction.first, direction.second);
      const auto found = records.find(next);
      if (found != records.end() && (found->second.closed || found->second.travel <= travel)) {continue;}
      const auto candidate = pose_at(next);
      if (!observedTranslationFree(map, footprint, current, candidate)) {continue;}
      records.insert_or_assign(next, Record{key, travel, false});
      pending.emplace(travel + distance(candidate), next);
    }
  }
  return best == origin ? std::vector<Pose2>{} : path_to(best);
}

}  // namespace nav_behaviors
