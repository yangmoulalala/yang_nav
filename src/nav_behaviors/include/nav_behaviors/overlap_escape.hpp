// Escape an already overlapping map footprint without clearing the map or increasing overlap.
// Callers own the costmap lock. Unknown cells and map bounds never receive an exemption.
#pragma once
#include <cstdint>
#include <utility>
#include "nav_behaviors/costmap_collision.hpp"

namespace nav_behaviors
{
struct ContactState
{
  bool valid{false};
  std::vector<std::pair<int64_t, int64_t>> cells;  // world micrometres: stable across rolling origins
  double area{0.0};
};

inline double cellOverlapArea(std::vector<geometry_msgs::msg::Point> poly,
  double cx, double cy, double half)
{
  for (int axis = 0; axis < 2; ++axis) {
    for (int side : {-1, 1}) {
      const double edge = (axis == 0 ? cx : cy) + side * half;
      const auto input = poly;
      poly.clear();
      if (input.empty()) {return 0.0;}
      auto value = [&](const auto & p) {return axis == 0 ? p.x : p.y;};
      for (size_t i = 0; i < input.size(); ++i) {
        const auto & a = input[i];
        const auto & b = input[(i + 1) % input.size()];
        const bool ai = side * (value(a) - edge) <= 0.0;
        const bool bi = side * (value(b) - edge) <= 0.0;
        if (ai) {poly.push_back(a);}
        if (ai != bi) {
          const double f = (edge - value(a)) / (value(b) - value(a));
          geometry_msgs::msg::Point q;
          q.x = a.x + f * (b.x - a.x); q.y = a.y + f * (b.y - a.y);
          poly.push_back(q);
        }
      }
    }
  }
  double area = 0.0;
  for (size_t i = 0; i < poly.size(); ++i) {
    const auto & a = poly[i]; const auto & b = poly[(i + 1) % poly.size()];
    area += (a.x - cx) * (b.y - cy) - (b.x - cx) * (a.y - cy);
  }
  return 0.5 * std::abs(area);
}

inline ContactState footprintContacts(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & fp, const Pose2 & pose)
{
  ContactState out;
  out.valid = visitFootprintCells(map, fp, pose, [&](unsigned x, unsigned y, const auto & poly) {
      const auto cost = map.getCost(x, y);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {return false;}
      if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {return true;}
      double cx, cy; map.mapToWorld(x, y, cx, cy);
      out.cells.emplace_back(std::llround(cx * 1e6), std::llround(cy * 1e6));
      out.area += cellOverlapArea(poly, cx, cy, map.getResolution() * 0.5);
      return true;
    });
  std::sort(out.cells.begin(), out.cells.end());
  return out;
}

inline bool contactsDecrease(const ContactState & before, const ContactState & after)
{
  return before.valid && after.valid && after.area <= before.area + 1e-8 &&
    std::all_of(after.cells.begin(), after.cells.end(), [&](const auto & cell) {
      return std::binary_search(before.cells.begin(), before.cells.end(), cell);
    });
}

// A start embedded in a thick map obstacle must be able to leave its connected
// component, rather than being trapped at the first newly touched cell of the same
// obstacle. Freeze this bounded set for the episode; unrelated/new obstacles stay lethal.
inline ContactState escapeAllowance(const nav2_costmap_2d::Costmap2D & map,
  const ContactState & initial, const Pose2 & start, double max_distance = 1.5)
{
  auto allowed = initial;
  if (!initial.valid) {return allowed;}
  std::vector<unsigned> pending;
  std::vector<bool> seen(map.getSizeInCellsX() * map.getSizeInCellsY(), false);
  for (const auto & key : initial.cells) {
    unsigned x, y;
    if (map.worldToMap(key.first * 1e-6, key.second * 1e-6, x, y)) {pending.push_back(map.getIndex(x, y));}
  }
  double reach = map.getResolution();
  for (const auto & key : initial.cells) {
    reach = std::max(reach, std::hypot(key.first * 1e-6 - start.x, key.second * 1e-6 - start.y));
  }
  allowed.cells.clear();
  for (size_t i = 0; i < pending.size(); ++i) {
    const unsigned index = pending[i];
    if (seen[index]) {continue;}
    seen[index] = true;
    unsigned x, y; map.indexToCells(index, x, y);
    if (map.getCost(x, y) != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
    double cx, cy; map.mapToWorld(x, y, cx, cy);
    if (std::hypot(cx - start.x, cy - start.y) > max_distance + reach + map.getResolution()) {continue;}
    allowed.cells.emplace_back(std::llround(cx * 1e6), std::llround(cy * 1e6));
    for (int dx : {-1, 0, 1}) {
      for (int dy : {-1, 0, 1}) {
        const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
        if (nx >= 0 && ny >= 0 && nx < static_cast<int>(map.getSizeInCellsX()) &&
          ny < static_cast<int>(map.getSizeInCellsY())) {pending.push_back(map.getIndex(nx, ny));}
      }
    }
  }
  std::sort(allowed.cells.begin(), allowed.cells.end());
  return allowed;
}

// Translation only. Overlap area may not grow, including within the original component.
inline bool escapeMotionFree(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & fp, const Pose2 & a, const Pose2 & b,
  const ContactState * episode = nullptr)
{
  if (!std::isfinite(b.yaw) || std::abs(normAngle(b.yaw - a.yaw)) > 1e-3) {return false;}
  const auto current = footprintContacts(map, fp, a);
  auto previous = episode ? *episode : escapeAllowance(map, current, a);
  if (!contactsDecrease(previous, current)) {return false;}
  previous.area = current.area;
  const double distance = std::hypot(b.x - a.x, b.y - a.y);
  if (!std::isfinite(distance)) {return false;}
  const int count = std::max(1, static_cast<int>(std::ceil(distance / (map.getResolution() * 0.5))));
  for (int i = 1; i <= count; ++i) {
    const double f = static_cast<double>(i) / count;
    auto next = footprintContacts(map, fp, {a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), a.yaw});
    if (!contactsDecrease(previous, next)) {return false;}
    previous.area = next.area;
  }
  return true;
}

struct EscapePath {std::vector<Pose2> poses; double score{0.0};};
inline std::vector<EscapePath> escapePaths(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & fp, const Pose2 & start, const Pose2 & goal,
  double max_distance = 1.5)
{
  std::vector<EscapePath> out;
  const auto initial = footprintContacts(map, fp, start);
  if (!initial.valid) {return out;}
  const auto allowed = escapeAllowance(map, initial, start, max_distance);
  const double step = map.getResolution() * 0.5;
  const double preferred = std::atan2(goal.y - start.y, goal.x - start.x);
  for (int ray = 0; ray < 64; ++ray) {
    const double yaw = preferred + ray * (2.0 * kPi / 64.0);
    EscapePath candidate; candidate.poses.push_back(start);
    auto previous = allowed;
    double first_free = -1.0;
    for (double d = step; d <= max_distance + 1e-9; d += step) {
      const Pose2 q{start.x + d * std::cos(yaw), start.y + d * std::sin(yaw), start.yaw};
      auto next = footprintContacts(map, fp, q);
      if (!contactsDecrease(previous, next)) {break;}
      previous.area = next.area;
      candidate.poses.push_back(q);
      unsigned x, y;
      const bool free = next.cells.empty() && map.worldToMap(q.x, q.y, x, y) &&
        map.getCost(x, y) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      if (free && first_free < 0.0) {first_free = d;}
      if (free && d >= first_free + 0.08) {
        candidate.score = d + 0.25 * (std::hypot(goal.x - q.x, goal.y - q.y) -
          std::hypot(goal.x - start.x, goal.y - start.y));
        out.push_back(std::move(candidate));
        break;
      }
    }
  }
  std::sort(out.begin(), out.end(), [](const auto & a, const auto & b) {return a.score < b.score;});
  return out;
}
}  // namespace nav_behaviors
