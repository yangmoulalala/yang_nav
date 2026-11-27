#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include "nav_behaviors/costmap_collision.hpp"
#include "nav_behaviors/translation_reference.hpp"

namespace nav_behaviors
{
class ClearMapConfirmation
{
public:
  void reset() {first_.reset(); last_.reset();}

  static bool fresh(int64_t now, int64_t map_stamp, int64_t observation_stamp)
  {
    return map_stamp > 0 && observation_stamp > 0 && now >= map_stamp &&
           now >= observation_stamp && now - map_stamp <= max_age &&
           now - observation_stamp <= max_age;
  }

  bool update(bool clear, int64_t now, int64_t map_stamp, int64_t observation_stamp)
  {
    constexpr int64_t clear_span = 1000000000;
    if (!clear || !fresh(now, map_stamp, observation_stamp)) {
      reset(); return false;
    }
    const Sample sample{now, map_stamp, observation_stamp};
    if (!last_ || now < last_->now || now - last_->now > max_age ||
      map_stamp < last_->map || observation_stamp < last_->observation ||
      map_stamp - last_->map > max_age || observation_stamp - last_->observation > max_age)
    {
      first_ = sample;
    }
    last_ = sample;
    return now - first_->now >= clear_span && map_stamp - first_->map >= clear_span &&
           observation_stamp - first_->observation >= clear_span;
  }

private:
  static constexpr int64_t max_age = 500000000;
  struct Sample {int64_t now, map, observation;};
  std::optional<Sample> first_, last_;
};

class OrdinaryEscapeHold
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Decision {Wait, Rejoin, Escape, Fail};

  bool active() const {return escape_deadline_.has_value();}

  void start(Clock::time_point now)
  {
    escape_deadline_ = now + std::chrono::seconds(1);
    rejoin_deadline_ = *escape_deadline_ + rejoin_window;
  }

  void reset()
  {
    escape_deadline_.reset();
    confirmation_.reset();
  }

  Decision update(
    Clock::time_point now, bool usable, bool clear, int64_t stamp, int64_t map_stamp,
    int64_t observation_stamp)
  {
    if (confirmation_.update(usable && clear, stamp, map_stamp, observation_stamp)) {
      return Decision::Rejoin;
    }
    if (now < *escape_deadline_) {return Decision::Wait;}
    if (!usable || !ClearMapConfirmation::fresh(stamp, map_stamp, observation_stamp)) {
      return Decision::Fail;
    }
    if (!clear) {return Decision::Escape;}
    return now < rejoin_deadline_ ? Decision::Wait : Decision::Fail;
  }

  void commandedEscape(Clock::time_point now) {rejoin_deadline_ = now + rejoin_window;}

private:
  static constexpr auto rejoin_window = std::chrono::milliseconds(1250);
  std::optional<Clock::time_point> escape_deadline_;
  Clock::time_point rejoin_deadline_{};
  ClearMapConfirmation confirmation_;
};

inline bool escapeRejoinPathFree(const nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint, const std::vector<XY> & path,
  const Pose2 & robot, double preview)
{
  if (path.empty() || !std::isfinite(preview) || preview <= 0.0) {return false;}
  const auto projection = projectOntoPath(path, {robot.x, robot.y});
  if (!std::isfinite(projection.distance_squared)) {return false;}
  const auto join = pathLookahead(path, projection, 0.0);
  Pose2 previous = robot;
  auto check_to = [&](XY target) {
      if (!std::isfinite(target.x) || !std::isfinite(target.y)) {return false;}
      const double dx = target.x - previous.x, dy = target.y - previous.y;
      const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / (0.5 * map.getResolution()))));
      for (int step = 0; step <= steps; ++step) {
        const double fraction = static_cast<double>(step) / steps;
        const Pose2 pose{previous.x + fraction * dx, previous.y + fraction * dy, robot.yaw};
        unsigned col, row;
        if (!map.worldToMap(pose.x, pose.y, col, row) ||
          map.getCost(col, row) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
          !footprintFree(map, footprint, pose)) {return false;}
      }
      previous = {target.x, target.y, robot.yaw};
      return true;
    };
  if (!check_to(join)) {return false;}
  for (size_t index = projection.segment + 1; index < path.size() && preview > 0.0; ++index) {
    const double dx = path[index].x - previous.x, dy = path[index].y - previous.y;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length)) {return false;}
    if (length == 0.0) {continue;}
    const double fraction = std::min(1.0, preview / length);
    if (!check_to({previous.x + fraction * dx, previous.y + fraction * dy})) {return false;}
    preview -= length;
  }
  return true;
}
}  // namespace nav_behaviors
