// Keep dynamic evidence for a bounded duration, independently of the static map.
#pragma once
#include <map>
#include <utility>
#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace nav_behaviors
{
class TimedObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;
  void updateBounds(double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void reset() override;
private:
  struct Evidence {double x, y, stamp;};
  std::map<std::pair<int64_t, int64_t>, Evidence> evidence_;
  double retention_{1.0}, resolution_{0.0}, anchor_x_{0.0}, anchor_y_{0.0}, last_time_{-1.0};
};
}  // namespace nav_behaviors
