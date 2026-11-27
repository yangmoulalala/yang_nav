#pragma once

#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/timed_obstacle_layer.hpp"

namespace nav_behaviors
{
class MappingObstacleLayer : public TimedObstacleLayer
{
public:
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
    int min_col, int min_row, int max_col, int max_row) override;

protected:
  unsigned char expiredObstacleCost() const override {return nav2_costmap_2d::NO_INFORMATION;}

private:
  Pose2 robot_{};
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::vector<geometry_msgs::msg::Point> world_footprint_;
};
}
