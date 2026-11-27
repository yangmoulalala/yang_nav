#include "nav_behaviors/mapping_obstacle_layer.hpp"

#include "nav_behaviors/costmap_collision.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav_behaviors
{
void MappingObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  TimedObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  for (const auto & point : world_footprint_) {
    touch(point.x, point.y, min_x, min_y, max_x, max_y);
  }
  world_footprint_.clear();
  if (!enabled_) {return;}
  robot_ = {robot_x, robot_y, robot_yaw};
  footprint_ = getFootprint();
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, footprint_, world_footprint_);
  for (const auto & point : world_footprint_) {
    touch(point.x, point.y, min_x, min_y, max_x, max_y);
  }
}

void MappingObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
  int min_col, int min_row, int max_col, int max_row)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  ObstacleLayer::updateCosts(master_grid, min_col, min_row, max_col, max_row);
  if (!enabled_ || world_footprint_.empty()) {return;}
  visitFootprintCells(master_grid, footprint_, robot_,
    [&](unsigned col, unsigned row, const auto &) {
      if (master_grid.getCost(col, row) == nav2_costmap_2d::NO_INFORMATION) {
        master_grid.setCost(col, row, nav2_costmap_2d::FREE_SPACE);
      }
      return true;
    });
}
}

PLUGINLIB_EXPORT_CLASS(nav_behaviors::MappingObstacleLayer, nav2_costmap_2d::Layer)
