#include <gtest/gtest.h>

#include "nav_behaviors/costmap_collision.hpp"
#include "nav_behaviors/mapping_obstacle_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rcl/time.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

TEST(MappingObstacleLayer, ClearsIntersectingUnknownCellsButPreservesMarksAndOutsideUnknown)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<nav2_util::LifecycleNode>("mapping_layer_test", "",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("use_sim_time", true),
      rclcpp::Parameter("live.footprint_clearing_enabled", false)}));
  auto clock = node->get_clock()->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 10000000000LL), RCL_RET_OK);
  node->declare_parameter("track_unknown_space", true);
  node->declare_parameter("transform_tolerance", 0.5);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap layered("map", true, true);
  layered.resizeMap(100, 100, 0.05, -2.5, -2.5);
  auto layer = std::make_shared<nav_behaviors::MappingObstacleLayer>();
  layered.addPlugin(layer);
  layer->initialize(&layered, "live", &tf, node,
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));

  std::vector<geometry_msgs::msg::Point> footprint;
  ASSERT_TRUE(nav2_costmap_2d::makeFootprintFromString(
    "[[0.51,0.4],[-0.51,0.4],[-0.51,-0.4],[0.51,-0.4]]", footprint));
  layered.setFootprint(footprint);
  auto & map = *layered.getCostmap();
  const auto cell_cost = [&](double world_x, double world_y) {
      unsigned col, row;
      if (!map.worldToMap(world_x, world_y, col, row)) {
        ADD_FAILURE() << "Test point is out of bounds";
        return nav2_costmap_2d::NO_INFORMATION;
      }
      return map.getCost(col, row);
    };
  const nav_behaviors::Pose2 start{0.0, 0.0, 0.0};
  EXPECT_FALSE(nav_behaviors::footprintFree(map, footprint, start));
  layered.updateMap(start.x, start.y, start.yaw);
  EXPECT_TRUE(nav_behaviors::footprintFree(map, footprint, start));
  EXPECT_EQ(cell_cost(0.525, 0.025), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cell_cost(0.575, 0.025), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(nav_behaviors::footprintMotionFree(map, footprint, start, {0.10, 0.0, 0.0}));
  layered.updateMap(1.0, 0.0, 0.0);
  EXPECT_EQ(cell_cost(0.025, 0.025), nav2_costmap_2d::NO_INFORMATION);

  nav2_costmap_2d::Observation hit;
  hit.origin_.x = 0.0;
  hit.obstacle_max_range_ = 6.0;
  hit.cloud_->header.frame_id = "map";
  hit.cloud_->header.stamp = rclcpp::Time(10, 0);
  sensor_msgs::PointCloud2Modifier points(*hit.cloud_);
  points.setPointCloud2FieldsByString(1, "xyz");
  points.resize(2);
  sensor_msgs::PointCloud2Iterator<float> point_x(*hit.cloud_, "x");
  sensor_msgs::PointCloud2Iterator<float> point_y(*hit.cloud_, "y");
  sensor_msgs::PointCloud2Iterator<float> point_z(*hit.cloud_, "z");
  for (float x : {0.525f, 0.625f}) {
    *point_x = x;
    *point_y = 0.025f;
    *point_z = 0.1f;
    ++point_x; ++point_y; ++point_z;
  }
  layer->addStaticObservation(hit, true, false);
  layered.updateMap(start.x, start.y, start.yaw);
  EXPECT_EQ(cell_cost(0.525, 0.025), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(nav_behaviors::footprintFree(map, footprint, start));
  ASSERT_EQ(rcl_set_ros_time_override(clock, 10900000000LL), RCL_RET_OK);
  layered.updateMap(0.10, 0.0, 0.0);
  EXPECT_EQ(cell_cost(0.625, 0.025), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_EQ(rcl_set_ros_time_override(clock, 11100000000LL), RCL_RET_OK);
  layered.updateMap(start.x, start.y, start.yaw);
  EXPECT_EQ(cell_cost(0.525, 0.025), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(cell_cost(0.625, 0.025), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_TRUE(nav_behaviors::footprintFree(map, footprint, start));
  layered.updateMap(start.x, start.y, start.yaw);
  EXPECT_EQ(cell_cost(0.625, 0.025), nav2_costmap_2d::NO_INFORMATION);
}
