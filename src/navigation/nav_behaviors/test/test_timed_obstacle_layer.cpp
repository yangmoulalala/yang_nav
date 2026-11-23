#include <gtest/gtest.h>
#include "nav_behaviors/timed_obstacle_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rcl/time.h"

class TimedLayerTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<nav2_util::LifecycleNode>("timed_layer_test", "", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true), rclcpp::Parameter("dynamic.min_obstacle_height", -10.0)}));
    auto clock = node->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    at(10.0);
    map = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    map->resizeMap(100, 100, 0.1, -5.0, -5.0);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    layer = std::make_shared<nav_behaviors::TimedObstacleLayer>();
    map->addPlugin(layer);
    layer->initialize(map.get(), "dynamic", tf.get(), node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  }
  void at(double seconds) {
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
      static_cast<int64_t>(seconds * 1e9)), RCL_RET_OK);
  }
  void observe(double stamp, float x=1.0, float z=-0.2) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id="map"; cloud.header.stamp=rclcpp::Time(static_cast<int64_t>(stamp*1e9));
    sensor_msgs::PointCloud2Modifier mod(cloud); mod.setPointCloud2FieldsByString(1,"xyz"); mod.resize(1);
    sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z");
    *px=x; *py=1.0; *pz=z;
    nav2_costmap_2d::Observation obs;
    *obs.cloud_=cloud; obs.obstacle_max_range_=10; obs.obstacle_min_range_=0;
    layer->clearStaticObservations(true, true); layer->addStaticObservation(obs,true,false);
  }
  void update() {map->updateMap(0,0,0);}
  unsigned char cost(float x=1.0) {unsigned mx,my; layer->worldToMap(x,1.0,mx,my);return layer->getCost(mx,my);}
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> map;
  std::shared_ptr<nav_behaviors::TimedObstacleLayer> layer;
};

TEST_F(TimedLayerTest, OldCloudCannotRefreshItsOwnEvidence) {
  observe(10); update(); EXPECT_EQ(cost(),254);
  at(10.9); update(); EXPECT_EQ(cost(),254);
  at(11.1); update(); EXPECT_EQ(cost(),0);
  at(11.2); update(); EXPECT_EQ(cost(),0);
}
TEST_F(TimedLayerTest, NewAcquisitionRefreshesAndStationaryObstacleStays) {
  observe(10); update(); at(10.8); observe(10.8); update();
  at(11.2); update(); EXPECT_EQ(cost(),254);
  at(11.9); update(); EXPECT_EQ(cost(),0);
}
TEST_F(TimedLayerTest, ExpiryDoesNotClearAnotherLayer) {
  observe(10); update(); at(11.1); update();
  nav2_costmap_2d::Costmap2D master(100,100,.1,-5,-5,0);
  unsigned mx,my;master.worldToMap(1,1,mx,my);master.setCost(mx,my,254);
  layer->updateCosts(master,0,0,100,100);
  EXPECT_EQ(master.getCost(mx,my),254);
}
TEST_F(TimedLayerTest, EvidenceExpiresAfterObservationDisappears) {
  observe(10); update(); layer->clearStaticObservations(true,true);
  at(11.1); update(); EXPECT_EQ(cost(),0);
}
TEST_F(TimedLayerTest, ResetAndClockRewindDoNotLeaveImmortalMarks) {
  observe(10);update();at(5);layer->clearStaticObservations(true,true);update();
  EXPECT_EQ(cost(),254);at(6.1);update();EXPECT_EQ(cost(),0);
  layer->reset();at(7);observe(7);update();EXPECT_EQ(cost(),254);
  layer->clearStaticObservations(true,true);at(8.1);update();EXPECT_EQ(cost(),0);
}