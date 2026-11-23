#include <cmath>
#include <limits>
#include "nav_behaviors/timed_obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav_behaviors
{
void TimedObstacleLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  declareParameter("retention_seconds", rclcpp::ParameterValue(1.0));
  auto node = node_.lock();
  node->get_parameter(name_ + ".retention_seconds", retention_);
  if (!std::isfinite(retention_) || retention_ < 0.5 || retention_ > 10.0) {
    throw std::runtime_error("TimedObstacleLayer: retention_seconds must be in [0.5,10]");
  }
}

void TimedObstacleLayer::reset()
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  ObstacleLayer::reset();
  evidence_.clear();
  last_time_ = -1.0;
}

void TimedObstacleLayer::updateBounds(double x, double y, double yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  ObstacleLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);
  if (!enabled_) {return;}
  const double now = clock_->now().seconds();
  if (resolution_ != getResolution()) {
    evidence_.clear();
    resolution_ = getResolution();
    anchor_x_ = getOriginX(); anchor_y_ = getOriginY();
  }
  if (last_time_ >= 0.0 && now < last_time_) {
    // A clock rewind must not leave evidence immortal or expire it prematurely.
    for (auto & item : evidence_) {item.second.stamp = now;}
  }
  last_time_ = now;
  std::vector<nav2_costmap_2d::Observation> observations;
  getMarkingObservations(observations);
  for (const auto & obs : observations) {
    const auto & cloud = *obs.cloud_;
    const double stamp = rclcpp::Time(cloud.header.stamp).seconds();
    if (!std::isfinite(stamp) || stamp > now + 0.05) {continue;}
    sensor_msgs::PointCloud2ConstIterator<float> px(cloud, "x"), py(cloud, "y"), pz(cloud, "z");
    for (; px != px.end(); ++px, ++py, ++pz) {
      unsigned mx, my;
      if (!std::isfinite(*px) || !std::isfinite(*py) || !std::isfinite(*pz) ||
        *pz > max_obstacle_height_ || !worldToMap(*px, *py, mx, my)) {continue;}
      const double distance = std::hypot(std::hypot(*px - obs.origin_.x, *py - obs.origin_.y), *pz - obs.origin_.z);
      if (distance >= obs.obstacle_max_range_ || distance < obs.obstacle_min_range_ ||
        getCost(mx, my) != nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
      double wx, wy; mapToWorld(mx, my, wx, wy);
      const auto key = std::make_pair(std::llround((wx - anchor_x_) / resolution_),
        std::llround((wy - anchor_y_) / resolution_));
      auto found = evidence_.find(key);
      if (found == evidence_.end() || stamp > found->second.stamp) {
        evidence_[key] = {wx, wy, stamp};
      }
    }
  }
  for (auto it = evidence_.begin(); it != evidence_.end();) {
    unsigned mx, my;
    const auto & point = it->second;
    if (!worldToMap(point.x, point.y, mx, my)) {it = evidence_.erase(it); continue;}
    if (now - point.stamp > retention_) {
      // Only this layer is changed. Static walls and other layers remain intact.
      setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
      touch(point.x, point.y, min_x, min_y, max_x, max_y);
      it = evidence_.erase(it);
    } else {++it;}
  }
}
}  // namespace nav_behaviors
PLUGINLIB_EXPORT_CLASS(nav_behaviors::TimedObstacleLayer, nav2_costmap_2d::Layer)
