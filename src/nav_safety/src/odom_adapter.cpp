// ZLIO odometry (imu frame) -> body odometry for Nav2.
//
//   input  (remap)  nav_msgs/Odometry  pose lio_odom <- imu, twist in the imu frame
//   output (remap)  nav_msgs/Odometry  pose lio_odom <- base_link, twist in base_link
//
// The MID-360 frame is rolled ~105 deg against the chassis, so its twist cannot feed the
// controller or velocity smoother directly.  imu <- base comes from TF and is refreshed
// periodically (the turret yaw joint moves it on the real robot).
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav_safety/odom_math.hpp"

namespace nav_safety
{

class OdomAdapter : public rclcpp::Node
{
public:
  OdomAdapter()
  : Node("odom_adapter")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    refresh_ = declare_parameter<double>("extrinsic_refresh", 0.5);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<nav_msgs::msg::Odometry>("output", 20);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "input", 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {on_odom(*m);});
  }

private:
  bool update_extrinsic(const std::string & imu_frame)
  {
    const rclcpp::Time now = get_clock()->now();
    if (have_ext_ && (now - ext_time_).seconds() < refresh_) {
      return true;
    }
    try {
      const auto t = tf_buffer_->lookupTransform(imu_frame, base_frame_, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      const auto & p = t.transform.translation;
      R_ib_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      r_ib_ = Eigen::Vector3d(p.x, p.y, p.z);
      have_ext_ = true;
      ext_time_ = now;
    } catch (const tf2::TransformException & e) {
      if (!have_ext_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "waiting for %s <- %s: %s",
          imu_frame.c_str(), base_frame_.c_str(), e.what());
        return false;
      }
    }
    return true;
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    if (!update_extrinsic(msg.child_frame_id)) {
      return;
    }
    const auto & pp = msg.pose.pose.position;
    const auto & pq = msg.pose.pose.orientation;
    Eigen::Isometry3d T_oi = Eigen::Isometry3d::Identity();
    T_oi.linear() = Eigen::Quaterniond(pq.w, pq.x, pq.y, pq.z).normalized().toRotationMatrix();
    T_oi.translation() = Eigen::Vector3d(pp.x, pp.y, pp.z);
    const Eigen::Isometry3d T_ob = imu_to_base_pose(T_oi, R_ib_, r_ib_);

    const auto & tw = msg.twist.twist;
    const BodyTwist bt = imu_to_base_twist(
      Eigen::Vector3d(tw.linear.x, tw.linear.y, tw.linear.z),
      Eigen::Vector3d(tw.angular.x, tw.angular.y, tw.angular.z), R_ib_, r_ib_);

    nav_msgs::msg::Odometry out;
    out.header = msg.header;
    out.child_frame_id = base_frame_;
    const Eigen::Quaterniond q(T_ob.linear());
    out.pose.pose.position.x = T_ob.translation().x();
    out.pose.pose.position.y = T_ob.translation().y();
    out.pose.pose.position.z = T_ob.translation().z();
    out.pose.pose.orientation.w = q.w();
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.covariance = msg.pose.covariance;
    out.twist.twist.linear.x = bt.v.x();
    out.twist.twist.linear.y = bt.v.y();
    out.twist.twist.linear.z = bt.v.z();
    out.twist.twist.angular.x = bt.w.x();
    out.twist.twist.angular.y = bt.w.y();
    out.twist.twist.angular.z = bt.w.z();
    out.twist.covariance = msg.twist.covariance;
    pub_->publish(out);
  }

  std::string base_frame_;
  double refresh_{0.5};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  bool have_ext_{false};
  rclcpp::Time ext_time_;
  Eigen::Matrix3d R_ib_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d r_ib_{Eigen::Vector3d::Zero()};
};

}  // namespace nav_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_safety::OdomAdapter>());
  rclcpp::shutdown();
  return 0;
}
