// Rigid-body helpers for moving ZLIO odometry (imu frame) onto base_link.
#pragma once

#include <Eigen/Geometry>

namespace nav_safety
{

struct BodyTwist
{
  Eigen::Vector3d v;  // linear, m/s
  Eigen::Vector3d w;  // angular, rad/s
};

// Twist of the base origin expressed in the base frame.
//   v_i, w_i : imu twist expressed in the imu frame
//   R_ib     : rotation imu <- base
//   r_ib     : base origin expressed in the imu frame
// v_b = R_bi (v_i + w_i x r_ib),  w_b = R_bi w_i
inline BodyTwist imu_to_base_twist(
  const Eigen::Vector3d & v_i, const Eigen::Vector3d & w_i,
  const Eigen::Matrix3d & R_ib, const Eigen::Vector3d & r_ib)
{
  const Eigen::Matrix3d R_bi = R_ib.transpose();
  return {R_bi * (v_i + w_i.cross(r_ib)), R_bi * w_i};
}

// Pose odom <- base from pose odom <- imu and the extrinsic imu <- base.
inline Eigen::Isometry3d imu_to_base_pose(
  const Eigen::Isometry3d & T_oi, const Eigen::Matrix3d & R_ib, const Eigen::Vector3d & r_ib)
{
  Eigen::Isometry3d T_ib = Eigen::Isometry3d::Identity();
  T_ib.linear() = R_ib;
  T_ib.translation() = r_ib;
  return T_oi * T_ib;
}

}  // namespace nav_safety
