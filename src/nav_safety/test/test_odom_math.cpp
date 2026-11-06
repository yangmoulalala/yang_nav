#include <gtest/gtest.h>

#include <cmath>

#include "nav_safety/odom_math.hpp"

using namespace nav_safety;
using Eigen::Vector3d;

TEST(OdomMath, PureTranslationThroughRolledImu)
{
  // imu rolled 105 deg about base x, at the base origin: a base-forward motion appears
  // along imu x unchanged, base-left motion appears rotated in the imu y/z plane.
  const Eigen::Matrix3d R_bi = Eigen::AngleAxisd(105.0 * M_PI / 180.0, Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d R_ib = R_bi.transpose();
  const Vector3d v_b(0.5, 0.3, 0.0);
  const auto t = imu_to_base_twist(R_ib * v_b, Vector3d::Zero(), R_ib, Vector3d::Zero());
  EXPECT_TRUE(t.v.isApprox(v_b, 1e-12));
  EXPECT_TRUE(t.w.isZero(1e-12));
}

TEST(OdomMath, RotationAddsLeverArm)
{
  // frames aligned, base origin 0.2 m behind the imu; spinning at 1 rad/s about z with the
  // imu momentarily still means the base origin moves at -0.2 m/s along y.
  const auto t = imu_to_base_twist(
    Vector3d::Zero(), Vector3d(0, 0, 1.0), Eigen::Matrix3d::Identity(), Vector3d(-0.2, 0, 0));
  EXPECT_NEAR(t.v.x(), 0.0, 1e-12);
  EXPECT_NEAR(t.v.y(), -0.2, 1e-12);
  EXPECT_NEAR(t.w.z(), 1.0, 1e-12);
}

TEST(OdomMath, PoseComposition)
{
  Eigen::Isometry3d T_oi = Eigen::Isometry3d::Identity();
  T_oi.linear() = Eigen::AngleAxisd(M_PI / 2, Vector3d::UnitZ()).toRotationMatrix();
  T_oi.translation() = Vector3d(1, 2, 0);
  const auto T_ob = imu_to_base_pose(T_oi, Eigen::Matrix3d::Identity(), Vector3d(-0.2, 0, -0.1));
  EXPECT_TRUE(T_ob.translation().isApprox(Vector3d(1, 1.8, -0.1), 1e-12));
}
