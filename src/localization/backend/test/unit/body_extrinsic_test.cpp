// 时间索引外参：精确时刻、容差内的最新变换与过期回退。
#include <gtest/gtest.h>

#include <tf2/buffer_core.h>

#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include "apps/body_extrinsic.hpp"

namespace backend::ros {
namespace {

/// 云台绕 z 转动 yaw 时的 T_sensor_body：传感器在 body 前方 0.2 m、上方 0.4 m。
Eigen::Matrix4d sensorBody(const double yaw) {
    Eigen::Isometry3d T_body_sensor = Eigen::Isometry3d::Identity();
    T_body_sensor.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_body_sensor.translation() = Eigen::Vector3d(0.2, 0.0, 0.4);
    return T_body_sensor.inverse().matrix();
}

void setExtrinsic(tf2::BufferCore* buffer, const double stamp, const double yaw) {
    geometry_msgs::msg::TransformStamped transform =
        tf2::eigenToTransform(Eigen::Isometry3d(sensorBody(yaw)));
    transform.header.frame_id = "sensor";
    transform.child_frame_id = "body";
    transform.header.stamp.sec = static_cast<int32_t>(std::floor(stamp));
    transform.header.stamp.nanosec =
        static_cast<uint32_t>(std::lround((stamp - std::floor(stamp)) * 1e9));
    ASSERT_TRUE(buffer->setTransform(transform, "test", false));
}

struct Fixture {
    tf2::BufferCore buffer;
    std::vector<std::string> warnings;
    BodyExtrinsicResolver resolver{buffer, "sensor", "body", 0.2,
                                   [this](const std::string& m) { warnings.push_back(m); },
                                   nullptr};
};

/// 启动前 TF 不可用时初始化失败；可用后返回最新变换。
TEST(BodyExtrinsicResolver, InitializesFromLatestTransform) {
    Fixture f;
    std::string error;
    EXPECT_FALSE(f.resolver.tryInitialize(&error).has_value());
    EXPECT_THROW(f.resolver.resolve(1.0), std::logic_error);
    setExtrinsic(&f.buffer, 10.0, 0.0);
    setExtrinsic(&f.buffer, 11.0, 0.5);
    const auto initial = f.resolver.tryInitialize(&error);
    ASSERT_TRUE(initial.has_value());
    EXPECT_TRUE(initial->isApprox(sensorBody(0.5), 1e-9));
}

/// 请求时刻在 TF 缓存范围内时返回该时刻的（插值）外参。
TEST(BodyExtrinsicResolver, ExactTimeInterpolatesTurretYaw) {
    Fixture f;
    setExtrinsic(&f.buffer, 10.0, 0.0);
    setExtrinsic(&f.buffer, 11.0, 0.5);
    ASSERT_TRUE(f.resolver.tryInitialize(nullptr).has_value());
    const auto result = f.resolver.resolve(10.5);
    EXPECT_EQ(result.source, BodyExtrinsicResolver::Source::Exact);
    // TF 对旋转做球面插值、对平移做线性插值。
    const Eigen::Matrix3d R_mid = sensorBody(0.25).topLeftCorner(3, 3);
    const Eigen::Matrix3d R_result = result.T_sensor_body.topLeftCorner(3, 3);
    EXPECT_TRUE(R_result.isApprox(R_mid, 1e-6));
    const Eigen::Vector3d t_mid = 0.5 * (sensorBody(0.0).block<3, 1>(0, 3) +
                                         sensorBody(0.5).block<3, 1>(0, 3));
    EXPECT_LT((result.T_sensor_body.block<3, 1>(0, 3) - t_mid).norm(), 1e-6);
    EXPECT_TRUE(f.resolver.resolve(10.0).T_sensor_body.isApprox(sensorBody(0.0), 1e-9));
    EXPECT_TRUE(f.warnings.empty());
}

/// 请求时刻超出缓存但在容差内时使用最新变换；超出容差时告警并沿用上次有效值。
TEST(BodyExtrinsicResolver, LatestWithinToleranceElseKeepsLastGood) {
    Fixture f;
    setExtrinsic(&f.buffer, 10.0, 0.0);
    setExtrinsic(&f.buffer, 11.0, 0.5);
    ASSERT_TRUE(f.resolver.tryInitialize(nullptr).has_value());

    const auto latest = f.resolver.resolve(11.15);
    EXPECT_EQ(latest.source, BodyExtrinsicResolver::Source::Latest);
    EXPECT_TRUE(latest.T_sensor_body.isApprox(sensorBody(0.5), 1e-9));

    // 上次有效值更新为 yaw=0，随后的过期查询沿用它，而不是最新的 yaw=0.5。
    ASSERT_EQ(f.resolver.resolve(10.0).source, BodyExtrinsicResolver::Source::Exact);
    const auto stale = f.resolver.resolve(12.0);
    EXPECT_EQ(stale.source, BodyExtrinsicResolver::Source::Stale);
    EXPECT_TRUE(stale.T_sensor_body.isApprox(sensorBody(0.0), 1e-9));
    ASSERT_EQ(f.warnings.size(), 1U);
    EXPECT_NE(f.warnings.front().find("common.body_extrinsic_tolerance"), std::string::npos);
    // 告警节流：短时间内重复过期不再输出。
    f.resolver.resolve(13.0);
    EXPECT_EQ(f.warnings.size(), 1U);
}

/// 全静态链（时间戳 0）对任意时刻均视为有效。
TEST(BodyExtrinsicResolver, StaticChainIsAlwaysValid) {
    Fixture f;
    geometry_msgs::msg::TransformStamped transform =
        tf2::eigenToTransform(Eigen::Isometry3d(sensorBody(0.3)));
    transform.header.frame_id = "sensor";
    transform.child_frame_id = "body";
    ASSERT_TRUE(f.buffer.setTransform(transform, "test", true));
    ASSERT_TRUE(f.resolver.tryInitialize(nullptr).has_value());
    const auto result = f.resolver.resolve(1234.5);
    EXPECT_NE(result.source, BodyExtrinsicResolver::Source::Stale);
    EXPECT_TRUE(result.T_sensor_body.isApprox(sensorBody(0.3), 1e-9));
    EXPECT_TRUE(f.warnings.empty());
}

}  // namespace
}  // namespace backend::ros
