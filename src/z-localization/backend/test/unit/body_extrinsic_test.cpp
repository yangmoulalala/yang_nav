// 时间索引外参：精确时刻、容差内的最新变换与过期回退，以及按测量时刻换算的 UWB 杆臂。
#include <gtest/gtest.h>

#include <tf2/buffer_core.h>

#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include "apps/body_extrinsic.hpp"
#include "backend/uwb_fusion.hpp"

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

/// 天线固定在 body 系；云台转动后，传感器系杆臂随测量时刻的外参变化，
/// 而反算回 body 系始终是同一点。
TEST(UwbLeverArm, FollowsExtrinsicAtMeasurementTime) {
    const Eigen::Vector3d antenna_body(0.1, -0.15, 0.6);
    for (const double yaw : {0.0, 0.5, -1.2}) {
        const UwbMeasurementExtrinsic e = uwbExtrinsicFromBody(sensorBody(yaw), antenna_body);
        const Eigen::Vector3d back = e.T_sensor_body.inverse().block<3, 3>(0, 0) *
                                         e.antenna_position_sensor +
                                     e.T_sensor_body.inverse().block<3, 1>(0, 3);
        EXPECT_LT((back - antenna_body).norm(), 1e-12);
    }
    EXPECT_GT((uwbExtrinsicFromBody(sensorBody(0.0), antenna_body).antenna_position_sensor -
               uwbExtrinsicFromBody(sensorBody(0.5), antenna_body).antenna_position_sensor)
                  .norm(),
              0.05);
}

/// 因子构造使用观测携带的测量时刻杆臂，而非配置中的启动杆臂。
TEST(UwbLeverArm, FactorUsesPerMeasurementLeverArm) {
    const Eigen::Vector3d antenna_body(0.1, -0.15, 0.6);
    UwbFactorBuilderConfig config;
    config.antenna_position_sensor =
        uwbExtrinsicFromBody(sensorBody(0.0), antenna_body).antenna_position_sensor;
    UwbFactorBuilder builder(config);

    TimedPoseBuffer odometry;
    odometry.append(10.0, Eigen::Matrix4d::Identity(), 10.0);
    odometry.append(10.2, Eigen::Matrix4d::Identity(), 10.0);
    Keyframe keyframe;
    keyframe.timestamp = 10.1;
    keyframe.odom_pose = Eigen::Matrix4d::Identity();
    keyframe.optimized_pose = Eigen::Matrix4d::Identity();

    UwbObservationStream stream({true, true, 0.0});
    const UwbMeasurementExtrinsic turned = uwbExtrinsicFromBody(sensorBody(0.5), antenna_body);
    ASSERT_TRUE(stream.ingest(10.1, Eigen::Vector3d(1, 2, 0), turned));
    const Eigen::Matrix4d T_map_uwb = Eigen::Matrix4d::Identity();
    const UwbFactorBuildContext context{*stream.latest(), keyframe, odometry, T_map_uwb, {}};
    const auto input = builder.build(context);
    ASSERT_TRUE(input.has_value());
    EXPECT_LT((input->antenna_in_keyframe - turned.antenna_position_sensor).norm(), 1e-9);
    EXPECT_GT((input->antenna_in_keyframe - config.antenna_position_sensor).norm(), 0.05);

    // 未携带时变外参的观测回退到配置杆臂。
    UwbObservationStream fixed({true, true, 0.0});
    ASSERT_TRUE(fixed.ingest(10.1, Eigen::Vector3d(1, 2, 0)));
    const UwbFactorBuildContext fixed_context{*fixed.latest(), keyframe, odometry, T_map_uwb, {}};
    const auto fixed_input = builder.build(fixed_context);
    ASSERT_TRUE(fixed_input.has_value());
    EXPECT_LT((fixed_input->antenna_in_keyframe - config.antenna_position_sensor).norm(), 1e-9);
}

}  // namespace
}  // namespace backend::ros
