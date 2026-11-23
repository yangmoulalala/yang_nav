// TF 外参 T_sensor_body 与旧版 offset/rpy 表达必须给出同一车体位姿。
#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "backend/body.hpp"
#include "backend/math.hpp"

namespace backend {
namespace {

/// 构造非平凡的 T_sensor_body：三轴旋转加任意平移。
Eigen::Matrix4d nontrivialSensorBody() {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = rpyDegreesToRotation(-14.94, 0.523, -150.6);
    T.block<3, 1>(0, 3) = Eigen::Vector3d(0.12, -0.34, 0.05);
    return T;
}

/// 传感器位于父系原点时，车体位姿即为 TF 给出的 T_sensor_body。
TEST(BodyTransform, IdentitySensorPoseYieldsTfExtrinsic) {
    const Eigen::Matrix4d T_sensor_body = nontrivialSensorBody();
    BodyTransform body;
    body.configure(T_sensor_body);
    EXPECT_TRUE(body.bodyPoseFromSensorPose(Eigen::Matrix4d::Identity())
                    .isApprox(T_sensor_body, 1e-12));
}

/// 任意父系传感器位姿下满足 T_parent_body = T_parent_sensor * T_sensor_body，且反变换互逆。
TEST(BodyTransform, ComposesWithParentPoseAndInverts) {
    const Eigen::Matrix4d T_sensor_body = nontrivialSensorBody();
    BodyTransform body;
    body.configure(T_sensor_body);
    Eigen::Matrix4d T_parent_sensor = Eigen::Matrix4d::Identity();
    T_parent_sensor.block<3, 3>(0, 0) = rpyDegreesToRotation(3.0, -7.0, 75.0);
    T_parent_sensor.block<3, 1>(0, 3) = Eigen::Vector3d(4.0, -2.0, 0.5);
    const Eigen::Matrix4d T_parent_body = body.bodyPoseFromSensorPose(T_parent_sensor);
    EXPECT_TRUE(T_parent_body.isApprox(T_parent_sensor * T_sensor_body, 1e-12));
    EXPECT_TRUE(body.sensorPoseFromBodyPose(T_parent_body).isApprox(T_parent_sensor, 1e-12));
}

/// 矩阵重载与旧版 offset/rpy 语义一致：R_body_sensor = R_sb^T，offset = R_sb^T * t_sb。
TEST(BodyTransform, MatchesLegacyOffsetRotationConvention) {
    const Eigen::Matrix4d T_sensor_body = nontrivialSensorBody();
    const Eigen::Matrix3d R_sb = T_sensor_body.block<3, 3>(0, 0);
    const Eigen::Vector3d offset = R_sb.transpose() * T_sensor_body.block<3, 1>(0, 3);
    BodyTransform legacy;
    legacy.configure(offset, Eigen::Vector3d(14.94, -0.523, 150.6));
    BodyTransform legacy_exact;
    legacy_exact.configure(offset, Eigen::Vector3d::Zero());
    BodyTransform from_tf;
    from_tf.configure(T_sensor_body);
    EXPECT_TRUE(from_tf.offset().isApprox(offset, 1e-12));
    // 旧版 rpy 描述 R_body_sensor；由 R_sb^T 反解欧拉角后应得到同一车体位姿。
    const Eigen::Vector3d rpy = rpyFromRotation(R_sb.transpose()) * 180.0 / M_PI;
    BodyTransform legacy_equivalent;
    legacy_equivalent.configure(offset, rpy);
    EXPECT_TRUE(legacy_equivalent.bodyPoseFromSensorPose(Eigen::Matrix4d::Identity())
                    .isApprox(T_sensor_body, 1e-9));
}

}  // namespace
}  // namespace backend
