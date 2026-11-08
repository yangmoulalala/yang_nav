// 验证停车、暖重置与跨代运动的位姿和协方差连续性。
#include "localization_fixture.hpp"

namespace backend {
namespace {
using testing::LocalizationFlow;

/// 首个新代次观测保留边界之后的实际位移，不把重置后的运动吸收到地图变换中。
TEST_F(LocalizationFlow, WarmBoundaryPreservesMotionBeforeTheFirstNewFrame) {
    reset.available = true;
    start(false);
    const auto initial_pose = testing::scenePose(2, 0, 0, 0);
    const auto first = frame(10, initial_pose, initial_pose);
    ASSERT_TRUE(ingest(first, Eigen::Vector3d::UnitX()));
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(10), initial_pose});
    for (double elapsed : {.1, .2}) {
        const auto local = testing::scenePose(elapsed, 0, 0, 0);
        const Eigen::Matrix4d expected = initial_pose * local;
        const auto current = frame(10 + elapsed, expected, local, 1);
        ASSERT_TRUE(ingest(current, Eigen::Vector3d::UnitX()));
        const auto projected = output(current);
        ASSERT_TRUE(projected.valid);
        EXPECT_TRUE(projected.pose_trusted);
        EXPECT_NEAR(projected.T_map_sensor(0, 3), 2 + elapsed, 1e-10);
    }
}

/// 连续暖重置复合旋转规范，并在延迟首关键帧入图时覆盖整个物理运动区间。
TEST_F(LocalizationFlow, ConsecutiveWarmBoundariesPreserveDelayedGraphMotionAndUncertainty) {
    config.loop.enabled = true;
    config.keyframes.min_interval = .5;
    reset.available = true;
    const auto map_from_old = testing::scenePose(4, -3, 1, .6);
    const auto old_pose = testing::scenePose(1, .5, .2, -.3);
    const Eigen::Matrix4d initial_pose = map_from_old * old_pose;
    config.startup_hint.T_map_sensor = initial_pose;
    start(false);
    ASSERT_TRUE(ingest(frame(10, initial_pose, old_pose)));
    ASSERT_EQ(trajectory.size(), 1U);
    const auto newer_odom = testing::scenePose(1.1, .5, .2, -.2);
    ASSERT_TRUE(ingest(frame(10.1, map_from_old * newer_odom, newer_odom)));
    EXPECT_DOUBLE_EQ(trajectory.latest().timestamp, 10);

    const auto old_from_middle = testing::scenePose(1.2, .4, .2, .4);
    const auto middle_from_new = testing::scenePose(.2, -.1, 0, -.25);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(10.2), old_from_middle});
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 2,
          FrontendResetBoundary{1, Timestamp::fromSeconds(10.4), middle_from_new});

    const auto new_pose = testing::scenePose(.4, .1, 0, .2);
    const Eigen::Matrix4d expected = map_from_old * old_from_middle * middle_from_new * new_pose;
    const auto current = frame(10.8, expected, new_pose, 2);
    ASSERT_TRUE(ingest(current));
    ASSERT_EQ(trajectory.size(), 2U);
    EXPECT_EQ(trajectory.latest().epoch, 2U);
    EXPECT_LT((trajectory.latest().optimized_pose - expected).norm(), 1e-9);
    const auto projected = output(current);
    ASSERT_TRUE(projected.pose_covariance_valid);
    EXPECT_LT((projected.T_map_sensor - expected).norm(), 1e-9);

    // 独立两节点图使用真实首尾位姿和完整 0.8 秒过程噪声，不加入停车重置噪声。
    PoseGraph reference(config.graph);
    reference.addPrior(0, initial_pose, 10);
    reference.update(false);
    const Eigen::Matrix4d measured_between = initial_pose.inverse() * expected;
    const auto relative_covariance = config.graph.odom_noise.covariance(
        .8, measured_between.block<3, 1>(0, 3).norm(), rotationAngleFromMatrix(measured_between));
    reference.addResetContinuityMeasurement(0, 1, measured_between, expected, relative_covariance,
                                            10.8);
    reference.update(false);
    const auto covariance = reference.marginalCovariance(1);
    ASSERT_TRUE(covariance);
    const auto expected_covariance = localToWorldAdditivePoseCovariance(expected, *covariance);
    EXPECT_LT((projected.pose_covariance - expected_covariance).norm(), 1e-9);
    const auto next_local = testing::scenePose(.9, .2, 0, .3);
    const Eigen::Matrix4d next_expected =
        map_from_old * old_from_middle * middle_from_new * next_local;
    const auto next = frame(11.5, next_expected, next_local, 2);
    ASSERT_TRUE(ingest(next));
    ASSERT_EQ(trajectory.size(), 3U);
    EXPECT_LT((trajectory.latest().optimized_pose - next_expected).norm(), 1e-9);
}

/// 非法边界时间不能改变坐标规范；终态仍换代并采用无运动边界的停车合同。
TEST_F(LocalizationFlow, StaleWarmBoundaryCannotSupplyANewCoordinateTransform) {
    reset.available = true;
    start(false);
    const auto initial_pose = testing::scenePose(2, 0, 0, 0);
    ASSERT_TRUE(ingest(frame(10, initial_pose, initial_pose)));
    ASSERT_TRUE(controller->requestCoordinatedReset(/*assume_stationary=*/true));
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(9), testing::scenePose(100, 0, 0, 0)});
    EXPECT_EQ(controller->activeEpoch(), 1U);
    const auto current = frame(10.1, initial_pose, testing::scenePose(.1, 0, 0, 0), 1);
    ASSERT_TRUE(ingest(current));
    EXPECT_LT((output(current).T_map_sensor - initial_pose).norm(), 1e-10);
}

/// 边界之前及同刻的新代次帧均不得消费种子，后续合法观测仍可正常入图。
TEST_F(LocalizationFlow, WarmBoundaryRejectsNoncausalFramesWithoutConsumingTheTransform) {
    reset.available = true;
    start(false);
    ASSERT_TRUE(ingest(frame(10, testing::scenePose(2, 0, 0, 0), testing::scenePose(2, 0, 0, 0))));
    const auto boundary_pose = testing::scenePose(2.2, 0, 0, 0);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(10.2), boundary_pose});
    for (double time : {10.1, 10.2})
        EXPECT_FALSE(ingest(frame(time, boundary_pose, testing::scenePose(100, 0, 0, 0), 1)));
    EXPECT_EQ(trajectory.size(), 1U);
    const auto local = testing::scenePose(.1, 0, 0, 0);
    const Eigen::Matrix4d expected = boundary_pose * local;
    const auto current = frame(10.3, expected, local, 1);
    ASSERT_TRUE(ingest(current));
    EXPECT_LT((output(current).T_map_sensor - expected).norm(), 1e-10);
    EXPECT_EQ(trajectory.size(), 2U);
}

/// 坐标边界只转换里程计规范，不为先验定位中已经丢失的状态恢复可信度。
TEST_F(LocalizationFlow, WarmBoundaryCannotMakeLostPriorLocalizationTrusted) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.recovery.suspect_failures_before_lost = 1;
    config.recovery.automatic_when_lost = false;
    config.uwb.enabled = true;
    reset.available = true;
    start(true);
    ASSERT_TRUE(controller->setUwbTeam("red"));
    const auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(first));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto trusted = output(first);
    auto sparse = frame(11, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    sparse.cloud_local->resize(1);
    ASSERT_TRUE(controller->ingestUwbPosition(11, Eigen::Vector3d::Zero()));
    ASSERT_TRUE(ingest(sparse, Eigen::Vector3d(10, 0, 0)));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Lost);
    ASSERT_TRUE(controller->requestCoordinatedReset());
    const auto boundary_pose = testing::scenePose(.2, 0, 0, .1);
    reply(FrontendResetOutcome::Completed, 1,
          FrontendResetBoundary{0, Timestamp::fromSeconds(11.1), boundary_pose});
    const auto local = testing::scenePose(.1, 0, 0, 0);
    const auto current = frame(11.2, boundary_pose * local, local, 1);
    ASSERT_TRUE(ingest(current));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    const auto projected = output(current);
    EXPECT_FALSE(projected.pose_trusted);
    EXPECT_LT((projected.T_map_sensor - trusted.T_map_sensor).norm(), 1e-10);
}
}  // namespace
}  // namespace backend
