// 验证先验恢复、常规跟踪与回环的完整提交。
#include "localization_fixture.hpp"

namespace backend {
namespace {
using testing::LocalizationFlow;

/// 验证先验重定位、后续跟踪及输出版本更新，同一几何观测只提交一次。
TEST_F(LocalizationFlow, PriorRecoveryAndTrackingCommitGeometryOnce) {
    const auto anchor = testing::scenePose(.2, -.1, .05, .05);
    config.startup_hint.T_map_sensor = testing::scenePose(.15, -.1, .05, .05);
    start(true);
    const auto prior_revision = prior.revision();
    auto first = frame(10, anchor, Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(first));
    EXPECT_EQ(controller->status(), LocalizationStatus::Initializing);
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(first).T_map_sensor - anchor).norm(), 2e-4);
    const Eigen::Matrix4d truth = anchor * testing::scenePose(.15, 0, 0, 0);
    auto next = frame(11, truth, testing::scenePose(.18, 0, 0, 0));
    ASSERT_TRUE(ingest(next));
    const auto prediction = output(next);
    maintain();
    const auto corrected = output(next);
    ASSERT_TRUE(corrected.valid);
    EXPECT_TRUE(corrected.pose_covariance_valid);
    EXPECT_LT((corrected.T_map_sensor - truth).norm(), .005);
    EXPECT_FALSE(controller->projectionIsCurrent(prediction));
    EXPECT_TRUE(controller->projectionIsCurrent(corrected));
    const auto covariance = corrected.pose_covariance;
    const auto revision = controller->snapshot().pose_revision;
    maintain();
    EXPECT_EQ(controller->snapshot().pose_revision, revision);
    EXPECT_LT((output(next).pose_covariance - covariance).norm(), 1e-12);
    EXPECT_EQ(prior.revision(), prior_revision);
}

/// 在恢复计算期间插入更新帧，检查恢复锚提交后按新帧运动传播输出。
TEST_F(LocalizationFlow, RecoveryPropagatesNewerFrameArrivingBeforeCommit) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.registration_debug_enabled = true;
    start(true);
    auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    auto next = frame(11, testing::scenePose(.2, 0, 0, 0), testing::scenePose(.2, 0, 0, 0));
    ASSERT_TRUE(ingest(first));
    bool received = false;
    visualization.before_commit = [&] { received = ingest(next); };
    maintain();
    ASSERT_TRUE(received);
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_DOUBLE_EQ(trajectory.latest().timestamp, first.timestamp);
    EXPECT_LT((output(next).T_map_sensor - next.odom_pose).norm(), 2e-4);
}

/// 手动重定位请求等待新恢复源，成功后保持先验地图内容与版本。
TEST_F(LocalizationFlow, LostManualRecoveryWaitsForNewDataAndKeepsPriorImmutable) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.recovery.suspect_failures_before_lost = 1;
    config.recovery.automatic_when_lost = false;
    config.uwb.enabled = true;
    start(true);
    ASSERT_TRUE(controller->setUwbTeam("red"));
    auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(first));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto prior_revision = prior.revision();
    auto sparse = frame(11, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    sparse.cloud_local->resize(1);
    ASSERT_TRUE(controller->ingestUwbPosition(11, Eigen::Vector3d::Zero()));
    ASSERT_TRUE(ingest(sparse, Eigen::Vector3d(10, 0, 0)));
    EXPECT_EQ(controller->status(), LocalizationStatus::Suspect);
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Lost);
    auto next = frame(12, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(next));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    ASSERT_TRUE(controller->requestManualRelocalization());
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Lost);
    next.timestamp = 13;
    ASSERT_TRUE(controller->ingestUwbPosition(13, Eigen::Vector3d::Zero()));
    ASSERT_TRUE(ingest(next));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    const auto restored = output(next);
    ASSERT_TRUE(restored.valid);
    EXPECT_TRUE(restored.pose_trusted);
    EXPECT_TRUE(restored.pose_covariance_valid);
    EXPECT_LT((restored.T_map_sensor - Eigen::Matrix4d::Identity()).norm(), 2e-4);
    EXPECT_FALSE(controller->requestManualRelocalization());
    EXPECT_EQ(prior.revision(), prior_revision);
}

/// 执行真实回环检索、几何验证和图提交，检查后续检索遵守冷却间隔。
TEST_F(LocalizationFlow, ActualLoopRetrievalVerificationAndGraphCommitRespectCooldown) {
    config.loop.enabled = true;
    config.loop.cooldown_keyframes = 2;
    config.loop.history_size = 1;
    config.loop.candidate_max_distance = 2;
    start(false);
    for (int index = 0; index < 4; ++index) {
        ASSERT_TRUE(
            ingest(frame(10 + index, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
        maintain();
    }
    const auto edges = controller->visualization().loop_edges;
    ASSERT_EQ(edges.size(), 1U);
    EXPECT_EQ(edges.front().first, 2);
    EXPECT_LT(edges.front().second, edges.front().first);
    const auto revision = controller->snapshot().pose_revision;
    maintain();
    EXPECT_EQ(controller->snapshot().pose_revision, revision);
    const int accepted_index = edges.front().first;
    for (int index = 4; index < 7; ++index) {
        ASSERT_TRUE(
            ingest(frame(10 + index, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity())));
        maintain();
    }
    ASSERT_EQ(controller->visualization().loop_edges.size(), 1U);
    EXPECT_GT(controller->visualization().loop_edges.front().first, accepted_index);
    EXPECT_EQ(controller->visualization().loop_edges.front().first, 6);
    EXPECT_EQ(descriptor.size(), trajectory.size());
}

/// 拒绝不一致回环后，原里程计轨迹和正常跟踪状态保持不变。
TEST_F(LocalizationFlow, InconsistentLoopCannotRewriteOdometryOrDegradeTrackingHealth) {
    config.loop.enabled = true;
    config.loop.source.max_frames = 1;
    config.graph.odom_noise.translation_variance_per_second = 1e-6;
    config.graph.odom_noise.rotation_variance_per_second = 1e-6;
    start(false);
    FrontendFrame current;
    for (int index = 0; index < 4; ++index) {
        current =
            frame(10 + index, Eigen::Matrix4d::Identity(), testing::scenePose(.1 * index, 0, 0, 0));
        ASSERT_TRUE(ingest(current));
        maintain();
    }
    LoopWork candidate_work;
    candidate_work.latest = trajectory.latest();
    candidate_work.trajectory = trajectory.snapshot();
    candidate_work.candidates = descriptor.detectLatestCandidates();
    candidate_work.descriptor_level_with_odom_roll_pitch = descriptor.levelWithOdomRollPitch();
    LoopMatcher matcher({config.loop, config.relocalization.point_to_plane_config});
    ASSERT_TRUE(matcher.compute(candidate_work).selected);
    EXPECT_TRUE(controller->visualization().loop_edges.empty());
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(current).T_map_sensor - current.odom_pose).norm(), 1e-10);
}


/// 局中恢复只在有新鲜 UWB 时进行：没有 UWB 时保持 SUSPECT 等待，不计失败、不进 LOST；
/// 收到 UWB 后在其位置附近匹配恢复。UWB 不用于判断定位是否正常。
TEST_F(LocalizationFlow, InGameRecoveryWaitsForUwbAndThenRecoversNearIt) {
    config.startup_hint.T_map_sensor = Eigen::Matrix4d::Identity();
    config.recovery.suspect_failures_before_lost = 1;
    config.uwb.enabled = true;
    start(true);
    ASSERT_TRUE(controller->setUwbTeam("red"));
    auto first = frame(10, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(first));
    maintain();
    ASSERT_EQ(controller->status(), LocalizationStatus::Normal);
    auto next = frame(11, Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
    ASSERT_TRUE(ingest(next, Eigen::Vector3d(10, 0, 0)));
    ASSERT_EQ(controller->status(), LocalizationStatus::Suspect);
    for (int i = 0; i < 3; ++i) {
        next.timestamp = 12 + i;
        ASSERT_TRUE(ingest(next));
        maintain();
        EXPECT_EQ(controller->status(), LocalizationStatus::Suspect);
    }
    next.timestamp = 15;
    ASSERT_TRUE(controller->ingestUwbPosition(15, Eigen::Vector3d::Zero()));
    ASSERT_TRUE(ingest(next));
    maintain();
    EXPECT_EQ(controller->status(), LocalizationStatus::Normal);
    EXPECT_LT((output(next).T_map_sensor - Eigen::Matrix4d::Identity()).norm(), 2e-4);
}
}  // namespace
}  // namespace backend
