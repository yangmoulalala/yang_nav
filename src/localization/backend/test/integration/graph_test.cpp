// 使用解析位姿链与合成几何检验图估计、降秩似然及回环共识。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "backend/loop_consistency.hpp"
#include "backend/pose_graph.hpp"
#include "backend/registration.hpp"
#include "geometry.hpp"

namespace backend {
namespace {
/// 用解析位姿链检查里程计图估计及累计运动带来的不确定度增长。
TEST(PoseGraph, OdometryChainRecoversGeometryAndUncertaintyGrows) {
    PoseGraph graph;
    graph.addPrior(0, Eigen::Matrix4d::Identity(), PoseCovariance6d::Identity() * .01, 1);
    graph.update(false);
    for (int index = 1; index <= 4; ++index) {
        graph.addOdometryMeasurement(index - 1, index, testing::translation(1),
                                     testing::translation(index + .1),
                                     PoseCovariance6d::Identity() * .01, index + 1);
        graph.update(false);
    }
    const auto pose = graph.estimate(4);
    ASSERT_TRUE(pose);
    EXPECT_LT((*pose - testing::translation(4)).norm(), 1e-7);
    const auto first_covariance = graph.marginalCovariance(0);
    const auto last_covariance = graph.marginalCovariance(4);
    ASSERT_TRUE(first_covariance);
    ASSERT_TRUE(last_covariance);
    EXPECT_GT((*last_covariance)(0, 0), (*first_covariance)(0, 0));
    EXPECT_FALSE(graph.estimate(999));
    graph.reset({});
    EXPECT_FALSE(graph.hasEstimate(4));
}

/// 固定滞后窗口移除过期节点后，当前位姿及其边缘协方差仍可查询。
TEST(PoseGraph, FixedLagRetiresOldStatesWithoutLosingCurrentMotion) {
    PoseGraphConfig config;
    config.fixed_lag_seconds = 2;
    PoseGraph graph(config);
    graph.addPrior(0, Eigen::Matrix4d::Identity(), 1);
    graph.update(false);
    for (int index = 1; index <= 8; ++index) {
        graph.addOdometryMeasurement(index - 1, index, testing::translation(1),
                                     testing::translation(index),
                                     PoseCovariance6d::Identity() * .01, index + 1);
        graph.update(false);
    }
    EXPECT_FALSE(graph.hasEstimate(0));
    EXPECT_FALSE(graph.marginalCovariance(0));
    const auto current = graph.estimate(8);
    ASSERT_TRUE(current);
    EXPECT_LT((*current - testing::translation(8)).norm(), 1e-7);
    EXPECT_TRUE(graph.marginalCovariance(8));
}

/// 构造具有精确二进制坐标的窗口轨迹，避免观测残差引发额外重线性化。
Eigen::Matrix4d windowPose(int index) {
    return testing::translation(.25 * index, .125 * (index % 3), .0625 * (index % 5));
}

/// 秩一位置观测仅约束对应平移方向，其余方向沿用图预测。
TEST(PoseGraph, RankOneObservationConstrainsOnlyMeasuredTranslation) {
    PoseGraph graph;
    graph.addPrior(0, Eigen::Matrix4d::Identity(), PoseCovariance6d::Identity(), 1);
    graph.update(false);
    PoseSqrtInformation information = PoseSqrtInformation::Zero(1, 6);
    information(0, 0) = 2;
    graph.addAbsolutePoseInformation(0, testing::translation(.2), information);
    graph.update(true);
    const auto pose = graph.estimate(0);
    const auto covariance = graph.marginalCovariance(0);
    ASSERT_TRUE(pose);
    ASSERT_TRUE(covariance);
    EXPECT_NEAR((*pose)(0, 3), .16, 1e-6);
    EXPECT_NEAR((*covariance)(0, 0), .2, 1e-6);
    EXPECT_NEAR((*covariance)(1, 1), 1, 1e-6);
}

/// 单平面配准提供法向约束，但无法通过完整位姿锚的验收。
TEST(Registration, PlaneConstrainsNormalMotionWithoutInventingFullPoseAnchor) {
    RegistrationConfig config;
    config.voxel_resolution = 0;
    config.min_correspondences = 20;
    config.plane_knn = 8;
    Registration registration(config);
    const auto cloud = testing::planeCloud();
    ASSERT_TRUE(registration.setTarget(cloud, 0));
    const auto result = registration.align(*cloud, testing::translation(0, 0, .04));
    ASSERT_TRUE(result.likelihoodValid());
    EXPECT_NEAR(result.source_to_target(2, 3), 0, 1e-5);
    EXPECT_LT(result.measurement_information.rank, 6);
    EXPECT_FALSE(result.anchorValid());
    const auto scaled = registration.align(*cloud, testing::translation(0, 0, .04), .25);
    ASSERT_TRUE(scaled.likelihoodValid());
    EXPECT_LT((scaled.source_to_target - result.source_to_target).norm(), 1e-9);
    const auto precision = result.measurement_information.sqrt_information.transpose() *
                           result.measurement_information.sqrt_information;
    const auto scaled_precision = scaled.measurement_information.sqrt_information.transpose() *
                                  scaled.measurement_information.sqrt_information;
    EXPECT_LT(scaled_precision.trace(), precision.trace());
}

/// 回环候选先按一致集合规模筛选，再在集合内按残差选择。
TEST(LoopConsensus, SelectsLargestGroupBeforeBestIndividualResidual) {
    LoopClosureConfig config;
    config.enable_pcm = true;
    config.pcm_translation_threshold = .2;
    LoopClosureVerifier verifier(config);
    std::vector<VerifiedLoop> candidates(5);
    for (int index = 0; index < 5; ++index) {
        candidates[index].target_keyframe.odom_pose = testing::translation(index);
        candidates[index].measured_between = testing::translation(index + (index < 3 ? 0 : 10));
        candidates[index].registration.rmse = index < 3 ? .2 : .01;
        candidates[index].candidate.descriptor_distance = .1;
    }
    auto selected = verifier.selectConsistentCandidateClique(candidates);
    std::sort(selected.begin(), selected.end());
    EXPECT_EQ(selected, (std::vector<int>{0, 1, 2}));
    candidates[2].target_keyframe.epoch = 1;
    EXPECT_FALSE(verifier.areSameSourceLoopsConsistent(candidates[0], candidates[2]));
}

/// 区分已有历史下的一致边数量要求与没有可比较历史时的启动放行。
TEST(LoopConsensus, HistoricalSupportThresholdAndMissingHistoryHaveSeparateMeaning) {
    LoopClosureConfig config;
    config.min_consistent_loops = 2;
    config.pcm_translation_threshold = .2;
    LoopClosureVerifier verifier(config);
    Keyframe latest, candidate;
    const auto lookup = [](int, Keyframe* out) {
        *out = Keyframe{};
        return true;
    };
    EXPECT_TRUE(verifier.passesAcceptedConsistency(latest, candidate, Eigen::Matrix4d::Identity(),
                                                   {}, lookup));
    std::vector<AcceptedLoop> history{{0, 1, Eigen::Matrix4d::Identity()}};
    EXPECT_FALSE(verifier.passesAcceptedConsistency(latest, candidate, Eigen::Matrix4d::Identity(),
                                                    history, lookup));
    history.push_back({2, 3, Eigen::Matrix4d::Identity()});
    EXPECT_TRUE(verifier.passesAcceptedConsistency(latest, candidate, Eigen::Matrix4d::Identity(),
                                                   history, lookup));
    EXPECT_FALSE(verifier.passesAcceptedConsistency(latest, candidate, testing::translation(3),
                                                    history, lookup));
}

}  // namespace
}  // namespace backend
