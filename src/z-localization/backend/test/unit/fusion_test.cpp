// 全局观测集合的联合接纳策略。
#include <gtest/gtest.h>

#include "backend/observation.hpp"

namespace backend {
namespace {
/// 缺少时间或几何支撑的观测保持不可用状态，不累计冲突。
TEST(ObservationPolicy, UnavailableEvidenceIsNotAConflictVote) {
    ObservationBatch batch;
    batch.unavailable.push_back("no temporal support");
    EXPECT_EQ(batch.conclusion().outcome, ObservationOutcome::Unavailable);
    EXPECT_FALSE(batch.conclusion().had_conflict);
}

/// 联合一致时接受全部条目；逐项诊断数量不完整时拒绝。
TEST(ObservationPolicy, JointAssessmentControlsAcceptance) {
    ObservationBatch batch;
    batch.entries = {{ObservationSource::PriorMap, "map", {}, false}};
    InnovationBatchAssessment assessment;
    assessment.joint = {true, 6, .2, .9, {}};
    assessment.individual = {{true, 6, .2, .9, {}}};
    ASSERT_TRUE(batch.validate(assessment));
    batch.classify(assessment, .001);
    EXPECT_EQ(batch.conclusion().outcome, ObservationOutcome::Accepted);
    EXPECT_TRUE(batch.entries[0].accepted);
    ObservationBatch inconsistent;
    inconsistent.entries = batch.entries;
    assessment.individual.clear();
    EXPECT_FALSE(inconsistent.validate(assessment));
}

/// 先验地图与预测冲突时结论为冲突。
TEST(ObservationPolicy, ConflictingMapIsAConflict) {
    ObservationBatch batch;
    batch.entries = {{ObservationSource::PriorMap, "map", {}, false}};
    InnovationBatchAssessment assessment;
    assessment.joint = {true, 6, 290, 1e-50, {}};
    assessment.individual = {{true, 6, 290, 1e-50, {}}};
    ASSERT_TRUE(batch.validate(assessment));
    batch.classify(assessment, .001);
    EXPECT_FALSE(batch.entries[0].accepted);
    EXPECT_EQ(batch.conclusion().outcome, ObservationOutcome::Conflict);
}
}  // namespace
}  // namespace backend
