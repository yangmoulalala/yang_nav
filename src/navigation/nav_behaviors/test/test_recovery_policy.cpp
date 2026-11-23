#include <gtest/gtest.h>

#include <string>

#include "nav_behaviors/recovery_policy.hpp"

using namespace nav_behaviors;  // NOLINT

namespace
{
RecoveryConfig ladder()
{
  return RecoveryConfig{};   // retries 6, spin at 1, back-up at 2
}
}  // namespace

TEST(RecoveryPolicy, FirstAttemptOnlyClears)
{
  // a stale mark is the most common failure and costs no motion
  EXPECT_EQ(recoveryAction(ladder(), 0, false), RecoveryAction::ClearCostmaps);
}

TEST(RecoveryPolicy, EscalatesSpinThenBackUp)
{
  const auto c = ladder();
  EXPECT_EQ(recoveryAction(c, 1, false), RecoveryAction::Spin);
  EXPECT_EQ(recoveryAction(c, 2, false), RecoveryAction::BackUp);
  EXPECT_EQ(recoveryAction(c, 5, false), RecoveryAction::BackUp);
}

TEST(RecoveryPolicy, GivesUpAfterTheRetries)
{
  const auto c = ladder();   // retries 6 -> attempts 0..5 recover, 6 gives up
  EXPECT_EQ(recoveryAction(c, 5, false), RecoveryAction::BackUp);
  EXPECT_EQ(recoveryAction(c, 6, false), RecoveryAction::GiveUp);
  EXPECT_EQ(recoveryAction(c, 60, false), RecoveryAction::GiveUp);
}

TEST(RecoveryPolicy, ZeroRetriesFailsImmediately)
{
  auto c = ladder();
  c.retries = 0;
  EXPECT_EQ(recoveryAction(c, 0, false), RecoveryAction::GiveUp);
}

TEST(RecoveryPolicy, NoMotionInsideAZone)
{
  // ALIGN / TRAVERSE / EXIT: a spin sweeps the corners into the walls, a back-up loses the alignment
  for (int attempt = 0; attempt <= 8; ++attempt) {
    SCOPED_TRACE(attempt);
    const auto a = recoveryAction(ladder(), attempt, true);
    EXPECT_TRUE(a == RecoveryAction::ClearCostmaps || a == RecoveryAction::GiveUp);
  }
  EXPECT_EQ(recoveryAction(ladder(), 3, true), RecoveryAction::ClearCostmaps);
  EXPECT_EQ(recoveryAction(ladder(), 6, true), RecoveryAction::GiveUp);
}

TEST(RecoveryPolicy, MotionCanBeDisabledPerAction)
{
  auto c = ladder();
  c.spin = false;
  EXPECT_EQ(recoveryAction(c, 1, false), RecoveryAction::ClearCostmaps);
  c.backup = false;
  EXPECT_EQ(recoveryAction(c, 3, false), RecoveryAction::ClearCostmaps);

  auto d = ladder();
  d.spin_after = 3;
  d.backup_after = 4;
  EXPECT_EQ(recoveryAction(d, 2, false), RecoveryAction::ClearCostmaps);
  EXPECT_EQ(recoveryAction(d, 3, false), RecoveryAction::Spin);
  EXPECT_EQ(recoveryAction(d, 4, false), RecoveryAction::BackUp);
}

TEST(RecoveryPolicy, NegativeRetriesBehaveLikeZero)
{
  auto c = ladder();
  c.retries = -3;
  EXPECT_EQ(recoveryAction(c, 0, false), RecoveryAction::GiveUp);
}

TEST(RecoveryPolicy, NamesCoverEveryAction)
{
  EXPECT_STREQ(recoveryActionName(RecoveryAction::ClearCostmaps), "clear");
  EXPECT_STREQ(recoveryActionName(RecoveryAction::Spin), "spin");
  EXPECT_STREQ(recoveryActionName(RecoveryAction::BackUp), "backup");
  EXPECT_STREQ(recoveryActionName(RecoveryAction::GiveUp), "give_up");
}
