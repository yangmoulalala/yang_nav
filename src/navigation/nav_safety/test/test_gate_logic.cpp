#include <gtest/gtest.h>
#include <limits>

#include "nav_safety/gate_logic.hpp"

using namespace nav_safety;

TEST(Decide, RequiresFreshProcessedObstacleObservation)
{
  GateConfig cfg;
  cfg.require_trusted = false;
  cfg.require_observation = true;
  GateInput in;
  in.have_cmd = true;
  std::string reason;
  EXPECT_EQ(decide(cfg, in, reason), Decision::Hold);
  in.have_observation = true;
  in.observation_age = 0.1;
  EXPECT_EQ(decide(cfg, in, reason), Decision::Pass);
  for (double age : {0.6, -0.1, std::numeric_limits<double>::quiet_NaN()}) {
    in.observation_age = age;
    EXPECT_EQ(decide(cfg, in, reason), Decision::Hold);
  }
}

TEST(Limit, RejectsNonFiniteCommands)
{
  const auto out = limit({std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0}, {1.5, 2.0});
  EXPECT_DOUBLE_EQ(out.vx, 0.0);
  EXPECT_DOUBLE_EQ(out.vy, 0.0);
}

TEST(ParseLocalization, Variants)
{
  EXPECT_EQ(parse_localization("state=NORMAL;pose_trusted=true;epoch=3"), Localization::Trusted);
  EXPECT_EQ(parse_localization("status=OBS_DEGRADED;pose_trusted = TRUE"), Localization::Trusted);
  EXPECT_EQ(parse_localization("status=SUSPECT;pose_trusted=false;pose_extrapolated=true"),
    Localization::Extrapolated);
  for (const auto * state : {"status=LOST;pose_trusted=false;pose_extrapolated=true",
    "status=INITIALIZING;pose_trusted=false", "status=SUSPECT;pose_extrapolated=false",
    "status=SUSPECT", "state=NORMAL", "", "not_pose_trusted=true"})
  {
    EXPECT_EQ(parse_localization(state), Localization::Unavailable);
  }
}

TEST(ParseMode, Known)
{
  EXPECT_EQ(parse_mode("tunnel"), Mode::Tunnel);
  EXPECT_EQ(parse_mode("ramp"), Mode::Ramp);
  EXPECT_FALSE(parse_mode("fast").has_value());
}

static GateInput fresh_trusted()
{
  GateInput in;
  in.have_cmd = true;
  in.cmd_age = 0.05;
  in.have_state = true;
  in.state_age = 0.1;
  in.localization = Localization::Trusted;
  return in;
}

TEST(Decide, PassWhenFreshAndTrusted)
{
  GateConfig c;
  std::string r;
  EXPECT_EQ(decide(c, fresh_trusted(), r), Decision::Pass);
  EXPECT_TRUE(r.empty());
}

TEST(Decide, ExtrapolatedPoseStillRequiresFreshStateAndObstacles)
{
  GateConfig cfg;
  cfg.require_observation = true;
  auto input = fresh_trusted();
  input.localization = Localization::Extrapolated;
  input.have_observation = true;
  std::string reason;
  EXPECT_EQ(decide(cfg, input, reason), Decision::Pass);
  input.state_age = 2.0;
  EXPECT_EQ(decide(cfg, input, reason), Decision::Hold);
  input.state_age = 0.1;
  input.observation_age = 0.6;
  EXPECT_EQ(decide(cfg, input, reason), Decision::Hold);
}

TEST(Decide, IdleOnStaleOrMissingInput)
{
  GateConfig c;
  std::string r;
  auto in = fresh_trusted();
  in.cmd_age = 0.3;
  EXPECT_EQ(decide(c, in, r), Decision::Idle);
  in.have_cmd = false;
  EXPECT_EQ(decide(c, in, r), Decision::Idle);
}

TEST(Decide, HoldWhenUntrustedOrStale)
{
  GateConfig c;
  std::string r;
  auto in = fresh_trusted();
  in.localization = Localization::Unavailable;
  EXPECT_EQ(decide(c, in, r), Decision::Hold);
  in = fresh_trusted();
  in.state_age = 2.0;
  EXPECT_EQ(decide(c, in, r), Decision::Hold);
  in = fresh_trusted();
  in.have_state = false;
  EXPECT_EQ(decide(c, in, r), Decision::Hold);
}

TEST(Decide, TrustNotRequired)
{
  GateConfig c;
  c.require_trusted = false;
  std::string r;
  auto in = fresh_trusted();
  in.localization = Localization::Unavailable;
  in.have_state = false;
  EXPECT_EQ(decide(c, in, r), Decision::Pass);
}

TEST(Limit, ExtrapolationCapDoesNotRaiseOtherLimits)
{
  const auto capped = motion_limits({2.0, 2.5}, 1.2, Localization::Extrapolated);
  EXPECT_DOUBLE_EQ(capped.max_linear, 0.6);
  EXPECT_DOUBLE_EQ(capped.max_angular, 1.0);
  const auto stricter = motion_limits({0.5, 0.7}, 0.4, Localization::Extrapolated);
  EXPECT_DOUBLE_EQ(stricter.max_linear, 0.4);
  EXPECT_DOUBLE_EQ(stricter.max_angular, 0.7);
}

TEST(Limit, KeepsDirection)
{
  const Cmd c = limit({3.0, 4.0, 5.0}, {1.0, 2.0});
  EXPECT_NEAR(c.vx, 0.6, 1e-9);
  EXPECT_NEAR(c.vy, 0.8, 1e-9);
  EXPECT_NEAR(c.wz, 2.0, 1e-9);
  const Cmd d = limit({0.3, -0.4, -0.5}, {1.0, 2.0});
  EXPECT_NEAR(d.vx, 0.3, 1e-9);
  EXPECT_NEAR(d.vy, -0.4, 1e-9);
  EXPECT_NEAR(d.wz, -0.5, 1e-9);
}
