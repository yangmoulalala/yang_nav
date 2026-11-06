#include <gtest/gtest.h>

#include "nav_safety/gate_logic.hpp"

using namespace nav_safety;

TEST(ParseTrusted, Variants)
{
  EXPECT_TRUE(parse_trusted("state=NORMAL;pose_trusted=true;epoch=3"));
  EXPECT_TRUE(parse_trusted("pose_trusted = TRUE"));
  EXPECT_FALSE(parse_trusted("state=LOST;pose_trusted=false"));
  EXPECT_FALSE(parse_trusted("state=NORMAL"));
  EXPECT_FALSE(parse_trusted(""));
  EXPECT_FALSE(parse_trusted("not_pose_trusted=true"));
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
  in.trusted = true;
  return in;
}

TEST(Decide, PassWhenFreshAndTrusted)
{
  GateConfig c;
  std::string r;
  EXPECT_EQ(decide(c, fresh_trusted(), r), Decision::Pass);
  EXPECT_TRUE(r.empty());
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
  in.trusted = false;
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
  in.trusted = false;
  in.have_state = false;
  EXPECT_EQ(decide(c, in, r), Decision::Pass);
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
