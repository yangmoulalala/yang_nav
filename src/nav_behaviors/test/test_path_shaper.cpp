#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/math_utils.hpp"

using nav_behaviors::CollisionFree;
using nav_behaviors::normAngle;
using nav_behaviors::Pose2;
using nav_behaviors::shapePath;
using nav_behaviors::ShapeEvent;
using nav_behaviors::ShaperParams;
using nav_behaviors::Zone;
using nav_behaviors::TurnMode;

namespace
{

// planner-like path: poses every `step` m, yaw = segment direction
std::vector<Pose2> polyline(const std::vector<std::pair<double, double>> & pts, double step = 0.05)
{
  std::vector<Pose2> out;
  for (size_t k = 0; k + 1 < pts.size(); ++k) {
    const double dx = pts[k + 1].first - pts[k].first, dy = pts[k + 1].second - pts[k].second;
    const double yaw = std::atan2(dy, dx);
    const int n = std::max(1, static_cast<int>(std::round(std::hypot(dx, dy) / step)));
    for (int i = 0; i < n; ++i) {
      out.push_back({pts[k].first + dx * i / n, pts[k].second + dy * i / n, yaw});
    }
  }
  out.push_back({pts.back().first, pts.back().second, out.back().yaw});
  return out;
}

Zone tunnel(double cx, double cy, double yaw_deg, const char * id = "t")
{
  Zone t;
  t.id = id;
  t.cx = cx;
  t.cy = cy;
  t.yaw = yaw_deg * M_PI / 180.0;
  t.depth = 0.35;
  t.width = 0.8;
  return t;
}

const CollisionFree kFree = [](const Pose2 &) {return true;};

// axis-aligned wall boxes, robot outline 0.70 x 0.62 (padded footprint) sampled along its edges
struct Box
{
  double x0, x1, y0, y1;
};

CollisionFree walls(const std::vector<Box> & boxes)
{
  return [boxes](const Pose2 & q) {
           const double fx[4] = {0.39, 0.39, -0.31, -0.31}, fy[4] = {0.31, -0.31, -0.31, 0.31};
           const double c = std::cos(q.yaw), s = std::sin(q.yaw);
           for (int k = 0; k < 4; ++k) {
             for (int i = 0; i <= 40; ++i) {
               const double a = fx[k] + (fx[(k + 1) % 4] - fx[k]) * i / 40.0;
               const double b = fy[k] + (fy[(k + 1) % 4] - fy[k]) * i / 40.0;
               const double x = q.x + c * a - s * b, y = q.y + s * a + c * b;
               for (const auto & w : boxes) {
                 if (x > w.x0 && x < w.x1 && y > w.y0 && y < w.y1) {
                   return false;
                 }
               }
             }
           }
           return true;
         };
}

// every pose within the straight axis is on the zone axis with the locked heading
void expectOnAxis(const std::vector<Pose2> & p, const Zone & t, double heading, double margin = 0.5)
{
  int n = 0;
  for (const auto & q : p) {
    double u, v;
    t.toLocal(q.x, q.y, u, v);
    if (std::abs(u) < 0.5 * t.depth + margin - 1e-6 && std::abs(v) < 0.5) {
      ++n;
      EXPECT_NEAR(v, 0.0, 1e-9) << "u " << u;
      EXPECT_NEAR(normAngle(q.yaw - heading), 0.0, 1e-9) << "u " << u;
    }
  }
  EXPECT_GT(n, 40);   // 1.35 m of axis at 0.025 m
}

double maxStep(const std::vector<Pose2> & p)
{
  double m = 0.0;
  for (size_t i = 1; i < p.size(); ++i) {
    m = std::max(m, std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y));
  }
  return m;
}

// largest direction change between consecutive segments (G1 check)
double maxTurn(const std::vector<Pose2> & p)
{
  double m = 0.0, prev = 0.0;
  bool have = false;
  for (size_t i = 1; i < p.size(); ++i) {
    const double dx = p[i].x - p[i - 1].x, dy = p[i].y - p[i - 1].y;
    if (std::hypot(dx, dy) < 1e-6) {
      continue;
    }
    const double a = std::atan2(dy, dx);
    if (have) {
      m = std::max(m, std::abs(normAngle(a - prev)));
    }
    prev = a;
    have = true;
  }
  return m;
}

double maxYawStep(const std::vector<Pose2> & p)
{
  double m = 0.0;
  for (size_t i = 1; i < p.size(); ++i) {
    m = std::max(m, std::abs(normAngle(p[i].yaw - p[i - 1].yaw)));
  }
  return m;
}

}  // namespace

TEST(PathShaper, GridOffsetSnapsToAxis)
{
  const auto t = tunnel(0, 0, 0);
  const auto in = polyline({{-2.5, 0.05}, {2.5, 0.05}});   // Smac one cell off the axis
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_EQ(ev[0].direction, 1);
  expectOnAxis(out, t, 0.0);
  EXPECT_LE(maxStep(out), 0.0501);
  EXPECT_LT(maxTurn(out), 0.05);
  EXPECT_LT(maxYawStep(out), 0.05);
  EXPECT_DOUBLE_EQ(out.front().x, in.front().x);
  EXPECT_DOUBLE_EQ(out.back().x, in.back().x);
  EXPECT_DOUBLE_EQ(out.back().y, in.back().y);
}

TEST(PathShaper, ReverseDirectionLocksOppositeHeading)
{
  const auto t = tunnel(0, 0, 0);
  const auto in = polyline({{2.5, -0.04}, {-2.5, -0.04}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_EQ(ev[0].direction, -1);
  expectOnAxis(out, t, M_PI);
}

TEST(PathShaper, DiagonalApproachTurnsWhileDriving)
{
  const auto t = tunnel(0, 0, 0);
  const auto in = polyline({{-2.2, -1.6}, {-0.9, 0.0}, {2.5, 0.0}});   // sharp corner before the mouth
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  expectOnAxis(out, t, 0.0);
  EXPECT_LE(maxStep(out), std::max(0.0501, maxStep(in) + 1e-9));   // untouched input keeps its own spacing
  EXPECT_LT(maxTurn(out), 0.1);      // corner replaced by the lead-in curve
  EXPECT_LT(maxYawStep(out), 0.05);  // ~51 deg turned over the lead-in, no jump
}

TEST(PathShaper, RotatedZoneFromMap)
{
  const auto t = tunnel(4.8, -4.975, 90.4);   // tunnel_3 in maps/sim
  const auto in = polyline({{4.83, -7.0}, {4.78, -3.0}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  expectOnAxis(out, t, t.yaw);
}

TEST(PathShaper, TwoTunnelsOnOnePath)
{
  const auto a = tunnel(0, 0, 0, "a"), b = tunnel(4, 0, 0, "b");
  const auto in = polyline({{-3.0, 0.04}, {7.0, 0.04}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {b, a}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 2u);
  EXPECT_EQ(ev[0].zone_id, "a");   // in path order, not list order
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_TRUE(ev[1].shaped) << ev[1].reason;
  expectOnAxis(out, a, 0.0);
  expectOnAxis(out, b, 0.0);
  EXPECT_LE(maxStep(out), 0.0501);
  EXPECT_LT(maxTurn(out), 0.05);
}

TEST(PathShaper, CollisionKeepsPlannerPath)
{
  const auto t = tunnel(0, 0, 0);
  const auto in = polyline({{-2.5, 0.05}, {2.5, 0.05}});
  const CollisionFree blocked = [](const Pose2 & q) {return std::abs(q.x) > 1.0;};
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, blocked, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_FALSE(ev[0].shaped);
  EXPECT_EQ(ev[0].reason.rfind(nav_behaviors::kCollisionReason, 0), 0u) << ev[0].reason;
  ASSERT_EQ(out.size(), in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    EXPECT_DOUBLE_EQ(out[i].x, in[i].x);
    EXPECT_DOUBLE_EQ(out[i].y, in[i].y);
  }
}

TEST(PathShaper, NoCrossingUnchanged)
{
  const auto in = polyline({{-2.5, 2.0}, {2.5, 2.0}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {tunnel(0, 0, 0)}, ShaperParams{}, kFree, &ev);
  EXPECT_TRUE(ev.empty());
  EXPECT_EQ(out.size(), in.size());
}

TEST(PathShaper, GoalInsideTunnelShapesUpToTheGoal)
{
  // a chained stretch (tunnel_5 + slope_19 in maps/sim) is longer than the tunnel it contains, and the
  // goal on the ramp lands inside the body.  Shape onto the axis and stop there: refusing left the
  // planner path unshaped and the robot turned inside the tunnel.
  Zone t = tunnel(0, 0, 0);
  t.depth = 1.8;   // tunnel_5 + slope_19
  auto in = polyline({{-2.225, 0.06}, {0.775, 0.0}});
  in.back().yaw = 0.0;
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  expectOnAxis(out, t, 0.0);
  double u, v;
  t.toLocal(out.back().x, out.back().y, u, v);
  EXPECT_NEAR(u, 0.775, 1e-6);   // the goal is kept
  EXPECT_NEAR(v, 0.0, 1e-9);
}

TEST(PathShaper, GoalJustInsideNearMouthIsShaped)
{
  const auto in = polyline({{-2.5, 0.0}, {-0.17, 0.0}});   // ends a few cm inside the body
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {tunnel(0, 0, 0)}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_NEAR(out.back().x, in.back().x, 1e-9);
  EXPECT_NEAR(out.back().y, in.back().y, 1e-9);
}

TEST(PathShaper, GoalJustAfterExitIsShaped)
{
  const auto in = polyline({{-2.5, 0.0}, {0.4, 0.0}});   // ends before the axis margin
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {tunnel(0, 0, 0)}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_NEAR(out.back().x, in.back().x, 1e-9);
  EXPECT_NEAR(out.back().y, in.back().y, 1e-9);
}

TEST(PathShaper, OneSidedNarrowingShiftsLaterally)
{
  // tunnel_2 in maps/sim: past the exit the south wall narrows to 0.30 m from the axis
  const auto t = tunnel(0, 0, 0);
  const auto free = walls({{-0.175, 0.175, 0.40, 5.0}, {-0.175, 0.175, -5.0, -0.40}, {0.6, 3.0, -5.0, -0.30}});
  const auto in = polyline({{-2.5, 0.0}, {2.5, 0.06}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, free, &ev);
  ASSERT_EQ(ev.size(), 1u);
  ASSERT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_GT(ev[0].v_body, 0.0);              // away from the wall
  EXPECT_LE(ev[0].v_body, 0.03 + 1e-9);
  for (const auto & q : out) {
    EXPECT_TRUE(free(q)) << q.x << " " << q.y;
    double u, v;
    t.toLocal(q.x, q.y, u, v);
    if (std::abs(u) <= 0.175) {
      EXPECT_NEAR(v, ev[0].v_body, 1e-9);
      EXPECT_NEAR(q.yaw, 0.0, 1e-9);
    }
  }
  EXPECT_LE(maxStep(out), 0.0501);
  EXPECT_LT(maxTurn(out), 0.05);
}

TEST(PathShaper, LeadInTurnsFromTheRobotHeading)
{
  // Gazebo tunnel_1: robot aligned 1.4 m before the mouth in a 0.8 m channel, the Smac path leaves it
  // diagonally; a lead-in turning from that tangent (-36 deg) swept the footprint into the channel wall
  const auto t = tunnel(0, 0, 0);
  const CollisionFree channel = [](const Pose2 & q) {
      return !(q.x > -1.6 && q.x < -0.175) || (std::abs(q.yaw) < 3.0 * M_PI / 180.0 && std::abs(q.y) < 0.12);
    };
  const auto in = polyline({{-1.42, 0.10}, {-1.2, -0.06}, {-0.9, 0.0}, {2.5, 0.0}});
  const double robot_yaw = 0.0;
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, channel, &ev, &robot_yaw);
  ASSERT_EQ(ev.size(), 1u);
  ASSERT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_NEAR(out.front().yaw, robot_yaw, 1e-9);
  for (const auto & q : out) {
    EXPECT_TRUE(channel(q)) << q.x << " " << q.y << " " << q.yaw * 180.0 / M_PI;
  }
}

TEST(PathShaper, NearlyAlignedRobotTurnsEarly)
{
  // Gazebo tunnel_1: robot at -8 deg at the channel entrance; the heading must be on the axis within
  // early_turn, not 2.5 m later where the 0.7 m channel leaves 5 cm per side
  const auto t = tunnel(0, 0, 0);
  const auto in = polyline({{-1.9, 0.02}, {2.5, 0.0}});   // closer than lead: the lead-in starts at the robot
  const double robot_yaw = -8.0 * M_PI / 180.0;
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &ev, &robot_yaw);
  ASSERT_EQ(ev.size(), 1u);
  ASSERT_TRUE(ev[0].shaped) << ev[0].reason;
  double s = 0.0;
  for (size_t i = 1; i < out.size(); ++i) {
    s += std::hypot(out[i].x - out[i - 1].x, out[i].y - out[i - 1].y);
    if (s > ShaperParams{}.early_turn + 0.05 && out[i].x < 0.675) {   // up to the end of the exit margin
      EXPECT_NEAR(out[i].yaw, 0.0, 1e-6) << "s " << s;
    }
  }
}

TEST(PathShaper, ResumeInsideTheBody)
{
  // replan after a stop half way through: the path starts in the body, the axis continues from the robot
  const auto t = tunnel(0, 0, 0);
  const auto in = polyline({{0.05, 0.01}, {2.5, 0.0}});
  const double robot_yaw = M_PI;   // passing backwards
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &ev, &robot_yaw);
  ASSERT_EQ(ev.size(), 1u);
  ASSERT_TRUE(ev[0].shaped) << ev[0].reason;
  for (const auto & q : out) {
    double u, v;
    t.toLocal(q.x, q.y, u, v);
    if (u < 0.175 + 0.5) {
      EXPECT_LE(std::abs(v), 0.01 + 1e-9);
      EXPECT_NEAR(std::abs(normAngle(q.yaw)), M_PI, 1e-9);
    }
  }
}

TEST(PathShaper, NarrowApproachTurnsEarly)
{
  // tunnel_7 in maps/sim: a 0.8 m corridor before the mouth only fits an aligned robot
  const auto t = tunnel(0, 0, 0);
  const CollisionFree free = [](const Pose2 & q) {
      return !(q.x > -1.25 && q.x < -0.175) || (std::abs(q.yaw) < 3.0 * M_PI / 180.0 && std::abs(q.y) < 0.25);
    };
  const auto in = polyline({{-2.5, -0.9}, {-1.2, 0.0}, {2.5, 0.0}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t}, ShaperParams{}, free, &ev);
  ASSERT_EQ(ev.size(), 1u);
  ASSERT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_NE(ev[0].turn, TurnMode::Full);
  for (const auto & q : out) {
    EXPECT_TRUE(free(q)) << q.x << " " << q.y << " " << q.yaw;
  }
  expectOnAxis(out, t, 0.0);
}

TEST(PathShaper, NeverMovesGoalToMakeItFit)
{
  for (const auto goal : {Pose2{0.0, 0.10, 0.0}, Pose2{0.0, 0.0, M_PI / 2}}) {
    auto in = polyline({{-2.5, 0.0}, {goal.x, goal.y}});
    in.back().yaw = goal.yaw;
    std::vector<ShapeEvent> events;
    const auto out = shapePath(in, {tunnel(0, 0, 0)}, ShaperParams{}, kFree, &events);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_FALSE(events[0].shaped);
    EXPECT_NEAR(out.back().x, goal.x, 1e-9);
    EXPECT_NEAR(out.back().y, goal.y, 1e-9);
    EXPECT_NEAR(out.back().yaw, goal.yaw, 1e-9);
  }
}

TEST(PathShaper, KeepsStartAndGoalInsideBody)
{
  Zone t = tunnel(0, 0, 0);
  t.depth = 2.0;
  const auto in = polyline({{-0.5, 0.01}, {0.5, 0.0}});
  std::vector<ShapeEvent> events;
  const auto out = shapePath(in, {t}, ShaperParams{}, kFree, &events);
  ASSERT_EQ(events.size(), 1u);
  ASSERT_TRUE(events[0].shaped) << events[0].reason;
  EXPECT_NEAR(out.front().x, in.front().x, 1e-9);
  EXPECT_NEAR(out.front().y, in.front().y, 1e-9);
  EXPECT_NEAR(out.back().x, in.back().x, 1e-9);
  EXPECT_NEAR(out.back().y, in.back().y, 1e-9);
}

TEST(PathShaper, RobotStartCannotBypassCollisionChecks)
{
  const auto t = tunnel(0, 0, 0);
  const auto in = polyline({{-1.9, 0.0}, {2.5, 0.0}});
  const double robot_yaw = 0.0;
  const CollisionFree occupied_start = [&](const Pose2 & q) {
      return std::hypot(q.x - in.front().x, q.y - in.front().y) > 0.02;
    };
  std::vector<ShapeEvent> events;
  shapePath(in, {t}, ShaperParams{}, occupied_start, &events, &robot_yaw);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_FALSE(events.front().shaped);
}

TEST(PathShaper, LeadOutJoinsNextDirectionAfterFullChassisClears)
{
  const auto zone=tunnel(0,0,0);
  auto input=polyline({{-3,0},{1,0},{3,2}});
  std::vector<ShapeEvent> events;
  const auto output=shapePath(input,{zone},ShaperParams{},kFree,&events);
  ASSERT_FALSE(events.empty());ASSERT_TRUE(events.front().shaped);
  expectOnAxis(output,zone,0.0);
  bool turned=false;
  for(const auto & p:output) {
    if(p.x>.8 && std::abs(p.yaw)>.2) {turned=true;}
  }
  EXPECT_TRUE(turned);
  double largest=0;
  for(size_t i=1;i<output.size();++i) {
    largest=std::max(largest,std::abs(normAngle(output[i].yaw-output[i-1].yaw)));
  }
  EXPECT_LT(largest,.25);  // no untrackable heading step at the next terrain approach
}

TEST(PathShaper, ExitSuffixIsReshapedAndChecksLateralVariants)
{
  const auto zone = tunnel(0., 0., 0.);
  const auto input = polyline({{.23, -.05}, {.7, 0.}, {2., .5}, {3., 1.}});
  const double yaw = 0.;
  const CollisionFree free = [](const Pose2 & q) {
      return !(q.x > .5 && q.x < 1. && q.y < .01);
    };
  std::vector<ShapeEvent> events;
  const auto output = shapePath(input, {zone}, ShaperParams{}, free, &events, &yaw);
  ASSERT_EQ(events.size(), 1u);
  ASSERT_TRUE(events[0].shaped) << events[0].reason;
  EXPECT_NE(events[0].zone_id.find("exit"), std::string::npos);
  EXPECT_NEAR(output.front().x, input.front().x, 1e-9);
  EXPECT_NEAR(output.front().y, input.front().y, 1e-9);
  EXPECT_NEAR(output.back().x, input.back().x, 1e-9);
  EXPECT_NEAR(output.back().y, input.back().y, 1e-9);
  for (const auto & q : output) {
    EXPECT_TRUE(free(q));
    if (q.x < .175 + .44 + .5) {EXPECT_NEAR(q.yaw, 0., 1e-6);}
  }
}
