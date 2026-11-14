#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/zone_tracker.hpp"
#include "nav_behaviors/math_utils.hpp"

using namespace nav_behaviors;  // NOLINT

namespace
{

std::vector<Pose2> polyline(const std::vector<std::pair<double, double>> & pts, double step = 0.05)
{
  std::vector<Pose2> out;
  for (size_t k = 0; k + 1 < pts.size(); ++k) {
    const double dx = pts[k + 1].first - pts[k].first, dy = pts[k + 1].second - pts[k].second;
    const int n = std::max(1, static_cast<int>(std::round(std::hypot(dx, dy) / step)));
    for (int i = 0; i < n; ++i) {
      out.push_back({pts[k].first + dx * i / n, pts[k].second + dy * i / n, std::atan2(dy, dx)});
    }
  }
  out.push_back({pts.back().first, pts.back().second, out.back().yaw});
  return out;
}

Zone zone0()
{
  Zone t;
  t.id = "t";
  t.depth = 0.35;
  t.width = 0.8;
  return t;
}

// stand-in for MPPI: 1 m/s along the path, lateral pull, no heading control
void inner(const std::vector<Pose2> & p, const Pose2 & r, double & vx, double & vy, double & wz)
{
  size_t b = 0;
  double best = 1e9;
  for (size_t i = 0; i + 1 < p.size(); ++i) {
    const double d = std::hypot(p[i].x - r.x, p[i].y - r.y);
    if (d < best) {best = d; b = i;}
  }
  const double dx = p[b + 1].x - p[b].x, dy = p[b + 1].y - p[b].y, l = std::hypot(dx, dy);
  const double tx = dx / l, ty = dy / l, e = tx * (r.y - p[b].y) - ty * (r.x - p[b].x);
  const double wx = tx + 2.0 * e * ty, wy = ty - 2.0 * e * tx;
  vx = std::cos(r.yaw) * wx + std::sin(r.yaw) * wy;
  vy = -std::sin(r.yaw) * wx + std::cos(r.yaw) * wy;
  wz = 0.0;
}

struct Run
{
  double max_lat_body{0.0}, max_yaw_body{0.0}, max_yaw_front{0.0}, min_speed_window{1e9}, lock{0.0};
  bool gated{false}, passed{false}, entered_misaligned{false};
};

Run simulate(const std::vector<Pose2> & path, Pose2 r, const TrackerParams & prm, double t_max = 30.0)
{
  const Zone t = zone0();
  ZoneTracker tr;
  tr.setZones({t});
  tr.setParams(prm);
  tr.setPath(path);
  Run run;
  const double dt = 0.05;
  for (double time = 0.0; time < t_max; time += dt) {
    const auto o = tr.compute(r);
    double ix, iy, iw;
    inner(path, r, ix, iy, iw);
    const double w = o.weight;
    const double vx = (1 - w) * ix + w * o.vx, vy = (1 - w) * iy + w * o.vy, wz = (1 - w) * iw + w * o.wz;
    if (w >= 1.0) {
      run.min_speed_window = std::min(run.min_speed_window, std::hypot(vx, vy));
    }
    run.gated |= o.gated;
    double u, v;
    t.toLocal(r.x, r.y, u, v);
    const double u_front = u + 0.39 * (std::cos(r.yaw) >= 0 ? 1.0 : -1.0) * (path.back().x > path.front().x ? 1 : -1);
    if (std::abs(u_front) <= 0.175 && std::abs(u) > 0.175) {
      run.max_yaw_front = std::max(run.max_yaw_front, std::abs(normAngle(r.yaw - o.lock)));
    }
    if (std::abs(u) <= 0.175) {
      run.lock = o.lock;
      run.max_lat_body = std::max(run.max_lat_body, std::abs(v));
      run.max_yaw_body = std::max(run.max_yaw_body, std::abs(normAngle(r.yaw - o.lock)));
    }
    r.x += (std::cos(r.yaw) * vx - std::sin(r.yaw) * vy) * dt;
    r.y += (std::sin(r.yaw) * vx + std::cos(r.yaw) * vy) * dt;
    r.yaw = normAngle(r.yaw + wz * dt);
    if (std::hypot(r.x - path.back().x, r.y - path.back().y) < 0.1) {
      run.passed = true;
      break;
    }
  }
  return run;
}

std::vector<Pose2> shaped(const std::vector<std::pair<double, double>> & pts, const double * start_yaw = nullptr)
{
  std::vector<ShapeEvent> ev;
  auto out = shapePath(polyline(pts), {zone0()}, ShaperParams{}, [](const Pose2 &) {return true;}, &ev, start_yaw);
  EXPECT_EQ(ev.size(), 1u);
  EXPECT_TRUE(!ev.empty() && ev[0].shaped);
  return out;
}

}  // namespace

TEST(ZoneTracker, PassesWithoutStopping)
{
  const auto path = shaped({{-3.0, 0.05}, {3.0, 0.05}});
  const auto run = simulate(path, {-3.0, 0.10, 1.0}, TrackerParams{});   // 5 cm off, 57 deg off
  EXPECT_TRUE(run.passed);
  EXPECT_FALSE(run.gated);
  EXPECT_LT(run.max_lat_body, 0.01);
  EXPECT_LT(run.max_yaw_body, 2.0 * M_PI / 180.0);
  EXPECT_GT(run.min_speed_window, 0.3);
  EXPECT_NEAR(run.lock, 0.0, 1e-9);
}

TEST(ZoneTracker, DiagonalApproach)
{
  const auto path = shaped({{-2.6, -1.6}, {-1.0, 0.0}, {3.0, 0.0}});
  const auto run = simulate(path, {-2.6, -1.6, 0.0}, TrackerParams{});
  EXPECT_TRUE(run.passed);
  EXPECT_FALSE(run.gated);
  EXPECT_LT(run.max_lat_body, 0.01);
  EXPECT_LT(run.max_yaw_body, 2.0 * M_PI / 180.0);
  EXPECT_GT(run.min_speed_window, 0.3);
}

TEST(ZoneTracker, FollowsShaperSelectedBackwardsHeading)
{
  // driving -x with the robot facing +x: omni passes backwards instead of turning around
  const double start_yaw = .2;
  const auto path = shaped({{2.0, 0.0}, {-3.0, 0.0}}, &start_yaw);
  const auto run = simulate(path, {2.0, 0.0, 0.2}, TrackerParams{});
  EXPECT_TRUE(run.passed);
  EXPECT_NEAR(run.lock, 0.0, 1e-9);
  EXPECT_LT(run.max_lat_body, 0.01);
  EXPECT_LT(run.max_yaw_body, 2.0 * M_PI / 180.0);
}

TEST(ZoneTracker, SlowHeadingActuatorAlignsBeforeMouth)
{
  TrackerParams prm;
  prm.max_wz = 0.25;   // too slow to align before the gate
  const auto path = shaped({{-3.0, 0.0}, {3.0, 0.0}});
  const auto run = simulate(path, {-3.0, 0.0, 1.4}, prm, 60.0);
  // Rate limiting may finish alignment before the mouth gate is reached.
  EXPECT_TRUE(run.passed);
  EXPECT_LT(run.max_yaw_body, prm.yaw_tol + 0.5 * M_PI / 180.0);
  EXPECT_LT(run.max_yaw_front, prm.yaw_tol + 0.5 * M_PI / 180.0);   // front enters aligned too
}

TEST(ZoneTracker, NarrowApproachAlignsBeforeAdvancingWithLargeHeadingError)
{
  ZoneTracker tr; tr.setZones({zone0()});
  TrackerParams prm; prm.align_before = 1.5;
  tr.setParams(prm);
  tr.setPath(polyline({{-3., 0.}, {3., 0.}}));
  tr.compute({-1.5, 0., 2.4});
  const auto o = tr.compute({-1.3, 0., 2.4});
  ASSERT_GT(std::abs(o.e_yaw), M_PI / 2);
  EXPECT_NEAR(std::hypot(o.vx, o.vy), 0., 1e-9);
  EXPECT_LT(o.wz, 0.);  // keep correcting the heading while longitudinal motion waits
}

TEST(ZoneTracker, NoSpinOnDuplicatePoses)
{
  // planner paths carry duplicate / near-duplicate poses with differing yaw: the heading rate must not blow up
  auto path = shaped({{-3.0, 0.0}, {3.0, 0.0}});
  std::vector<Pose2> dup;
  for (size_t i = 0; i < path.size(); ++i) {
    dup.push_back(path[i]);
    if (i % 7 == 3) {
      Pose2 q = path[i];
      q.x += 1e-6;
      q.yaw += 0.3;
      dup.push_back(q);
    }
  }
  ZoneTracker tr;
  tr.setZones({zone0()});
  tr.setPath(dup);
  Pose2 r{-3.0, 0.0, 0.0};
  for (int k = 0; k < 400; ++k) {
    const auto o = tr.compute(r);
    EXPECT_LT(std::abs(o.wz), 0.2) << "x " << r.x;
    r.x += (std::cos(r.yaw) * o.vx - std::sin(r.yaw) * o.vy) * 0.05;
    r.y += (std::sin(r.yaw) * o.vx + std::cos(r.yaw) * o.vy) * 0.05;
    r.yaw = normAngle(r.yaw + o.wz * 0.05);
  }
}

TEST(ZoneTracker, BacksOffWhenWedgedInTheMouth)
{
  // leading edge at the mouth, 10 cm off the axis (corner on the mouth wall): back off, do not push
  const auto path = shaped({{-3.0, 0.0}, {3.0, 0.0}});
  ZoneTracker tr;
  tr.setZones({zone0()});
  tr.setParams(TrackerParams{});
  tr.setPath(path);
  tr.compute({-2.0, 0.0, 0.0});                     // window entered aligned
  const auto o = tr.compute({-0.175 - 0.39 + 0.01, 0.10, 0.0});
  EXPECT_TRUE(o.backing_off);
  EXPECT_LT(o.vx, 0.0);
  EXPECT_LT(o.vy, 0.0);                             // still correcting towards the axis
}

TEST(ZoneTracker, BacksOffWhenDraggedOffHeadingInTheChannel)
{
  // 1.2 m before the mouth a wall turns the robot against the commanded turn: back off until the heading
  // is corrected (hysteresis)
  const auto path = shaped({{-3.0, 0.0}, {3.0, 0.0}});
  ZoneTracker tr;
  tr.setZones({zone0()});
  tr.setParams(TrackerParams{});
  tr.setPath(path);
  tr.compute({-2.0, 0.0, 0.0});   // window entered aligned (s_rel -1.83)
  TrackerOutput o;
  double yaw = -0.10;
  for (int k = 0; k < 5; ++k) {   // commanded +wz, the robot keeps rotating negative
    o = tr.compute({-1.4, 0.0, yaw});
    yaw -= 0.05;
  }
  EXPECT_TRUE(o.backing_off);
  EXPECT_GT(o.wz, 0.0);
  o = tr.compute({-1.45, 0.0, -0.12});   // turned back to 7 deg: still above wedge_yaw / 2, keep backing
  EXPECT_TRUE(o.backing_off) << "e_yaw " << o.e_yaw * 180.0 / M_PI << " e_lat " << o.e_lat << " s " << o.s_rel;
  o = tr.compute({-1.5, 0.0, -0.02});
  EXPECT_FALSE(o.backing_off);
  EXPECT_GT(o.vx, 0.0);
}

TEST(ZoneTracker, NoTunnelOnPath)
{
  ZoneTracker tr;
  tr.setZones({zone0()});
  tr.setPath(polyline({{-3.0, 2.0}, {3.0, 2.0}}));
  EXPECT_EQ(tr.crossingCount(), 0u);
  EXPECT_EQ(tr.compute({0.0, 2.0, 0.0}).weight, 0.0);
}

TEST(ZoneTracker, SpeedLimitIncludesLateralCorrectionAndBackoff)
{
  ZoneTracker tr;
  tr.setZones({zone0()});
  tr.setPath(shaped({{-3.0, 0.0}, {3.0, 0.0}}));
  tr.compute({-2.0, 0.0, 0.0});
  for (const double cap : {0.1, 0.0}) {
    const auto o = tr.compute({-0.175 - 0.39 + 0.01, 0.10, 0.0}, cap);
    EXPECT_LE(std::hypot(o.vx, o.vy), cap + 1e-9);
  }
}

TEST(ZoneTracker, GoalInsideZoneRetainsOwnershipToTheEnd)
{
  Zone z = zone0();
  z.depth = 2.0;
  ZoneTracker tr;
  tr.setZones({z});
  tr.setPath(polyline({{-3.0, 0.0}, {0.2, 0.0}}));
  ASSERT_EQ(tr.crossingCount(), 1u);
  const auto o = tr.compute({0.1, 0.0, 0.0});
  EXPECT_EQ(o.phase, ZonePhase::Traverse);
  EXPECT_DOUBLE_EQ(o.weight, 1.0);
  EXPECT_TRUE(o.zone_mode);
  const auto end = tr.compute({0.2, 0.0, 0.0});
  EXPECT_EQ(end.phase, ZonePhase::Traverse);
  EXPECT_DOUBLE_EQ(end.weight, 1.0);
  EXPECT_NEAR(end.vx, 0.0, 1e-9);
}

TEST(ZoneTracker, PlanWhollyInsideZoneAndReversePlanAreTracked)
{
  Zone z = zone0();
  z.depth = 2.0;
  for (int dir : {-1, 1}) {
    ZoneTracker tr;
    tr.setZones({z});
    auto path = polyline({{-dir * 0.6, 0.0}, {dir * 0.6, 0.0}});
    for (auto & q : path) {q.yaw = 0.;}  // both directions use the validated physical heading
    tr.setPath(path);
    ASSERT_EQ(tr.crossingCount(), 1u);
    const auto o = tr.compute({0.0, 0.0, 0.0});
    EXPECT_EQ(o.phase, ZonePhase::Traverse);
    EXPECT_DOUBLE_EQ(o.weight, 1.0);
    EXPECT_GT(dir * o.vx, 0.0);
  }
}

TEST(ZoneTracker, CollisionSamplesCoverSparsePlanAndPhysicalReverseHeading)
{
  ZoneTracker tr;
  tr.setZones({zone0()});
  tr.setPath({{-3.0, 0.0, M_PI}, {-0.1, 0.0, M_PI}, {3.0, 0.0, M_PI}});
  tr.compute({-0.1, 0.0, M_PI});
  const auto samples = tr.ahead(0.4);
  ASSERT_GT(samples.size(), 2u);
  EXPECT_NEAR(samples.front().x, -0.1, 1e-9);
  EXPECT_NEAR(samples.back().x, 0.3, 1e-9);
  for (size_t i = 1; i < samples.size(); ++i) {
    EXPECT_LE(std::hypot(samples[i].x - samples[i - 1].x, samples[i].y - samples[i - 1].y), 0.025001);
    EXPECT_NEAR(std::abs(samples[i].yaw), M_PI, 1e-9);
  }
  const auto behind = tr.ahead(-0.4);
  EXPECT_NEAR(behind.back().x, -0.5, 1e-9);
}


TEST(ZoneTracker, ExitSuffixReplanKeepsControlAndHeadingUntilRearClears)
{
  ZoneTracker tr; tr.setZones({zone0()});
  tr.setPath(polyline({{-2, 0}, {2, 0}}));
  tr.compute({-.5, 0, 0});
  const auto before = tr.compute({.4, 0, 0});
  ASSERT_EQ(before.phase, ZonePhase::Exit);
  tr.setPath(polyline({{.4, 0}, {1.5, .3}, {2, .8}}));
  const auto after = tr.compute({.4, 0, 0});
  EXPECT_EQ(after.phase, ZonePhase::Exit);
  EXPECT_DOUBLE_EQ(after.weight, 1.0);
  EXPECT_NEAR(after.yaw_ref, before.yaw_ref, 1e-6);
  EXPECT_NEAR(after.wz, 0.0, 1e-6);
  EXPECT_GT(std::hypot(after.vx, after.vy), 0.1);
}

TEST(ZoneTracker, ColdExitSuffixAndReverseSuffixRemainTracked)
{
  for (int dir : {-1, 1}) {
    ZoneTracker tr; tr.setZones({zone0()});
    auto path = polyline({{dir * .4, 0}, {dir * 2., 0}});
    for (auto & q : path) {q.yaw = 0.;}  // collision-checked backwards heading for dir -1
    tr.setPath(path);
    const auto o = tr.compute({dir * .4, 0, 0});
    EXPECT_EQ(o.phase, ZonePhase::Exit);
    EXPECT_DOUBLE_EQ(o.weight, 1.0);
    EXPECT_NEAR(o.yaw_ref, 0.0, 1e-6);
    EXPECT_GT(dir * o.vx, 0.1);
  }
}

TEST(ZoneTracker, ExitCollisionSamplesUseTheSameHeadingAsControl)
{
  for (int dir : {-1, 1}) {
    ZoneTracker tr; tr.setZones({zone0()});
    auto path = polyline({{dir * .4, 0}, {dir * 1.5, .3}, {dir * 2., .8}});
    if (dir < 0) {
      for (auto & q : path) {q.yaw = normAngle(q.yaw + M_PI);}
    }
    tr.setPath(path);
    const auto o = tr.compute({dir * .4, 0, 0});
    ASSERT_EQ(o.phase, ZonePhase::Exit);
    const auto samples = tr.ahead(.8);
    ASSERT_GT(samples.size(), 2u);
    EXPECT_NEAR(normAngle(samples.front().yaw - o.yaw_ref), 0.0, 1e-6);
    for (const auto & q : samples) {
      if (dir * q.x - .175 <= .44) {
        EXPECT_NEAR(q.yaw, 0.0, 1e-6) << "rear has not cleared the mouth";
      }
    }
    EXPECT_GT(std::abs(samples.back().yaw), .01) << "heading must join the suffix after clearance";
    EXPECT_LT(std::abs(samples.back().yaw), .1) << "suffix must not turn faster than the original lead-out";
  }
}

TEST(ZoneTracker, SuffixPastExitWindowAndSidewaysPathDoNotLatch)
{
  ZoneTracker tr; tr.setZones({zone0()});
  tr.setPath(polyline({{1.2, 0}, {2., 0}}));
  EXPECT_DOUBLE_EQ(tr.compute({1.2, 0, 0}).weight, 0.0);
  tr.setPath(polyline({{.4, 0}, {.4, 1.5}}));
  EXPECT_DOUBLE_EQ(tr.compute({.4, 0, 0}).weight, 0.0);
}

TEST(ZoneTracker, AdjacentRampDoesNotAcquireAnUntraversedTunnelExit)
{
  ZoneTracker tr; tr.setZones({zone0()});
  // Still inside window_after, but the complete chassis is already clear.
  // A new route heading diagonally away is not proof of a preceding crossing.
  tr.setPath(polyline({{.9, .1}, {1.5, .7}, {1.5, 1.5}}));
  EXPECT_DOUBLE_EQ(tr.compute({.9, .1, M_PI / 2}).weight, 0.0);
  tr.setPath(polyline({{.4, .1}, {1., .7}, {1., 1.5}}));
  EXPECT_DOUBLE_EQ(tr.compute({.4, .1, M_PI / 4}).weight, 0.0);
  // A real crossing keeps its exit blend at the same physical location.
  tr.setPath(polyline({{-2, 0}, {2, 0}}));
  tr.compute({0, 0, 0});
  tr.setPath(polyline({{.9, .1}, {1.5, .7}, {1.5, 1.5}}));
  EXPECT_GT(tr.compute({.9, .1, 0}).weight, 0.0);
}
