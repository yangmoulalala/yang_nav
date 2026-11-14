// Kinematic closed-loop test (no ROS): exported map tunnels in both directions, omni kinematics
// with execution and pose noise.
//
// The path is the shaped planner path (shapePath), the controller is the ZoneAwareController mix
// (1 - w) * inner + w * tracker with a stand-in inner controller, and the state machine phases of
// zone_state.hpp are checked in order (approach -> align -> traverse -> exit -> none, never back).
//
// A replan inside the body (setPath with the same shaped path, as the shaper hands it back) must keep
// the heading already locked: that is what the BT pause protects in Gazebo.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/zone_tracker.hpp"
#include "nav_behaviors/zone_state.hpp"
#include "nav_behaviors/zones_yaml.hpp"
#include "nav_behaviors/math_utils.hpp"

using namespace nav_behaviors;  // NOLINT

namespace
{

// the zones.yaml the stack loads (maps/sim, terrain_dir of run_nav.sh), same reader and min_cells as the
// plugins: 6 tunnels, tunnel_6 is a 2-cell label fragment and dropped
std::vector<Zone> simTunnels()
{
  return loadTunnels(SIM_ZONES_FILE, 50, nullptr);
}

const Zone & byId(const std::vector<Zone> & ts, const std::string & id)
{
  for (const auto & t : ts) {
    if (t.id == id) {
      return t;
    }
  }
  throw std::runtime_error("no tunnel " + id);
}

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

// stand-in for MPPI: 1 m/s along the path, lateral pull, no heading control
void inner(const std::vector<Pose2> & p, const Pose2 & r, double & vx, double & vy, double & wz)
{
  size_t b = 0;
  double best = 1e9;
  for (size_t i = 0; i + 1 < p.size(); ++i) {
    const double d = std::hypot(p[i].x - r.x, p[i].y - r.y);
    if (d < best) {
      best = d;
      b = i;
    }
  }
  const double dx = p[b + 1].x - p[b].x, dy = p[b + 1].y - p[b].y, l = std::hypot(dx, dy);
  const double tx = dx / l, ty = dy / l, e = tx * (r.y - p[b].y) - ty * (r.x - p[b].x);
  const double wx = tx + 2.0 * e * ty, wy = ty - 2.0 * e * tx;
  vx = std::cos(r.yaw) * wx + std::sin(r.yaw) * wy;
  vy = -std::sin(r.yaw) * wx + std::cos(r.yaw) * wy;
  wz = 0.0;
}

struct Case
{
  double approach{3.0};      // start distance before the near mouth, m
  double lat{0.02};          // start offset from the zone axis, m
  double yaw_deg{5.0};       // start heading offset from the passing direction, deg
  bool replan{false};        // re-issue the shaped path once the robot is inside the body
};

struct Result
{
  bool passed{false};
  bool gated{false};
  bool replanned{false};
  bool phase_order_ok{true};
  bool saw_align{false}, saw_traverse{false}, saw_exit{false};
  double max_lat_body{0.0}, max_yaw_body{0.0}, min_speed_body{1e9};
  std::vector<ZonePhase> phases;
};

// one crossing of tunnel `t` in direction `dir`
Result drive(const Zone & t, int dir, unsigned seed, const TrackerParams & prm, const Case & c)
{
  const double yaw = dir > 0 ? t.yaw : normAngle(t.yaw + M_PI);
  const Pose2 a = t.toMap(-dir * c.approach, 0.0, 0.0);
  const Pose2 b = t.toMap(dir * (c.approach + 1.5), 0.0, 0.0);
  const auto in = polyline({{a.x, a.y}, {b.x, b.y}});
  std::vector<ShapeEvent> ev;
  const auto shaped = shapePath(in, {t}, ShaperParams{}, [](const Pose2 &) {return true;}, &ev);
  EXPECT_EQ(ev.size(), 1u) << t.id;
  EXPECT_TRUE(!ev.empty() && ev[0].shaped) << t.id << " " << (ev.empty() ? "" : ev[0].reason);

  ZoneTracker tr;
  tr.setZones({t});
  tr.setParams(prm);
  tr.setPath(shaped);

  std::mt19937 rng(seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  const double v_scale_sigma = 0.03, yaw_drift_sigma = 0.5 * M_PI / 180.0;
  const double pose_sigma = 0.005, pose_yaw_sigma = 0.2 * M_PI / 180.0;

  Pose2 truth = t.toMap(-dir * c.approach, c.lat, 0.0);
  truth.yaw = normAngle(yaw + c.yaw_deg * M_PI / 180.0);
  Result r;
  const double dt = 0.05;
  for (double time = 0.0; time < 60.0; time += dt) {
    const Pose2 meas{truth.x + pose_sigma * gauss(rng), truth.y + pose_sigma * gauss(rng),
      normAngle(truth.yaw + pose_yaw_sigma * gauss(rng))};
    const auto o = tr.compute(meas);
    const ZonePhase ph = o.weight <= 0.0 ? ZonePhase::None : o.phase;
    if (r.phases.empty() || r.phases.back() != ph) {
      r.phases.push_back(ph);
    }
    r.saw_align |= ph == ZonePhase::Align;
    r.saw_traverse |= ph == ZonePhase::Traverse;
    r.saw_exit |= ph == ZonePhase::Exit;

    double ix, iy, iw;
    inner(shaped, meas, ix, iy, iw);
    const double w = o.weight;
    double vx = (1 - w) * ix + w * o.vx, vy = (1 - w) * iy + w * o.vy, wz = (1 - w) * iw + w * o.wz;
    // execution noise
    const double sc = 1.0 + v_scale_sigma * gauss(rng);
    vx *= sc;
    vy *= sc;
    wz += yaw_drift_sigma * gauss(rng);
    r.gated |= o.gated;

    double u, v;
    t.toLocal(truth.x, truth.y, u, v);
    if (std::abs(u) <= 0.5 * t.depth) {
      r.max_lat_body = std::max(r.max_lat_body, std::abs(v));
      r.max_yaw_body = std::max(r.max_yaw_body, std::abs(normAngle(truth.yaw - o.lock)));
      r.min_speed_body = std::min(r.min_speed_body, std::hypot(vx, vy));
    }
    // a replan inside the body: the shaped path comes back, the heading must stay locked
    if (c.replan && !r.replanned && dir * u > 0.1) {
      tr.setPath(shaped);
      r.replanned = true;
    }

    truth.x += (std::cos(truth.yaw) * vx - std::sin(truth.yaw) * vy) * dt;
    truth.y += (std::sin(truth.yaw) * vx + std::cos(truth.yaw) * vy) * dt;
    truth.yaw = normAngle(truth.yaw + wz * dt);
    if (std::hypot(truth.x - shaped.back().x, truth.y - shaped.back().y) < 0.15) {
      r.passed = true;
      break;
    }
  }
  // approach -> align -> traverse -> exit, never back; `none` only at the start and after the exit
  int last = 0;
  for (auto p : r.phases) {
    const int i = static_cast<int>(p);
    if (p == ZonePhase::None) {
      continue;
    }
    if (i < last) {
      r.phase_order_ok = false;
    }
    last = std::max(last, i);
  }
  return r;
}

}  // namespace

TEST(TunnelDrive, AllTunnelsBothDirections)
{
  const auto tunnels = simTunnels();
  ASSERT_GE(tunnels.size(), 6u);
  int cases = 0, passed = 0;
  for (const auto & t : tunnels) {
    for (int dir : {1, -1}) {
      SCOPED_TRACE(t.id + (dir > 0 ? " fwd" : " rev"));
      // facing the travel direction, backwards (omni: no turnaround) and 70 deg off
      for (double yaw0 : {5.0, 178.0, 70.0}) {
        for (unsigned seed : {1u, 7u, 21u}) {
          Case c;
          c.yaw_deg = yaw0;
          const auto r = drive(t, dir, seed, TrackerParams{}, c);
          ++cases;
          EXPECT_TRUE(r.passed) << "yaw0 " << yaw0;
          EXPECT_TRUE(r.phase_order_ok);
          EXPECT_TRUE(r.saw_align);
          EXPECT_TRUE(r.saw_traverse);
          EXPECT_TRUE(r.saw_exit);
          EXPECT_LT(r.max_lat_body, 0.03) << "lateral error in the body, m, yaw0 " << yaw0;
          EXPECT_LT(r.max_yaw_body, 3.0 * M_PI / 180.0) << "heading error in the body, yaw0 " << yaw0;
          EXPECT_GT(r.min_speed_body, 0.3) << "stopped inside the body, m/s";
          passed += r.passed ? 1 : 0;
        }
      }
    }
  }
  EXPECT_EQ(passed, cases);
}

TEST(TunnelDrive, ResumeFromInsideTheBody)
{
  // stopped half way through (gate / obstacle / recovery): the replanned path starts in the body, the
  // shaper keeps the robot's end on the axis and the tracker finishes the crossing
  for (const auto & t : simTunnels()) {
    for (int dir : {1, -1}) {
      SCOPED_TRACE(t.id + (dir > 0 ? " fwd" : " rev"));
      const double yaw = dir > 0 ? t.yaw : normAngle(t.yaw + M_PI);
      const Pose2 a = t.toMap(dir * 0.05, 0.01, 0.0), b = t.toMap(dir * 3.0, 0.0, 0.0);
      auto in = polyline({{a.x, a.y}, {b.x, b.y}});
      in.front().yaw = normAngle(yaw + M_PI + 0.01);   // robot pose: facing backwards
      std::vector<ShapeEvent> ev;
      const auto shaped = shapePath(in, {t}, ShaperParams{}, [](const Pose2 &) {return true;}, &ev);
      ASSERT_EQ(ev.size(), 1u);
      EXPECT_TRUE(ev[0].shaped) << ev[0].reason;
      ZoneTracker tr;
      tr.setZones({t});
      tr.setPath(shaped);
      ASSERT_EQ(tr.crossingCount(), 1u);
      Pose2 r{a.x, a.y, in.front().yaw};
      double max_yaw = 0.0;
      bool passed = false;
      for (double time = 0.0; time < 20.0; time += 0.05) {
        const auto o = tr.compute(r);
        ASSERT_GT(o.weight, 0.0);
        if (o.in_body) {
          max_yaw = std::max(max_yaw, std::abs(normAngle(r.yaw - o.lock)));
        }
        r.x += (std::cos(r.yaw) * o.vx - std::sin(r.yaw) * o.vy) * 0.05;
        r.y += (std::sin(r.yaw) * o.vx + std::cos(r.yaw) * o.vy) * 0.05;
        r.yaw = normAngle(r.yaw + o.wz * 0.05);
        double u, v;
        t.toLocal(r.x, r.y, u, v);
        if (dir * u > 0.5 * t.depth + 0.4) {
          passed = true;
          break;
        }
      }
      EXPECT_TRUE(passed);
      EXPECT_LT(max_yaw, 3.0 * M_PI / 180.0);
    }
  }
}

TEST(TunnelDrive, ReplanKeepsTheLock)
{
  // a replan inside the body (shaped path handed back again) must not unlock the heading
  const auto t = simTunnels().front();
  Case c;
  c.replan = true;
  const auto r = drive(t, 1, 3u, TrackerParams{}, c);
  EXPECT_TRUE(r.passed);
  EXPECT_TRUE(r.replanned);
  EXPECT_LT(r.max_yaw_body, 3.0 * M_PI / 180.0);
  EXPECT_LT(r.max_lat_body, 0.03);
}

TEST(TunnelDrive, MisalignedEntryAlignsBeforeBody)
{
  // an obviously misaligned approach is slowed / stopped before the mouth, never inside the body
  TrackerParams prm;
  prm.max_wz = 0.25;   // too slow to correct the heading in time
  Case c;
  c.approach = 1.2;
  c.yaw_deg = 60.0;
  const auto t = simTunnels().front();
  const auto r = drive(t, 1, 5u, prm, c);
  // Early heading-rate limiting can align before the mouth gate activates.
  EXPECT_TRUE(r.passed);
  EXPECT_LT(r.max_yaw_body, prm.yaw_tol + 0.5 * M_PI / 180.0);
  EXPECT_LT(r.max_lat_body, 0.03);
}

TEST(TunnelDrive, DiagonalApproachStaysInsideTheOpening)
{
  // start off the axis and off the heading: the lead-in + tracker must bring it on the axis
  Case c;
  c.approach = 2.0;
  c.lat = 0.10;
  c.yaw_deg = 25.0;
  const auto r = drive(byId(simTunnels(), "tunnel_3"), -1, 11u, TrackerParams{}, c);
  EXPECT_TRUE(r.passed);
  EXPECT_TRUE(r.phase_order_ok);
  EXPECT_LT(r.max_lat_body, 0.03);
  EXPECT_LT(r.max_yaw_body, 3.0 * M_PI / 180.0);
}
