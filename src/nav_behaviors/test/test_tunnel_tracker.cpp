#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/tunnel_tracker.hpp"

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

Tunnel tunnel0()
{
  Tunnel t;
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
  const Tunnel t = tunnel0();
  TunnelTracker tr;
  tr.setTunnels({t});
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

std::vector<Pose2> shaped(const std::vector<std::pair<double, double>> & pts)
{
  std::vector<ShapeEvent> ev;
  auto out = shapePath(polyline(pts), {tunnel0()}, ShaperParams{}, [](const Pose2 &) {return true;}, &ev);
  EXPECT_EQ(ev.size(), 1u);
  EXPECT_TRUE(!ev.empty() && ev[0].shaped);
  return out;
}

}  // namespace

TEST(TunnelTracker, PassesWithoutStopping)
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

TEST(TunnelTracker, DiagonalApproach)
{
  const auto path = shaped({{-2.6, -1.6}, {-1.0, 0.0}, {3.0, 0.0}});
  const auto run = simulate(path, {-2.6, -1.6, 0.0}, TrackerParams{});
  EXPECT_TRUE(run.passed);
  EXPECT_FALSE(run.gated);
  EXPECT_LT(run.max_lat_body, 0.01);
  EXPECT_LT(run.max_yaw_body, 2.0 * M_PI / 180.0);
  EXPECT_GT(run.min_speed_window, 0.3);
}

TEST(TunnelTracker, LocksTheCloserHeading)
{
  // driving -x with the robot facing +x: omni passes backwards instead of turning around
  const auto path = shaped({{3.0, 0.0}, {-3.0, 0.0}});
  const auto run = simulate(path, {3.0, 0.0, 0.2}, TrackerParams{});
  EXPECT_TRUE(run.passed);
  EXPECT_NEAR(run.lock, 0.0, 1e-9);
  EXPECT_LT(run.max_lat_body, 0.01);
  EXPECT_LT(run.max_yaw_body, 2.0 * M_PI / 180.0);
}

TEST(TunnelTracker, GateHoldsUntilAligned)
{
  TrackerParams prm;
  prm.max_wz = 0.25;   // too slow to align before the gate
  const auto path = shaped({{-3.0, 0.0}, {3.0, 0.0}});
  const auto run = simulate(path, {-3.0, 0.0, 1.4}, prm, 60.0);
  EXPECT_TRUE(run.gated);
  EXPECT_TRUE(run.passed);
  EXPECT_LT(run.max_yaw_body, prm.yaw_tol + 0.5 * M_PI / 180.0);
  EXPECT_LT(run.max_yaw_front, prm.yaw_tol + 0.5 * M_PI / 180.0);   // front enters aligned too
}

TEST(TunnelTracker, NoTunnelOnPath)
{
  TunnelTracker tr;
  tr.setTunnels({tunnel0()});
  tr.setPath(polyline({{-3.0, 2.0}, {3.0, 2.0}}));
  EXPECT_EQ(tr.crossingCount(), 0u);
  EXPECT_EQ(tr.compute({0.0, 2.0, 0.0}).weight, 0.0);
}
