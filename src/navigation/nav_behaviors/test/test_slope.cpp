// Slope policy: the uphill direction is shaped/tracked nose first with the planner's lateral line.
// Downhill remains with ordinary planning/MPPI; these unit tests do not validate slope physics.
#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "nav_behaviors/math_utils.hpp"
#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/zone_tracker.hpp"

using nav_behaviors::ChainParams;
using nav_behaviors::CollisionFree;
using nav_behaviors::groupZones;
using nav_behaviors::normAngle;
using nav_behaviors::Pose2;
using nav_behaviors::shapePath;
using nav_behaviors::ShapeEvent;
using nav_behaviors::ShaperParams;
using nav_behaviors::TrackerParams;
using nav_behaviors::Zone;
using nav_behaviors::ZoneProfile;
using nav_behaviors::ZoneTracker;

namespace
{

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

// a slope 1.2 m deep, 2.0 m wide, uphill along +x, centred at the origin
Zone slope(double cx = 0.0, double cy = 0.0, double yaw_deg = 0.0, const char * id = "slope")
{
  Zone z;
  z.id = id;
  z.profile = ZoneProfile::OpenDirectional;
  z.cx = cx;
  z.cy = cy;
  z.yaw = yaw_deg * M_PI / 180.0;
  z.depth = 1.2;
  z.width = 2.0;
  z.one_way_dir = 1;
  z.allow_flip = false;
  z.recentre = false;
  return z;
}

Zone tunnel(double cx, double cy, double yaw_deg, const char * id = "tunnel")
{
  Zone z;
  z.id = id;
  z.cx = cx;
  z.cy = cy;
  z.yaw = yaw_deg * M_PI / 180.0;
  z.depth = 0.35;
  z.width = 0.8;
  return z;
}

const CollisionFree kFree = [](const Pose2 &) {return true;};

// stand-in for MPPI outside the window: 1 m/s along the path, lateral pull, no heading control
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

// one crossing of zone `z` in direction `dir`, robot starting `approach` m before the near mouth
struct Run
{
  bool passed{false};
  bool gated{false};
  double max_lat_body{0.0}, max_yaw_body{0.0}, min_speed_body{1e9}, mean_speed_body{0.0};
  double lock{0.0};
  int samples{0};
};

Run drive(const Zone & z, int dir, double yaw0_deg, double lat, double approach, const TrackerParams & prm)
{
  const double yaw = dir > 0 ? z.yaw : normAngle(z.yaw + M_PI);
  const Pose2 a = z.toMap(-dir * approach, 0.0, 0.0), b = z.toMap(dir * (approach + 1.5), 0.0, 0.0);
  const auto in = polyline({{a.x, a.y}, {b.x, b.y}});
  const auto shaped = shapePath(in, {z}, ShaperParams{}, kFree, nullptr);
  ZoneTracker tr;
  tr.setZones({z});
  tr.setParams(prm);
  tr.setPath(shaped);
  Pose2 r = z.toMap(-dir * approach, lat, 0.0);
  r.yaw = normAngle(yaw + yaw0_deg * M_PI / 180.0);
  Run out;
  const double dt = 0.05;
  for (double time = 0.0; time < 40.0; time += dt) {
    const auto o = tr.compute(r);
    out.gated |= o.gated;
    double u, v;
    z.toLocal(r.x, r.y, u, v);
    if (std::abs(u) <= 0.5 * z.depth) {
      out.max_lat_body = std::max(out.max_lat_body, std::abs(v));
      out.max_yaw_body = std::max(out.max_yaw_body, std::abs(normAngle(r.yaw - o.lock)));
      const double sp = std::hypot(o.vx, o.vy);
      out.min_speed_body = std::min(out.min_speed_body, sp);
      out.mean_speed_body += sp;
      ++out.samples;
      out.lock = o.lock;
    }
    // The tracker deliberately returns zero outside its window. Exercise the actual
    // MPPI/tracker handoff, including the run-up, instead of parking outside the window.
    double vx, vy, wz;
    inner(shaped, r, vx, vy, wz);
    vx = (1.0 - o.weight) * vx + o.weight * o.vx;
    vy = (1.0 - o.weight) * vy + o.weight * o.vy;
    wz = (1.0 - o.weight) * wz + o.weight * o.wz;
    r.x += (std::cos(r.yaw) * vx - std::sin(r.yaw) * vy) * dt;
    r.y += (std::sin(r.yaw) * vx + std::cos(r.yaw) * vy) * dt;
    r.yaw = normAngle(r.yaw + wz * dt);
    if (dir * u - 0.5 * z.depth > 0.6) {
      out.passed = true;
      break;
    }
  }
  if (out.samples > 0) {
    out.mean_speed_body /= out.samples;
  }
  return out;
}

}  // namespace

TEST(Slope, UphillIsShaped)
{
  const auto z = slope();
  const auto in = polyline({{-3.0, 0.10}, {3.0, 0.10}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {z}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  ASSERT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_EQ(ev[0].profile, ZoneProfile::OpenDirectional);
  EXPECT_EQ(ev[0].direction, 1);
  EXPECT_NEAR(ev[0].heading, 0.0, 1e-9);   // uphill heading, no flip
  // the planner line is kept: a wide face is not re-centred onto its axis
  for (const auto & q : out) {
    double u, v;
    z.toLocal(q.x, q.y, u, v);
    if (std::abs(u) < 0.5 * z.depth) {
      EXPECT_NEAR(v, 0.10, 0.07) << "u " << u;
      EXPECT_NEAR(normAngle(q.yaw), 0.0, 1e-9);
    }
  }
}

TEST(Slope, DownhillIsLeftToThePlanner)
{
  const auto z = slope();
  const auto in = polyline({{3.0, 0.0}, {-3.0, 0.0}});   // driving -x, i.e. downhill
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {z}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_FALSE(ev[0].shaped);
  EXPECT_EQ(ev[0].direction, -1);
  EXPECT_NE(ev[0].reason.find("downhill"), std::string::npos) << ev[0].reason;
  ASSERT_EQ(out.size(), in.size());
}

TEST(Slope, LateralLineComesFromThisCrossingRatherThanAnEarlierAxisPlaneIntersection)
{
  const auto z = slope();
  const auto in = polyline({{-3., 3.}, {1., 3.}, {-3., .1}, {3., .1}});
  std::vector<ShapeEvent> events;
  const auto out = shapePath(in, {z}, ShaperParams{}, kFree, &events);
  ASSERT_EQ(events.size(), 1u);
  ASSERT_TRUE(events[0].shaped) << events[0].reason;
  EXPECT_NEAR(events[0].v_body, .1, 1e-6);
}

TEST(Slope, DiagonalUphillKeepsBothMouthOffsets)
{
  const auto z = slope();
  const auto in = polyline({{-3., -.5}, {3., .5}});
  std::vector<ShapeEvent> events;
  const auto out = shapePath(in, {z}, ShaperParams{}, kFree, &events);
  ASSERT_EQ(events.size(), 1u); ASSERT_TRUE(events[0].shaped);
  double near_v = 0., far_v = 0.;
  for (const auto & q : out) {
    if (std::abs(q.x + .6) < .001) {near_v = q.y;}
    if (std::abs(q.x - .6) < .001) {far_v = q.y;}
  }
  EXPECT_NEAR(near_v, -.1, .005);
  EXPECT_NEAR(far_v, .1, .005);
}

TEST(Slope, DownhillIsNotTracked)
{
  const auto z = slope();
  const auto path = polyline({{3.0, 0.0}, {-3.0, 0.0}});
  ZoneTracker tr;
  tr.setZones({z});
  tr.setPath(path);
  EXPECT_EQ(tr.crossingCount(), 0u);
  EXPECT_EQ(tr.compute({2.0, 0.0, M_PI}).weight, 0.0);
}

TEST(Slope, ChargesUpFasterThanATunnel)
{
  const auto s = slope();
  const auto t = tunnel(0.0, 0.0, 0.0);
  const TrackerParams prm;
  const auto rs = drive(s, 1, 3.0, 0.02, 3.0, prm);
  const auto rt = drive(t, 1, 3.0, 0.02, 3.0, prm);
  EXPECT_TRUE(rs.passed);
  EXPECT_TRUE(rt.passed);
  EXPECT_GT(rs.mean_speed_body, rt.mean_speed_body + 0.3) << "slope " << rs.mean_speed_body << " tunnel " <<
    rt.mean_speed_body;
  EXPECT_GT(rs.mean_speed_body, 1.0);
  EXPECT_LT(rs.max_yaw_body, 3.0 * M_PI / 180.0);
}

TEST(Slope, NoseFirstEvenWhenArrivingBackwards)
{
  // the robot arrives facing downhill: on a slope it must turn around (never charge backwards) and still
  // end up pointed uphill, unlike a tunnel which keeps the closer heading
  const auto s = slope();
  const auto r = drive(s, 1, 178.0, 0.02, 3.0, TrackerParams{});
  EXPECT_TRUE(r.passed);
  EXPECT_NEAR(std::abs(normAngle(r.lock)), 0.0, 1e-9);
  EXPECT_LT(r.max_yaw_body, 3.0 * M_PI / 180.0);
}

TEST(Slope, WideLateralGate)
{
  // 10 cm off the axis is fine on a 2 m wide face (a tunnel gates at 3 cm)
  const auto s = slope();
  const auto r = drive(s, 1, 2.0, 0.10, 3.0, TrackerParams{});
  EXPECT_TRUE(r.passed);
  EXPECT_FALSE(r.gated);
  EXPECT_LT(r.max_yaw_body, 3.0 * M_PI / 180.0);
}

TEST(Slope, AmbiguousDirectionIsLeftToThePlanner)
{
  auto z = slope();
  z.axis_reliable = false;
  const auto in = polyline({{-3.0, 0.0}, {3.0, 0.0}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {z}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_FALSE(ev[0].shaped);
  EXPECT_NE(ev[0].reason.find("ambiguous"), std::string::npos) << ev[0].reason;
}

TEST(Slope, ChainsWithTheTunnelBehindIt)
{
  // maps/sim tunnel_3 + slope_3: the slope mouth sits ~0.4 m past the tunnel mouth, axes parallel
  const auto t = tunnel(0.0, 0.0, 0.0, "tunnel_3");
  const auto s = slope(0.6, 0.02, 0.0, "slope_3");
  const auto groups = groupZones({t, s}, ChainParams{});
  ASSERT_EQ(groups.size(), 1u);
  EXPECT_EQ(groups[0].chain, 2);
  EXPECT_EQ(groups[0].id, "tunnel_3+slope_3");
  EXPECT_EQ(groups[0].profile, ZoneProfile::NarrowPassage);
  EXPECT_EQ(groups[0].one_way_dir, 0) << "a tunnel in the chain keeps it bidirectional";
  // the body spans both: from the tunnel's near mouth to the slope's far mouth
  EXPECT_NEAR(groups[0].depth, 0.35 / 2 + 0.6 + 1.2 / 2 + 0.175 - 0.175, 0.02);
}

TEST(Slope, ChainIsDrivenWithoutATurnInBetween)
{
  const auto t = tunnel(0.0, 0.0, 0.0, "tunnel_3");
  const auto s = slope(0.6, 0.0, 0.0, "slope_3");
  const auto in = polyline({{-3.0, 0.02}, {3.5, 0.02}});
  std::vector<ShapeEvent> ev;
  const auto out = shapePath(in, {t, s}, ShaperParams{}, kFree, &ev);
  ASSERT_EQ(ev.size(), 1u) << "one stretch, not two";
  ASSERT_TRUE(ev[0].shaped) << ev[0].reason;
  EXPECT_EQ(ev[0].chain, 2);
  // one locked heading from the tunnel mouth to the slope exit
  for (const auto & q : out) {
    double u, v;
    t.toLocal(q.x, q.y, u, v);
    if (u > -0.175 && u < 1.35) {
      EXPECT_NEAR(normAngle(q.yaw), 0.0, 1e-9) << "u " << u;
    }
  }
}

TEST(Slope, TwoSeparateSlopesAreTwoStretches)
{
  // far apart: not chained, each shaped on its own
  const auto a = slope(0.0, 0.0, 0.0, "slope_a");
  const auto b = slope(6.0, 0.0, 0.0, "slope_b");
  EXPECT_EQ(groupZones({a, b}, ChainParams{}).size(), 2u);
}

TEST(Slope, TunnelLeadOutCanAlsoBeTheNextRampApproach)
{
  const auto t = tunnel(0., 0., 0.);
  auto s = slope(2., 1., 90., "next_ramp"); s.depth = .6;
  ShaperParams params; params.lead_length = params.slope_lead_length = 2.5;
  const auto in = polyline({{-3., 0.}, {2., 0.}, {2., 3.}});
  std::vector<ShapeEvent> events;
  const auto out = shapePath(in, {t, s}, params, kFree, &events);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_TRUE(events[0].shaped) << events[0].reason;
  EXPECT_TRUE(events[1].shaped) << events[1].reason;
  EXPECT_EQ(events[1].direction, 1);
  for (size_t i = 1; i < out.size(); ++i) {
    EXPECT_LT(std::abs(normAngle(out[i].yaw - out[i - 1].yaw)), .25)
      << "adjacent stretches must share a continuous boundary heading";
  }
  for (const auto & q : out) {
    double u, v; s.toLocal(q.x, q.y, u, v);
    if (std::abs(u) < .3 && std::abs(v) < 1.) {
      EXPECT_NEAR(normAngle(q.yaw - s.yaw), 0., 1e-6);
    }
  }
}
