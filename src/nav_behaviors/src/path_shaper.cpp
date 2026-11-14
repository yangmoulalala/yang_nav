#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>
#include <vector>

namespace nav_behaviors
{

const char * const kCollisionReason = "footprint collision";

const char * zoneProfileName(ZoneProfile profile)
{
  return profile == ZoneProfile::OpenDirectional ? "open" : "narrow";
}

const char * zoneModeName(ZoneProfile profile)
{
  return profile == ZoneProfile::OpenDirectional ? "ramp" : "tunnel";
}

const char * turnName(TurnMode m)
{
  switch (m) {
    case TurnMode::Full: return "full lead";
    case TurnMode::Half: return "first half";
    default: return "locked";
  }
}

void Zone::toLocal(double x, double y, double & u, double & v) const
{
  const double dx = x - cx, dy = y - cy, c = std::cos(yaw), s = std::sin(yaw);
  u = c * dx + s * dy;
  v = -s * dx + c * dy;
}

Pose2 Zone::toMap(double u, double v, double yaw_rel) const
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {cx + c * u - s * v, cy + s * u + c * v, normAngle(yaw + yaw_rel)};
}

namespace
{

struct Vec
{
  double x, y;
};

std::vector<double> arcLength(const std::vector<Pose2> & p)
{
  std::vector<double> s(p.size(), 0.0);
  for (size_t i = 1; i < p.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
  }
  return s;
}

// path direction at i averaged over +- win of arc length (Smac grid paths zig-zag cell by cell)
Vec smoothTangent(const std::vector<Pose2> & p, const std::vector<double> & s, size_t i, double win)
{
  size_t a = i, b = i;
  while (a > 0 && s[i] - s[a] < win) {
    --a;
  }
  while (b + 1 < p.size() && s[b] - s[i] < win) {
    ++b;
  }
  const double dx = p[b].x - p[a].x, dy = p[b].y - p[a].y, n = std::hypot(dx, dy);
  if (n < 1e-9) {
    return {std::cos(p[i].yaw), std::sin(p[i].yaw)};
  }
  return {dx / n, dy / n};
}

// polyline d with arc lengths s -> points every <= spacing, first and last point exact
std::vector<Vec> resample(const std::vector<Vec> & d, const std::vector<double> & s, double spacing)
{
  if (d.size() < 2 || s.back() < 1e-9) {
    return {d.front()};
  }
  const double total = s.back();
  const int n = std::max(1, static_cast<int>(std::ceil(total / spacing - 1e-9)));
  std::vector<Vec> out;
  out.reserve(n + 1);
  size_t j = 0;
  for (int i = 0; i <= n; ++i) {
    const double q = total * i / n;
    while (j + 2 < d.size() && s[j + 1] < q) {
      ++j;
    }
    const double seg = s[j + 1] - s[j];
    const double f = seg > 1e-12 ? std::clamp((q - s[j]) / seg, 0.0, 1.0) : 0.0;
    out.push_back({d[j].x + f * (d[j + 1].x - d[j].x), d[j].y + f * (d[j + 1].y - d[j].y)});
  }
  out.front() = d.front();
  out.back() = d.back();
  return out;
}

// cubic Bezier p0 -> p3, leaving p0 along t0 and arriving at p3 along t3 (unit vectors)
std::vector<Vec> bezier(Vec p0, Vec t0, Vec p3, Vec t3, double spacing)
{
  const double L = std::hypot(p3.x - p0.x, p3.y - p0.y);
  if (L < 1e-9) {
    return {p0};
  }
  const double k = L / 3.0;
  const Vec p1{p0.x + t0.x * k, p0.y + t0.y * k}, p2{p3.x - t3.x * k, p3.y - t3.y * k};
  const int N = std::max(20, static_cast<int>(L / spacing * 8));
  std::vector<Vec> dense(N + 1);
  std::vector<double> s(N + 1, 0.0);
  for (int i = 0; i <= N; ++i) {
    const double t = static_cast<double>(i) / N, a = 1.0 - t;
    const double b0 = a * a * a, b1 = 3 * a * a * t, b2 = 3 * a * t * t, b3 = t * t * t;
    dense[i] = {b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x, b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y};
    if (i > 0) {
      s[i] = s[i - 1] + std::hypot(dense[i].x - dense[i - 1].x, dense[i].y - dense[i - 1].y);
    }
  }
  return resample(dense, s, spacing);
}

double smoothstep(double w)
{
  w = std::clamp(w, 0.0, 1.0);
  return w * w * (3.0 - 2.0 * w);
}

double blend(double a, double b, double w)
{
  return normAngle(a + normAngle(b - a) * smoothstep(w));
}

// along the axis in travel coordinate s (u = dir * s) from s0 to s1, lateral offset va -> vb (smoothstep,
// zero slope at both ends so the joints stay G1)
std::vector<Vec> ramp(const Zone & t, int dir, double s0, double s1, double va, double vb, double spacing)
{
  const int n = std::max(1, static_cast<int>(std::ceil((s1 - s0) / spacing - 1e-9)));
  std::vector<Vec> out;
  out.reserve(n + 1);
  for (int i = 0; i <= n; ++i) {
    const double w = static_cast<double>(i) / n;
    const Pose2 q = t.toMap(dir * (s0 + (s1 - s0) * w), va + (vb - va) * smoothstep(w), 0.0);
    out.push_back({q.x, q.y});
  }
  return out;
}

// headings ha -> hb, the turn finished after `turn_end` of the arc length (0 = hb from the first pose)
std::vector<Pose2> headed(const std::vector<Vec> & pts, double ha, double hb, double turn_end)
{
  std::vector<double> sl(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    sl[i] = sl[i - 1] + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  std::vector<Pose2> out;
  out.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    const double span = sl.back() * turn_end;
    const double w = span > 1e-9 ? sl[i] / span : 1.0;
    out.push_back({pts[i].x, pts[i].y, blend(ha, hb, w)});
  }
  return out;
}

// v0, v0 + step, v0 - step, v0 + 2 step, ... up to max (v0 first: the path's own line)
std::vector<double> offsetsAround(double v0, double max, double step)
{
  std::vector<double> out{v0};
  if (step <= 0.0) {
    return out;
  }
  for (double v = step; v <= max + 1e-9; v += step) {
    out.push_back(v0 + v);
    out.push_back(v0 - v);
  }
  return out;
}

// lateral offset (zone frame) where the path crosses travel coordinate s = s_target, NaN when it does not
double lateralAt(const std::vector<Pose2> & p, const Zone & z, int dir, double s_target,
  size_t first, size_t last)
{
  bool have = false;
  double prev_s = 0.0, prev_v = 0.0;
  for (size_t i = first; i <= last; ++i) {
    double u, v;
    z.toLocal(p[i].x, p[i].y, u, v);
    const double s = dir * u;
    if (have && (prev_s - s_target) * (s - s_target) <= 0.0 && std::abs(s - prev_s) > 1e-9) {
      const double f = (s_target - prev_s) / (s - prev_s);
      return prev_v + f * (v - prev_v);
    }
    prev_s = s;
    prev_v = v;
    have = true;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

struct Crossing
{
  size_t first{0}, last{0};   // samples inside the zone body
  int dir{0};
  bool through{false};
};

bool findCrossing(const std::vector<Pose2> & p, const Zone & t, size_t from, Crossing & c)
{
  const size_t n = p.size();
  std::vector<double> u(n, 0.0), v(n, 0.0);
  for (size_t i = from; i < n; ++i) {
    t.toLocal(p[i].x, p[i].y, u[i], v[i]);
  }
  auto inside = [&](size_t i) {return std::abs(u[i]) <= 0.5 * t.depth && std::abs(v[i]) < 0.5 * t.width;};
  size_t f = from;
  while (f < n && !inside(f)) {
    ++f;
  }
  if (f >= n) {
    return false;
  }
  size_t l = f;
  while (l + 1 < n && inside(l + 1)) {
    ++l;
  }
  c = Crossing{f, l, 0, false};
  if (l + 1 >= n) {
    // ends inside the zone body (a goal on the ramp behind a tunnel): the entry side gives the direction
    if (f > from) {
      c.dir = u[f - 1] < 0.0 ? 1 : -1;
      c.through = true;
    } else if (from == 0 && std::abs(u[l] - u[f]) > 1e-6) {
      c.dir = u[l] > u[f] ? 1 : -1;
      c.through = true;
    }
    return true;
  }
  if (f == from) {
    if (from == 0) {
      // starts inside the body (replan after a stop in the zone): the exit side gives the direction
      c.dir = u[l + 1] > 0 ? 1 : -1;
      c.through = true;
    }
    return true;
  }
  const double ub = u[f - 1], ua = u[l + 1];
  c.dir = (ub < 0 && ua > 0) ? 1 : (ub > 0 && ua < 0) ? -1 : 0;
  c.through = c.dir != 0;
  return true;
}

bool shapeCrossing(
  const std::vector<Pose2> & p, const Zone & t, const Crossing & c, size_t from, size_t stop,
  double lead, const ShaperParams & prm, const CollisionFree & collision_free, const double * start_yaw,
  std::vector<Pose2> & out, size_t & next, ShapeEvent & ev, std::string & reason)
{
  const size_t n = p.size();
  const int dir = c.dir;
  const bool open = t.profile == ZoneProfile::OpenDirectional;
  const double margin = open ? prm.slope_axis_margin : prm.axis_margin;
  const double max_body = open ? prm.slope_max_body_offset : prm.max_body_offset;
  const double hd = 0.5 * t.depth, ext = hd + margin;
  const auto s = arcLength(p);
  auto du = [&](size_t i) {double u, v; t.toLocal(p[i].x, p[i].y, u, v); return dir * u;};

  size_t k0 = n, k1 = n;
  for (size_t i = c.first; i-- > from; ) {
    if (du(i) <= -ext) {k0 = i; break;}
  }
  // the robot stands on the axis stretch already (replan after a gate stop or a stop in the body):
  // no lead-in, the axis starts at the robot
  const bool short_entry = k0 == n && from == 0;
  const double du0 = du(0);
  if (k0 == n && !short_entry) {
    reason = "path starts within axis_margin of the mouth";
    return false;
  }
  if (short_entry && du0 >= hd) {
    reason = "path starts past the far mouth";
    return false;
  }
  for (size_t i = c.last + 1; i < stop; ++i) {
    if (du(i) >= ext) {k1 = i; break;}
  }
  // the path ends inside the zone body: the goal sits on the ramp right behind a tunnel (a chained
  // stretch is longer than the tunnel it contains).  Shape up to the goal and stop on the axis -
  // refusing here left the planner path unshaped, so the robot turned inside the tunnel.
  const bool short_exit = k1 == n && stop == n && du(n - 1) > -hd;
  if (k1 == n && !short_exit) {
    reason = "path ends (or the next zone starts) within axis_margin of the mouth";
    return false;
  }
  size_t ia = short_entry ? 0 : k0;
  while (!short_entry && ia > from && s[k0] - s[ia] < lead) {
    --ia;
  }
  size_t ib = k1;
  while (!short_exit && ib + 1 < stop && s[ib] - s[k1] < lead) {
    ++ib;
  }
  if (short_exit) {
    ib = n - 1;
  }
  // the body ends at the far mouth, or at the goal when that is inside it
  const double b1 = short_exit ? std::min(hd, du(ib)) : hd;

  const double th = normAngle(t.yaw + (dir < 0 ? M_PI : 0.0));
  const Vec ax{std::cos(th), std::sin(th)};
  const Vec t0 = smoothTangent(p, s, ia, prm.tangent_window), t1 = smoothTangent(p, s, ib, prm.tangent_window);
  // the lead-in starts at the robot: turn from the robot heading, and keep the zone end it is closer to
  const bool from_robot = ia == 0 && start_yaw != nullptr;
  const double h0 = from_robot ? *start_yaw :
    (from > 0 ? p[ia].yaw : std::atan2(t0.y, t0.x));
  // short entry: the robot is on the axis already, keep the end it is facing (p[0] is the robot pose)
  const double href = short_entry ? (start_yaw ? *start_yaw : p[0].yaw) : h0;
  // a slope is charged nose first up the hill: no 180 deg flip, the heading is the uphill heading
  const double heading = (!t.allow_flip || std::abs(normAngle(th - href)) <= M_PI / 2) ? th : normAngle(th + M_PI);
  double goal_v = 0.0, goal_u = 0.0;
  if (short_exit) {
    t.toLocal(p.back().x, p.back().y, goal_u, goal_v);
    if (t.recentre && std::abs(goal_v) > max_body + 1e-6) {
      reason = "goal inside the constrained stretch is too far from its axis";
      return false;
    }
    if (std::abs(normAngle(p.back().yaw - heading)) > 3.0 * M_PI / 180.0) {
      reason = "goal heading conflicts with the constrained stretch";
      return false;
    }
  }

  // a slope is not re-centred: the robot charges up the line it was already going to take, and only the
  // heading is locked.  v0 = the path's lateral offset where it reaches the near mouth.
  double v0 = 0.0, v1 = 0.0;
  if (!t.recentre) {
    const double vc = lateralAt(p, t, dir, -hd, short_entry ? 0 : k0, c.last);
    double entry_u, entry_v;
    t.toLocal(p[c.first].x, p[c.first].y, entry_u, entry_v);
    v0 = std::isfinite(vc) ? vc : entry_v;
    const double far_v = lateralAt(p, t, dir, hd, c.first, std::min(c.last + 1, n - 1));
    double exit_u, exit_v;
    t.toLocal(p[c.last].x, p[c.last].y, exit_u, exit_v);
    v1 = std::isfinite(far_v) ? far_v : exit_v;
  }

  // the first collision seen is that of the default variant (no offset, full turn): it goes into the reason
  std::string first_hit, last_hit;
  // Every candidate pose, including the actual starting pose, must pass collision checks.
  auto collides = [&](const std::vector<Pose2> & q, const char * part) {
      for (const auto & e : q) {
        if (!collision_free(e)) {
          double u, v;
          t.toLocal(e.x, e.y, u, v);
          char buf[200];
          std::snprintf(buf, sizeof(buf), " at %s (%.3f, %.3f, %.1f deg), axis u %+.3f v %+.3f", part, e.x, e.y,
            e.yaw * 180.0 / M_PI, dir * u, v);
          (first_hit.empty() ? first_hit : last_hit) = buf;
          return true;
        }
      }
      return false;
    };
  auto append = [](std::vector<Pose2> & a, const std::vector<Pose2> & b) {
      a.insert(a.end(), b.begin() + 1, b.end());   // b starts on a's last pose
    };
  const auto side = offsetsAround(v0, prm.max_side_offset, prm.offset_step);
  const auto exit_side = offsetsAround(v1, prm.max_side_offset, prm.offset_step);

  const auto body_offsets = short_exit ? std::vector<double>{goal_v} : offsetsAround(v0, max_body, prm.offset_step);
  for (double vb : body_offsets) {
    const double ve = (!t.recentre && !short_exit) ? vb + v1 - v0 : vb;
    const double b0 = short_entry ? std::max(-hd, du0) : -hd;
    auto body = headed(ramp(t, dir, b0, b1, vb, ve, prm.spacing), heading, heading, 1.0);
    double start_u, start_v;
    t.toLocal(p.front().x, p.front().y, start_u, start_v);
    if (short_entry && du0 >= -hd && b1 > b0) {
      const double join_s = std::min(b1, b0 + 0.15);
      const double join_v = vb + (ve - vb) * (join_s - b0) / (b1 - b0);
      body = headed(ramp(t, dir, b0, join_s, start_v, join_v, prm.spacing), heading, heading, 1.0);
      const auto tail = headed(ramp(t, dir, join_s, b1, join_v, ve, prm.spacing), heading, heading, 1.0);
      append(body, tail);
    }
    if (collides(body, "zone body")) {
      continue;
    }
    std::vector<Pose2> entry;
    if (short_entry) {
      if (du0 < -hd - 1e-3) {
        entry = headed(ramp(t, dir, du0, -hd, start_v, vb, prm.spacing), heading, heading, 1.0);
        if (collides(entry, "entry margin")) {
          continue;
        }
      } else {
        entry = {body.front()};
      }
      ev.turn = TurnMode::Locked;
      ev.v_in = vb;
    }
    for (double vi : side) {
      if (short_entry) {
        break;
      }
      const Pose2 Ai = t.toMap(-dir * ext, vi, 0.0);
      const auto cin = bezier({p[ia].x, p[ia].y}, t0, {Ai.x, Ai.y}, ax, prm.spacing);
      const auto rin = headed(ramp(t, dir, -ext, -hd, vi, vb, prm.spacing), heading, heading, 1.0);
      if (collides(rin, "entry margin")) {
        continue;
      }
      // a lead-in from a robot that is nearly aligned already finishes its turn within the first
      // `early_turn` m: keeping the residual yaw down a 0.7 m approach channel (5 cm per side for the
      // 0.6 m chassis) put the front corner on the wall at -8 deg in Gazebo (tunnel_1)
      const double arc = cin.size() > 1 ? [&] {
          double a = 0.0;
          for (size_t i = 1; i < cin.size(); ++i) {
            a += std::hypot(cin[i].x - cin[i - 1].x, cin[i].y - cin[i - 1].y);
          }
          return a;
        }() : 0.0;
      const bool early = from_robot && arc > 1e-6 &&
        std::abs(normAngle(heading - h0)) <= prm.early_turn_max_yaw;
      const double early_end = arc > 1e-6 ? std::clamp(prm.early_turn / arc, 0.0, 1.0) : 0.0;
      std::vector<std::pair<TurnMode, double>> modes;
      if (early) {
        modes.push_back({TurnMode::Half, early_end});
      }
      modes.push_back({TurnMode::Full, 1.0});
      modes.push_back({TurnMode::Half, 0.5});
      // A later stretch starts at a heading already fixed by the preceding
      // stretch. The locked variant cannot teleport that shared boundary yaw.
      if (from == 0 || std::abs(normAngle(heading - h0)) < 1e-3) {
        modes.push_back({TurnMode::Locked, 0.0});
      }
      for (const auto & [m, end] : modes) {
        auto lin = headed(cin, h0, heading, end);
        if (collides(lin, "lead-in")) {
          continue;
        }
        append(lin, rin);
        entry.swap(lin);
        ev.turn = m;
        ev.v_in = vi;
        break;
      }
      if (!entry.empty()) {
        break;
      }
    }
    if (entry.empty()) {
      continue;
    }
    std::vector<Pose2> exit;
    size_t exit_axis_size = 0;
    if (short_exit) {
      exit = headed(ramp(t, dir, b1, du(ib), vb, goal_v, prm.spacing), heading, heading, 1.0);
      ev.v_out = vb;
    }
    for (double vo : exit_side) {
      if (short_exit) {
        break;
      }
      const Pose2 Bo = t.toMap(dir * ext, vo, 0.0);
      auto rout = headed(ramp(t, dir, hd, ext, ve, vo, prm.spacing), heading, heading, 1.0);
      const auto lout = headed(bezier({Bo.x, Bo.y}, ax, {p[ib].x, p[ib].y}, t1, prm.spacing),
          heading, std::atan2(t1.y, t1.x), 1.0);  // turn only after the full chassis clears the axis margin
      if (collides(rout, "exit margin") || collides(lout, "lead-out")) {
        continue;
      }
      exit_axis_size = rout.size();
      append(rout, lout);
      exit.swap(rout);
      ev.v_out = vo;
      break;
    }
    if (exit.empty()) {
      continue;
    }
    std::vector<Pose2> seg = entry;
    append(seg, body);
    append(seg, exit);
    if (ib + 1 == n) {
      // Never silently change the user's goal or its final orientation.
      seg.back() = p.back();
      if (!collision_free(seg.back())) {continue;}
    }
    out.assign(p.begin(), p.begin() + ia);
    out.insert(out.end(), seg.begin(), seg.end());
    out.insert(out.end(), p.begin() + ib + 1, p.end());
    // A following zone may use this lead-out as its own approach. Preserve
    // the current body and full straight clearance, then let the next shaping
    // pass revise only the remaining curve. Skipping the entire lead-out could
    // start the next search inside a ramp and misclassify its uphill direction.
    next = short_exit ? ia + seg.size() : ia + entry.size() + body.size() + exit_axis_size - 3;
    ev.v_body = vb;
    ev.heading = heading;
    return true;
  }
  reason = std::string(kCollisionReason) + first_hit + "; no offset / turn variant fits, last tried" + last_hit;
  return false;
}

}  // namespace

std::vector<Zone> groupZones(const std::vector<Zone> & zones, const ChainParams & chain)
{
  const size_t n = zones.size();
  std::vector<size_t> root(n);
  for (size_t i = 0; i < n; ++i) {
    root[i] = i;
  }
  auto find = [&](size_t i) {
      while (root[i] != i) {
        root[i] = root[root[i]];
        i = root[i];
      }
      return i;
    };
  // zones whose axes are parallel, side by side and nearly touching are one stretch: maps/sim has
  // tunnel_2 + slope_1, tunnel_3 + slope_3, tunnel_4 + slope_7, tunnel_5 + slope_9.  Shaping them apart
  // would put a lead-out and a lead-in inside the 0.4 m between the mouths, and the robot leaving a tunnel
  // onto the ramp behind it would turn on the ramp.
  auto chainable = [&](const Zone & a, const Zone & b) {
      // Keep adjacent open faces separate (e.g. six fortress slopes). Chaining is reserved for a
      // narrow passage sharing its short approach with a directional zone.
      if (a.profile == ZoneProfile::OpenDirectional && b.profile == ZoneProfile::OpenDirectional) {
        return false;
      }
      if (!a.axis_reliable || !b.axis_reliable) {
        return false;
      }
      const double dyaw = normAngle(b.yaw - a.yaw);
      if (std::abs(dyaw) > chain.angle && std::abs(std::abs(dyaw) - M_PI) > chain.angle) {
        return false;
      }
      double u, v;
      a.toLocal(b.cx, b.cy, u, v);
      if (std::abs(v) > chain.lat) {
        return false;
      }
      const double ha = 0.5 * a.depth;
      const double hb = 0.5 * (std::abs(b.depth * std::cos(dyaw)) + std::abs(b.width * std::sin(dyaw)));
      const double gap = std::max({u - hb - ha, -u - hb - ha, 0.0});
      return gap <= chain.gap;
    };
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      if (find(i) != find(j) && chainable(zones[i], zones[j])) {
        root[find(j)] = find(i);
      }
    }
  }
  std::vector<Zone> out;
  std::vector<bool> done(n, false);
  for (size_t i = 0; i < n; ++i) {
    if (done[i]) {
      continue;
    }
    std::vector<size_t> members;
    for (size_t j = i; j < n; ++j) {
      if (find(j) == find(i)) {
        members.push_back(j);
        done[j] = true;
      }
    }
    if (members.size() == 1) {
      out.push_back(zones[i]);
      continue;
    }
    // the narrowest member gives the axis (a tunnel inside a chain: its opening is the tight part)
    size_t axis = members.front();
    for (size_t k : members) {
      if (zones[k].width < zones[axis].width) {
        axis = k;
      }
    }
    const Zone & a = zones[axis];
    Zone g = a;
    double umin = std::numeric_limits<double>::infinity(), umax = -umin;
    g.chain = static_cast<int>(members.size());
    g.id.clear();
    bool any_tunnel = false;
    for (size_t k : members) {
      const Zone & m = zones[k];
      const double dyaw = normAngle(m.yaw - a.yaw);
      double u, v;
      a.toLocal(m.cx, m.cy, u, v);
      const double h = 0.5 * (std::abs(m.depth * std::cos(dyaw)) + std::abs(m.width * std::sin(dyaw)));
      umin = std::min(umin, u - h);
      umax = std::max(umax, u + h);
      g.width = std::min(g.width, m.width);
      g.id += (g.id.empty() ? "" : "+") + m.id;
      any_tunnel = any_tunnel || m.profile == ZoneProfile::NarrowPassage;
    }
    const Pose2 c = a.toMap(0.5 * (umin + umax), 0.0, 0.0);
    g.cx = c.x;
    g.cy = c.y;
    g.depth = umax - umin;
    // a chain is one-way only when every member allows the same direction, expressed in the chain frame:
    // a slope's uphill is +1 when its azimuth points along the chain axis, -1 when against it.  A tunnel
    // allows both ways, so a mixed tunnel/slope chain is bidirectional as a whole. The standalone
    // slope's downhill delegation does not apply to that grouped stretch.
    int dir = 0;
    bool first = true;
    for (size_t k : members) {
      const Zone & m = zones[k];
      int d = 0;
      if (m.one_way()) {
        d = std::cos(normAngle(m.yaw - a.yaw)) >= 0.0 ? 1 : -1;
      }
      if (first) {
        dir = d;
        first = false;
      } else if (d != dir) {
        dir = 0;   // a tunnel member (d = 0) or opposing slopes: both ways
      }
    }
    g.one_way_dir = dir;
    g.allow_flip = dir == 0;
    g.profile = any_tunnel ? ZoneProfile::NarrowPassage : ZoneProfile::OpenDirectional;
    g.recentre = any_tunnel;
    out.push_back(g);
  }
  return out;
}

std::vector<Pose2> shapePath(
  const std::vector<Pose2> & path, const std::vector<Zone> & zones, const ShaperParams & prm,
  const CollisionFree & collision_free, std::vector<ShapeEvent> * events, const double * start_yaw)
{
  const std::vector<Zone> groups = groupZones(zones, prm.chain);
  std::vector<Pose2> cur = path;
  size_t from = 0;
  // A suffix replanned just beyond the far mouth contains no body sample, but
  // the chassis still occupies the exit. Shape its clearance and lead-out too;
  // otherwise tracking adds an unvalidated axis/flip to an ordinary grid path.
  if (cur.size() >= 3) {
    for (const auto & zone : groups) {
      if (zone.profile != ZoneProfile::NarrowPassage || !zone.axis_reliable) {continue;}
      Crossing existing;
      if (findCrossing(cur, zone, 0, existing)) {continue;}
      double u, v; zone.toLocal(cur.front().x, cur.front().y, u, v);
      const int dir = u > 0. ? 1 : -1;
      const double beyond = dir * u - .5 * zone.depth;
      const double chassis_clear = prm.exit_clearance;
      const double heading = start_yaw ? *start_yaw : cur.front().yaw;
      if (beyond < 0. || beyond >= chassis_clear || std::abs(v) >= .5 * zone.width ||
        std::abs(std::sin(heading - zone.yaw)) > .5) {continue;}
      size_t j = 1;
      while (j + 1 < cur.size() && std::hypot(cur[j].x - cur[0].x, cur[j].y - cur[0].y) < .1) {++j;}
      double uj, vj; zone.toLocal(cur[j].x, cur[j].y, uj, vj);
      if (dir * (uj - u) <= .5 * std::hypot(cur[j].x - cur[0].x, cur[j].y - cur[0].y)) {continue;}
      Zone extended = zone;
      const auto center = zone.toMap(dir * .5 * chassis_clear, 0., 0.);
      extended.cx = center.x; extended.cy = center.y; extended.depth += chassis_clear;
      Crossing crossing;
      if (!findCrossing(cur, extended, 0, crossing) || !crossing.through) {continue;}
      size_t stop = cur.size();
      for (const auto & other : groups) {
        if (&other == &zone) {continue;}
        Crossing next;
        if (findCrossing(cur, other, crossing.last + 1, next)) {stop = std::min(stop, next.first);}
      }
      ShapeEvent event; event.zone_id = zone.id + " exit"; event.profile = zone.profile;
      event.chain = zone.chain; event.direction = dir;
      double lead = prm.lead_length;
      for (int k = 0; k <= prm.max_retries; ++k, lead *= prm.lead_growth) {
        std::vector<Pose2> out; size_t next = 0; std::string reason;
        if (shapeCrossing(cur, extended, crossing, 0, stop, lead, prm, collision_free,
          start_yaw, out, next, event, reason)) {
          cur.swap(out); from = next; event.shaped = true; event.lead = lead; break;
        }
        event.reason = reason;
        if (reason.rfind(kCollisionReason, 0) != 0) {break;}
      }
      if (events) {events->push_back(event);}
      break;
    }
  }
  while (cur.size() >= 3 && from < cur.size()) {
    const Zone * tb = nullptr;
    Crossing cb;
    for (const auto & t : groups) {
      Crossing c;
      if (findCrossing(cur, t, from, c) && (!tb || c.first < cb.first)) {
        tb = &t;
        cb = c;
      }
    }
    if (!tb) {
      break;
    }
    // the lead-out must not reach into the next zone on the path
    size_t stop = cur.size();
    for (const auto & t : groups) {
      if (&t == tb) {
        continue;
      }
      Crossing c;
      if (findCrossing(cur, t, cb.last + 1, c)) {
        stop = std::min(stop, c.first);
      }
    }
    ShapeEvent ev;
    ev.zone_id = tb->id;
    ev.profile = tb->profile;
    ev.chain = tb->chain;
    ev.direction = cb.dir;
    size_t next = cb.last + 1;
    if (tb->one_way() && cb.dir != tb->one_way_dir) {
      ev.reason = "downhill on a one-way slope, planner path kept";
    } else if (!tb->axis_reliable) {
      ev.reason = "slope direction ambiguous (multi-direction patch), planner path kept";
    } else if (!cb.through) {
      ev.reason = (cb.first == 0 || cb.last + 1 >= cur.size()) ?
        "path starts or ends inside the zone" : "path enters and leaves on the same side";
    } else {
      double lead = tb->profile == ZoneProfile::OpenDirectional ? prm.slope_lead_length : prm.lead_length;
      for (int k = 0; k <= prm.max_retries; ++k, lead *= prm.lead_growth) {
        std::vector<Pose2> out;
        size_t nx = 0;
        std::string why;
        ShapeEvent trial = ev;
        if (shapeCrossing(cur, *tb, cb, from, stop, lead, prm, collision_free, from == 0 ? start_yaw : nullptr,
          out, nx, trial, why)) {
          cur.swap(out);
          next = nx;
          ev = trial;
          ev.shaped = true;
          ev.lead = lead;
          ev.reason.clear();
          break;
        }
        ev.reason = why;
        if (why.rfind(kCollisionReason, 0) != 0) {
          break;   // geometry problems do not go away with a longer lead
        }
      }
    }
    if (events) {
      events->push_back(ev);
    }
    from = next;
  }
  return cur;
}

}  // namespace nav_behaviors
