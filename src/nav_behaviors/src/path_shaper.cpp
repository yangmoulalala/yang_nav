#include "nav_behaviors/path_shaper.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace nav_behaviors
{

const char * const kCollisionReason = "footprint collision";

const char * turnName(TurnMode m)
{
  switch (m) {
    case TurnMode::Full: return "full lead";
    case TurnMode::Half: return "first half";
    default: return "locked";
  }
}

double normAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

void Tunnel::toLocal(double x, double y, double & u, double & v) const
{
  const double dx = x - cx, dy = y - cy, c = std::cos(yaw), s = std::sin(yaw);
  u = c * dx + s * dy;
  v = -s * dx + c * dy;
}

Pose2 Tunnel::toMap(double u, double v, double yaw_rel) const
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
std::vector<Vec> ramp(const Tunnel & t, int dir, double s0, double s1, double va, double vb, double spacing)
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

// 0, +step, -step, 2 step, ... up to max
std::vector<double> offsets(double max, double step)
{
  std::vector<double> out{0.0};
  if (step <= 0.0) {
    return out;
  }
  for (double v = step; v <= max + 1e-9; v += step) {
    out.push_back(v);
    out.push_back(-v);
  }
  return out;
}

struct Crossing
{
  size_t first{0}, last{0};   // samples inside the tunnel body
  int dir{0};
  bool through{false};
};

bool findCrossing(const std::vector<Pose2> & p, const Tunnel & t, size_t from, Crossing & c)
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
  if (f == 0 || l + 1 >= n) {
    return true;   // starts or ends inside the tunnel body
  }
  const double ub = u[f - 1], ua = u[l + 1];
  c.dir = (ub < 0 && ua > 0) ? 1 : (ub > 0 && ua < 0) ? -1 : 0;
  c.through = c.dir != 0;
  return true;
}

bool shapeCrossing(
  const std::vector<Pose2> & p, const Tunnel & t, const Crossing & c, size_t from, size_t stop,
  double lead, const ShaperParams & prm, const CollisionFree & collision_free,
  std::vector<Pose2> & out, size_t & next, ShapeEvent & ev, std::string & reason)
{
  const size_t n = p.size();
  const int dir = c.dir;
  const double hd = 0.5 * t.depth, ext = hd + prm.axis_margin;
  const auto s = arcLength(p);
  auto du = [&](size_t i) {double u, v; t.toLocal(p[i].x, p[i].y, u, v); return dir * u;};

  size_t k0 = n, k1 = n;
  for (size_t i = c.first; i-- > from; ) {
    if (du(i) <= -ext) {k0 = i; break;}
  }
  if (k0 == n) {
    reason = "path starts within axis_margin of the mouth";
    return false;
  }
  for (size_t i = c.last + 1; i < stop; ++i) {
    if (du(i) >= ext) {k1 = i; break;}
  }
  if (k1 == n) {
    reason = "path ends (or the next tunnel starts) within axis_margin of the mouth";
    return false;
  }
  size_t ia = k0;
  while (ia > from && s[k0] - s[ia] < lead) {
    --ia;
  }
  size_t ib = k1;
  while (ib + 1 < stop && s[ib] - s[k1] < lead) {
    ++ib;
  }

  const double th = normAngle(t.yaw + (dir < 0 ? M_PI : 0.0));
  const Vec ax{std::cos(th), std::sin(th)};
  const Vec t0 = smoothTangent(p, s, ia, prm.tangent_window), t1 = smoothTangent(p, s, ib, prm.tangent_window);
  const double h0 = std::atan2(t0.y, t0.x);
  const double heading = std::abs(normAngle(th - h0)) <= M_PI / 2 ? th : normAngle(th + M_PI);

  // the first collision seen is that of the default variant (no offset, full turn): it goes into the reason
  std::string first_hit, last_hit;
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
  const auto side = offsets(prm.max_side_offset, prm.offset_step);

  for (double vb : offsets(prm.max_body_offset, prm.offset_step)) {
    const auto body = headed(ramp(t, dir, -hd, hd, vb, vb, prm.spacing), heading, heading, 1.0);
    if (collides(body, "tunnel body")) {
      continue;
    }
    std::vector<Pose2> entry;
    for (double vi : side) {
      const Pose2 Ai = t.toMap(-dir * ext, vi, 0.0);
      const auto cin = bezier({p[ia].x, p[ia].y}, t0, {Ai.x, Ai.y}, ax, prm.spacing);
      const auto rin = headed(ramp(t, dir, -ext, -hd, vi, vb, prm.spacing), heading, heading, 1.0);
      if (collides(rin, "entry margin")) {
        continue;
      }
      for (TurnMode m : {TurnMode::Full, TurnMode::Half, TurnMode::Locked}) {
        const double end = m == TurnMode::Full ? 1.0 : m == TurnMode::Half ? 0.5 : 0.0;
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
    for (double vo : side) {
      const Pose2 Bo = t.toMap(dir * ext, vo, 0.0);
      auto rout = headed(ramp(t, dir, hd, ext, vb, vo, prm.spacing), heading, heading, 1.0);
      const auto lout = headed(bezier({Bo.x, Bo.y}, ax, {p[ib].x, p[ib].y}, t1, prm.spacing),
          heading, heading, 1.0);   // keep the axis heading, the controller turns afterwards
      if (collides(rout, "exit margin") || collides(lout, "lead-out")) {
        continue;
      }
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
    out.assign(p.begin(), p.begin() + ia);
    out.insert(out.end(), seg.begin(), seg.end());
    out.insert(out.end(), p.begin() + ib + 1, p.end());
    next = ia + seg.size();
    ev.v_body = vb;
    ev.heading = heading;
    return true;
  }
  reason = std::string(kCollisionReason) + first_hit + "; no offset / turn variant fits, last tried" + last_hit;
  return false;
}

}  // namespace

std::vector<Pose2> shapePath(
  const std::vector<Pose2> & path, const std::vector<Tunnel> & tunnels, const ShaperParams & prm,
  const CollisionFree & collision_free, std::vector<ShapeEvent> * events)
{
  std::vector<Pose2> cur = path;
  size_t from = 0;
  while (cur.size() >= 3 && from < cur.size()) {
    const Tunnel * tb = nullptr;
    Crossing cb;
    for (const auto & t : tunnels) {
      Crossing c;
      if (findCrossing(cur, t, from, c) && (!tb || c.first < cb.first)) {
        tb = &t;
        cb = c;
      }
    }
    if (!tb) {
      break;
    }
    // the lead-out must not reach into the next tunnel on the path
    size_t stop = cur.size();
    for (const auto & t : tunnels) {
      Crossing c;
      if (&t != tb && findCrossing(cur, t, cb.last + 1, c)) {
        stop = std::min(stop, c.first);
      }
    }
    ShapeEvent ev;
    ev.tunnel_id = tb->id;
    ev.direction = cb.dir;
    size_t next = cb.last + 1;
    if (!cb.through) {
      ev.reason = (cb.first == 0 || cb.last + 1 >= cur.size()) ?
        "path starts or ends inside the tunnel" : "path enters and leaves on the same side";
    } else {
      double lead = prm.lead_length;
      for (int k = 0; k <= prm.max_retries; ++k, lead *= prm.lead_growth) {
        std::vector<Pose2> out;
        size_t nx = 0;
        std::string why;
        ShapeEvent trial = ev;
        if (shapeCrossing(cur, *tb, cb, from, stop, lead, prm, collision_free, out, nx, trial, why)) {
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
