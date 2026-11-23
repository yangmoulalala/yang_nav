#include "nav_behaviors/zone_tracker.hpp"
#include "nav_behaviors/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace nav_behaviors
{

namespace
{

// per-kind window / speed profile: a tunnel is a 0.25 m high slot (creep, tight gate), a slope is an open
// face (charge up it, only the heading has to be right)
struct KindParams
{
  double window_before, window_after, body_speed, approach_speed, decel, align_before, lat_tol;
};

KindParams kindParams(const Zone & z, const TrackerParams & p)
{
  if (z.profile == ZoneProfile::OpenDirectional) {
    return {p.ramp_window_before, p.ramp_window_after, p.ramp_speed, p.ramp_approach_speed, p.ramp_decel,
      p.ramp_align_before, p.ramp_lat_tol};
  }
  return {p.window_before, p.window_after, p.tunnel_speed, p.approach_speed, p.decel, p.align_before, p.lat_tol};
}

}  // namespace

void ZoneTracker::setPath(const std::vector<Pose2> & path)
{
  std::vector<Crossing> old;
  old.swap(crossings_);
  path_ = path;
  have_nearest_ = false;
  active_crossing_ = std::numeric_limits<size_t>::max();
  if (path_.size() < 2) {
    path_.clear();
    s_.clear();
    yaw_.clear();
    return;
  }
  const size_t n = path_.size();
  s_.assign(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    s_[i] = s_[i - 1] + std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
  }
  // pose headings, circular mean over +-5 cm: the shaped stretch is smooth already, the grid stretches of
  // the planner path zig-zag cell by cell
  yaw_.assign(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    double sx = 0.0, sy = 0.0;
    for (size_t a = i; ; --a) {
      if (s_[i] - s_[a] > 0.05) {break;}
      sx += std::cos(path_[a].yaw);
      sy += std::sin(path_[a].yaw);
      if (a == 0) {break;}
    }
    for (size_t b = i + 1; b < n && s_[b] - s_[i] <= 0.05; ++b) {
      sx += std::cos(path_[b].yaw);
      sy += std::sin(path_[b].yaw);
    }
    yaw_[i] = std::atan2(sy, sx);
  }
  for (size_t k = 0; k < zones_.size(); ++k) {
    const Zone & t = zones_[k];
    if (!t.axis_reliable) {
      continue;   // shaper and tracker must both defer ambiguous multi-direction regions to Nav2
    }
    const double hd = 0.5 * t.depth;
    auto inside = [&](size_t j) {
        double u, v;
        t.toLocal(path_[j].x, path_[j].y, u, v);
        return std::abs(u) <= hd && std::abs(v) < 0.5 * t.width;
      };
    size_t i = 0;
    while (i < n) {
      if (!inside(i)) {
        ++i;
        continue;
      }
      const size_t f = i;
      while (i + 1 < n && inside(i + 1)) {
        ++i;
      }
      const size_t l = i++;
      double ub, ua, v;
      t.toLocal(path_[std::min(l + 1, n - 1)].x, path_[std::min(l + 1, n - 1)].y, ua, v);
      int dir = 0;
      if (l + 1 == n) {
        t.toLocal(path_[f > 0 ? f - 1 : 0].x, path_[f > 0 ? f - 1 : 0].y, ub, v);
        dir = ua > ub ? 1 : ua < ub ? -1 : 0;
      } else if (f == 0) {
        dir = ua > 0 ? 1 : -1;   // starts inside the body: the exit side gives the direction
      } else {
        t.toLocal(path_[f - 1].x, path_[f - 1].y, ub, v);
        dir = (ub < 0 && ua > 0) ? 1 : (ub > 0 && ua < 0) ? -1 : 0;
      }
      if (dir == 0) {
        continue;
      }
      // The current directional policy delegates opposite-direction descent to the planner/MPPI.
      // This does not establish that physical downhill alignment or speed is safe.
      if (t.one_way() && dir != t.one_way_dir) {
        continue;
      }
      Crossing c;
      c.zone = k;
      c.dir = dir;
      if (f == 0) {
        double u0;
        t.toLocal(path_[0].x, path_[0].y, u0, v);
        c.s0 = -(dir * u0 + hd);   // near mouth behind the path start
      } else {
        c.s0 = s_[f];
      }
      // Keep the physical far mouth beyond the goal if the path ends inside.
      // Otherwise the exit blend would hand an in-zone goal back to MPPI.
      double ul;
      t.toLocal(path_[l].x, path_[l].y, ul, v);
      c.s1 = s_[l] + std::max(0.0, hd - dir * ul);
      c.body_yaw = yaw_[(f + l) / 2];
      for (const auto & o : old) {   // a replan inside the window keeps the heading already chosen
        if (o.zone == k && o.dir == dir && o.locked) {
          c.locked = true;
          c.e0 = o.e0;
          c.entry_s = o.entry_s;
        }
      }
      crossings_.push_back(c);
    }
  }
  // A replanned suffix can start beyond the far mouth while the rear is still in
  // the body. Reconstruct the exit window even without a body sample in this path.
  for (size_t k = 0; k < zones_.size(); ++k) {
    const auto & z = zones_[k];
    if (!z.axis_reliable || std::any_of(crossings_.begin(), crossings_.end(),
      [&](const auto & c) {return c.zone == k;})) {continue;}
    double u, v; z.toLocal(path_.front().x, path_.front().y, u, v);
    const int dir = u > 0.0 ? 1 : -1;
    const double beyond = dir * u - 0.5 * z.depth;
    const auto kp = kindParams(z, prm_);
    if (beyond < 0.0 || beyond >= kp.window_after || std::abs(v) >= 0.5 * z.width ||
      (z.one_way() && dir != z.one_way_dir)) {continue;}
    const bool known_exit = std::any_of(old.begin(), old.end(), [&](const auto & c) {
        return c.zone == k && c.dir == dir && c.locked;
      });
    // With no crossing history, infer only the part where the chassis may
    // still overlap the mouth. The rest is a blend from a completed crossing,
    // not evidence that a nearby path has traversed this zone. Otherwise an
    // adjacent ramp can repeatedly acquire this unrelated tunnel's exit window.
    const double chassis_clear = std::max(prm_.front_extent, prm_.rear_extent) + 0.05;
    if (!known_exit && (beyond > chassis_clear ||
      std::abs(std::sin(path_.front().yaw - z.yaw)) > 0.5)) {continue;}
    size_t j = 1;
    while (j + 1 < n && s_[j] < 0.1) {++j;}
    double uj, vj; z.toLocal(path_[j].x, path_[j].y, uj, vj);
    if (dir * (uj - u) <= 0.5 * std::hypot(path_[j].x - path_[0].x, path_[j].y - path_[0].y) ||
      s_[j] < 1e-6) {continue;}  // a sideways pass or a new approach from this side is not an exit
    Crossing c; c.zone = k; c.dir = dir; c.exit_only = true;
    c.s0 = -dir * u - 0.5 * z.depth;
    c.s1 = -beyond;
    // The shaper has already chosen and collision-checked the physical heading.
    // Recover its axis direction, including a backwards traversal.
    c.body_yaw = normAngle(z.yaw +
      (std::cos(path_.front().yaw - z.yaw) < 0.0 ? M_PI : 0.0));
    for (const auto & o : old) {
      if (o.zone == k && o.dir == dir && o.locked) {
        c.locked = true; c.e0 = o.e0; c.entry_s = o.entry_s;
      }
    }
    crossings_.push_back(c);
  }
  std::sort(crossings_.begin(), crossings_.end(), [](const Crossing & a, const Crossing & b) {return a.s0 < b.s0;});
}

double ZoneTracker::pathYaw(size_t j, double f) const
{
  return normAngle(yaw_[j] + f * normAngle(yaw_[j + 1] - yaw_[j]));
}

double ZoneTracker::exitProgress(const Crossing & crossing, const Pose2 & pose) const
{
  const auto & zone = zones_[crossing.zone];
  double u, v; zone.toLocal(pose.x, pose.y, u, v);
  const double clear = std::max(prm_.front_extent, prm_.rear_extent) + 0.05;
  return (crossing.dir * u - 0.5 * zone.depth - clear) / std::max(0.01, prm_.exit_turn_span);
}

std::vector<Pose2> ZoneTracker::ahead(double dist, double spacing) const
{
  std::vector<Pose2> out;
  if (!have_nearest_ || path_.size() < 2) {
    return out;
  }
  const double target = std::clamp(nearest_s_ + dist, 0.0, s_.back());
  const int count = std::max(1, static_cast<int>(std::ceil(std::abs(target - nearest_s_) / spacing)));
  for (int i = 0; i <= count; ++i) {
    const double s = nearest_s_ + (target - nearest_s_) * static_cast<double>(i) / count;
    auto it = std::upper_bound(s_.begin(), s_.end(), s);
    const size_t j = std::min(path_.size() - 2, it == s_.begin() ? 0u : static_cast<size_t>(it - s_.begin() - 1));
    const double length = s_[j + 1] - s_[j];
    const double f = length > 1e-12 ? std::clamp((s - s_[j]) / length, 0.0, 1.0) : 0.0;
    Pose2 pose{path_[j].x + f * (path_[j + 1].x - path_[j].x),
      path_[j].y + f * (path_[j + 1].y - path_[j].y), pathYaw(j, f)};
    if (active_crossing_ < crossings_.size() && crossings_[active_crossing_].exit_only) {
      const auto & c = crossings_[active_crossing_];
      pose.yaw = normAngle(c.body_yaw + smooth01(exitProgress(c, pose)) * normAngle(pose.yaw - c.body_yaw));
    }
    out.push_back(pose);
  }
  return out;
}

TrackerOutput ZoneTracker::compute(const Pose2 & r, double v_cap)
{
  TrackerOutput o;
  active_crossing_ = std::numeric_limits<size_t>::max();
  const size_t n = path_.size();
  if (n < 2 || crossings_.empty()) {
    return o;
  }
  // nearest point on the path (segment projection), searched around the last one
  auto search = [&](size_t a, size_t b, size_t & bj, double & bf) {
      double best = std::numeric_limits<double>::infinity();
      for (size_t j = a; j < b; ++j) {
        const double dx = path_[j + 1].x - path_[j].x, dy = path_[j + 1].y - path_[j].y, l2 = dx * dx + dy * dy;
        const double f = l2 > 1e-12 ? std::clamp(((r.x - path_[j].x) * dx + (r.y - path_[j].y) * dy) / l2, 0.0, 1.0) : 0.0;
        const double ex = path_[j].x + f * dx - r.x, ey = path_[j].y + f * dy - r.y, d2 = ex * ex + ey * ey;
        if (d2 < best) {
          best = d2;
          bj = j;
          bf = f;
        }
      }
      return best;
    };
  size_t bj = 0;
  double bf = 0.0, best;
  if (have_nearest_) {
    const size_t a = nearest_ > 40 ? nearest_ - 40 : 0;
    size_t b = a + 1;
    while (b < n - 1 && s_[b] - s_[nearest_] < prm_.search_dist) {
      ++b;
    }
    best = search(a, std::min(b, n - 1), bj, bf);
    if (best > 0.25 * 0.25) {   // squared distance: 25 cm off the local stretch -> full search
      best = search(0, n - 1, bj, bf);
    }
  } else {
    best = search(0, n - 1, bj, bf);
  }
  nearest_ = bj;
  have_nearest_ = true;

  const Pose2 & A = path_[bj], & B = path_[bj + 1];
  const double seg = std::hypot(B.x - A.x, B.y - A.y);
  const double tx = seg > 1e-9 ? (B.x - A.x) / seg : std::cos(yaw_[bj]);
  const double ty = seg > 1e-9 ? (B.y - A.y) / seg : std::sin(yaw_[bj]);
  const double px = A.x + bf * (B.x - A.x), py = A.y + bf * (B.y - A.y);
  const double s_r = s_[bj] + bf * seg;
  nearest_s_ = s_r;
  const double e_lat = tx * (r.y - py) - ty * (r.x - px);
  o.e_lat = e_lat;

  Crossing * c = nullptr;
  for (auto & x : crossings_) {
    const KindParams kp = kindParams(zones_[x.zone], prm_);
    if (s_r >= x.s0 - kp.window_before && s_r <= x.s1 + kp.window_after) {
      c = &x;
      break;
    }
  }
  if (!c) {
    return o;
  }
  active_crossing_ = static_cast<size_t>(c - crossings_.data());
  const Zone & t = zones_[c->zone];
  const KindParams kp = kindParams(t, prm_);
  const double s_rel = s_r - c->s0, body = c->s1 - c->s0, end_rel = body + kp.window_after;
  double w = 1.0;
  if (s_rel < -kp.window_before + prm_.blend) {
    w = smooth01((s_rel + kp.window_before) / prm_.blend);
  } else if (s_rel > end_rel - prm_.blend) {
    w = smooth01((end_rel - s_rel) / prm_.blend);
  }
  // The path carries the physical heading validated by the shaper, including
  // backwards tunnel traversal. Choosing another pi flip here invalidates that
  // footprint check, especially for the asymmetric chassis and exit turns.
  // Decay the initial tracking error before the mouth without changing the path.
  const double s_al = -kp.align_before;
  double p_yaw = pathYaw(bj, bf);
  double exit_blend = 1.0, exit_rate = 0.0;
  if (c->exit_only) {
    // Keep the body's heading until the complete padded chassis has cleared it.
    // Then join the suffix heading continuously over the ordinary blend distance.
    const double f = exitProgress(*c, r);
    exit_blend = smooth01(f);
    if (f <= 0.0) {w = 1.0;}
    const double delta = normAngle(p_yaw - c->body_yaw);
    exit_rate = delta * dsmooth01(std::clamp(f, 0.0, 1.0)) / std::max(0.01, prm_.exit_turn_span);
    p_yaw = normAngle(c->body_yaw + exit_blend * delta);
  }
  if (!c->locked || s_rel < c->entry_s - 0.2) {   // re-lock when a new approach starts behind the old one
    c->e0 = normAngle(r.yaw - p_yaw);
    c->entry_s = s_rel;
    c->locked = true;
  }
  // path heading rate over +-5 cm of arc length: a per-segment difference divided by a near-zero segment
  // (joints of the shaped stretch, duplicate planner poses) spun the robot at max_wz in Gazebo
  size_t ja = bj, jb = bj + 1;
  while (ja > 0 && s_r - s_[ja] < 0.05) {
    --ja;
  }
  while (jb + 1 < n && s_[jb] - s_r < 0.05) {
    ++jb;
  }
  const double ds_win = s_[jb] - s_[ja];
  const double dyaw_path = (ds_win > 0.04 ? normAngle(yaw_[jb] - yaw_[ja]) / ds_win : 0.0) * exit_blend + exit_rate;
  double yaw_ref = p_yaw, dyaw_ds = dyaw_path;
  if (s_rel < s_al && c->entry_s < s_al) {
    // the entry error goes over the shortest distance the turn rate allows (at most up to align_before):
    // a slow relax over the whole approach kept the robot 8 deg off inside the 0.7 m channel before
    // tunnel_1 (0.71 m across) and it hit the wall
    const double v_est = std::max(kp.body_speed, kp.approach_speed);
    const double need = std::abs(c->e0) * v_est / (0.6 * prm_.max_wz);
    const double span = std::max(1e-3, std::min(std::max(prm_.min_turn_span, need), s_al - c->entry_s));
    const double q = (s_rel - c->entry_s) / span;
    yaw_ref = normAngle(yaw_ref + c->e0 * (1.0 - smooth01(q)));
    dyaw_ds -= c->e0 * dsmooth01(std::clamp(q, 0.0, 1.0)) / span;
  }
  // speed profile
  const double ts = kp.body_speed;
  double v = ts;
  if (s_rel < s_al) {
    v = std::min(kp.approach_speed, std::sqrt(ts * ts + 2.0 * kp.decel * (s_al - s_rel)));
  } else if (s_rel > body + 0.1) {
    v = std::min(kp.approach_speed, std::sqrt(ts * ts + 2.0 * kp.decel * (s_rel - body - 0.1)));
  }
  v = std::min(v, v_cap);
  // tracker alone from here (approach channels 5 cm per side): not faster than align_speed
  if (w >= 1.0 && s_rel < 0.0) {
    v = std::min(v, prm_.align_speed);
  }
  v = std::min(v, std::sqrt(2.0 * kp.decel * std::max(0.0, s_.back() - s_r)));
  // the heading must be able to follow the path: slow down where the path turns faster than 0.8 max_wz
  // allows (lead-in from the robot heading, Gazebo tunnel_3 lagged 15 - 30 deg behind at 1 m/s)
  // Include the entry-error relaxation: its heading rate can exceed the
  // geometric path rate even on a straight approach.
  if (std::abs(dyaw_ds) > 1e-6) {
    v = std::min(v, 0.8 * prm_.max_wz / std::abs(dyaw_ds));
  }
  const double e_yaw = normAngle(r.yaw - yaw_ref);
  // Align before advancing into either a ramp or a narrow approach channel.
  // The mouth gate is too late when a large heading error starts farther away.
  if (std::abs(e_yaw) > 2.0 * prm_.yaw_tol) {
    v = std::min(v, prm_.align_speed) * std::max(0.0, std::cos(e_yaw));
  }
  // leading edge: the front when the robot faces the travel direction, the rear when it drives backwards
  const bool forward = std::cos(r.yaw - std::atan2(ty, tx)) >= 0.0;
  const double lead = forward ? prm_.front_extent : prm_.rear_extent;
  const double trail = forward ? prm_.rear_extent : prm_.front_extent;
  const double front_rel = s_rel + lead;   // leading edge relative to the mouth
  if (front_rel >= -prm_.gate_dist && front_rel < 0.0 &&
    (std::abs(e_lat) > kp.lat_tol || std::abs(e_yaw) > prm_.yaw_tol))
  {
    const double span = std::max(1e-3, prm_.gate_dist - prm_.gate_stop);
    v *= std::clamp((-front_rel - prm_.gate_stop) / span, 0.0, 1.0);
    o.gated = true;
  }
  // wedged in the mouth: the leading edge is at / past the mouth while the centre is still before it and
  // the robot is off the axis or the heading (a corner on the mouth wall; Gazebo tunnel_3 stuck 14 cm off
  // pushing sideways).  Wedged before the mouth: the heading error grew past wedge_yaw_far although the
  // tracker turns the other way (a corner dragged along an approach-channel wall, Gazebo tunnel_1 rotated
  // from -5 to -46 deg against +1 rad/s).  Pushing on only jams it: back off along the path, corrections
  // continuing, until the error is small again (hysteresis, so it does not chatter).
  // A slope has no mouth to wedge in: its face is open, so only the heading test applies, before the hill.
  const double wedge_lat = t.profile == ZoneProfile::OpenDirectional ? 2.0 * kp.lat_tol : prm_.wedge_lat;
  const bool in_mouth = s_rel < 0.0 && front_rel > -prm_.gate_stop &&
    (std::abs(e_yaw) > prm_.wedge_yaw || std::abs(e_lat) > wedge_lat);
  // Wedged before the mouth: the robot keeps rotating against the commanded turn (a corner dragged along an
  // approach-channel wall, Gazebo tunnel_1 rotated from -5 to -46 deg against +1 rad/s).  A robot that only
  // lags behind (slow turn) rotates with the command and is not wedged.
  if (c->have_prev) {
    const double d = normAngle(r.yaw - c->prev_yaw);
    if (std::abs(c->prev_wz) > 0.3 && d * c->prev_wz < 0.0 && std::abs(d) > 0.003) {
      ++c->against;
    } else if (std::abs(d) > 0.003 || std::abs(c->prev_wz) <= 0.3) {
      c->against = 0;
    }
  }
  if (s_rel < 0.0 && c->against >= 3 && std::abs(e_yaw) > prm_.wedge_yaw) {
    c->backing = true;
  } else if (std::abs(e_yaw) < 0.5 * prm_.wedge_yaw && std::abs(e_lat) < wedge_lat) {
    c->backing = false;
  }
  if (s_rel >= 0.0) {
    c->backing = false;
  }
  if (in_mouth || c->backing) {
    v = -prm_.backoff_speed;
    o.gated = true;
    o.backing_off = true;
  }
  const double vl = std::clamp(-prm_.k_lat * e_lat, -prm_.max_lat_speed, prm_.max_lat_speed);
  const double vwx = v * tx - vl * ty, vwy = v * ty + vl * tx;
  const double cy = std::cos(r.yaw), sy = std::sin(r.yaw);
  o.vx = cy * vwx + sy * vwy;
  o.vy = -sy * vwx + cy * vwy;
  o.align_vx = cy * (-vl * ty) + sy * (vl * tx);
  o.align_vy = -sy * (-vl * ty) + cy * (vl * tx);
  // SpeedFilter limits the planar norm, including lateral correction and backoff.
  // Capping only the longitudinal reference violates the limit when off the axis.
  const double speed = std::hypot(o.vx, o.vy);
  if (speed > std::max(0.0, v_cap)) {
    const double scale = std::max(0.0, v_cap) / speed;
    o.vx *= scale;
    o.vy *= scale;
  }
  o.wz = std::clamp(dyaw_ds * v - prm_.k_yaw * e_yaw, -prm_.max_wz, prm_.max_wz);
  c->prev_yaw = r.yaw;
  c->prev_wz = o.wz;
  c->have_prev = true;
  o.yaw_ref = yaw_ref;
  o.weight = w;
  // phase: approach while the weight ramps up, align once the tracker alone drives
  if (s_rel >= body) {
    o.phase = ZonePhase::Exit;
  } else if (s_rel >= 0.0) {
    o.phase = ZonePhase::Traverse;
  } else if (s_rel >= -kp.window_before + prm_.blend) {
    o.phase = ZonePhase::Align;
  } else {
    o.phase = ZonePhase::Approach;
  }
  o.profile = t.profile;
  o.zone_mode = s_rel >= -lead - prm_.gate_dist - 0.1 && s_rel <= body + trail + 0.05;
  o.in_body = s_rel >= 0.0 && s_rel <= body;
  o.zone_id = t.id;
  o.s_rel = s_rel;
  o.e_yaw = e_yaw;
  o.v_ref = v;
  o.lock = c->body_yaw;
  return o;
}

}  // namespace nav_behaviors
