#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace nav_behaviors
{

const char * zoneProfileName(ZoneProfile profile)
{
  return profile == ZoneProfile::OpenDirectional ? "open" : "narrow";
}

const char * zoneModeName(ZoneProfile profile)
{
  return profile == ZoneProfile::OpenDirectional ? "ramp" : "tunnel";
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
  // Exported face rectangles bound trapezoids and overlap at their seams. A
  // centre-point plan near that overlap can put the wheels on two different
  // slopes even when the occupancy footprint is entirely free. Keep distinct
  // faces, but use their central corridor and clear the rear before turning.
  // The separating-axis test also avoids imposing this policy on distant ramps.
  auto touching = [](const Zone & a, const Zone & b) {
      for (double axis : {a.yaw, a.yaw + M_PI / 2., b.yaw, b.yaw + M_PI / 2.}) {
        const double distance = std::abs((b.cx - a.cx) * std::cos(axis) +
          (b.cy - a.cy) * std::sin(axis));
        auto radius = [&](const Zone & z) {
            return .5 * (z.depth * std::abs(std::cos(z.yaw - axis)) +
              z.width * std::abs(std::sin(z.yaw - axis)));
          };
        if (distance > radius(a) + radius(b) + .05) {return false;}
      }
      return true;
    };
  for (size_t i = 0; i < out.size(); ++i) {
    for (size_t j = i + 1; j < out.size(); ++j) {
      auto & a = out[i];
      auto & b = out[j];
      if (a.profile != ZoneProfile::OpenDirectional || b.profile != ZoneProfile::OpenDirectional ||
        !a.axis_reliable || !b.axis_reliable || !a.one_way() || !b.one_way()) {continue;}
      const double angle = std::abs(normAngle(a.yaw - b.yaw));
      if (angle > chain.angle && angle < M_PI - chain.angle && touching(a, b)) {
        a.seam_constrained = b.seam_constrained = true;
      }
    }
  }
  return out;
}

}  // namespace nav_behaviors
