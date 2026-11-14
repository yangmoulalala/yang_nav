// Deterministic omni tracker for direction-constrained zones, with no ROS dependency.
//
// Window: from `window_before` m of path before the near mouth to `window_after` m after the far mouth
// (arc length along the current path, which the ZonePathShaper has put on the zone axis).  Inside it the
// output `weight` ramps 0 -> 1 over `blend` m at the start and back at the end; the caller mixes
// (1 - weight) * inner controller + weight * tracker. Gates and path-end limits may stop motion.
//
// Tracking (world frame, then rotated into the robot frame):
//   translation  v_ref along the path tangent + lateral P correction towards the path (clamped)
//   heading      the path heading is the only reference (the shaper wrote it and collision-checked it),
//                including any backwards traversal selected by the shaper. The heading
//                error found at that moment decays smoothly to zero over the shortest distance the turn
//                rate allows (>= min_turn_span, at the latest `align_before` before the mouth), with
//                feed-forward; from there the path heading is held.  A slope never flips: it is charged
//                nose first up the hill (the shaper wrote the uphill heading).
//   speed        approach_speed, decelerating (decel) to the body speed at align_before before the mouth,
//                body speed through the body, accelerating again 0.1 m after it; capped by the path end.
//                Body speed is `tunnel_speed` in a tunnel and `ramp_speed` on a slope: the robot charges
//                up the ramp instead of creeping.  A slope whose heading is still off is held at
//                `align_speed` until it is aligned, so the charge always starts pointing uphill.
//   gate         measured at the leading edge (front_extent ahead, rear_extent when the robot drives
//                backwards): within gate_dist before the mouth, |lateral| > lat_tol or |heading| > yaw_tol
//                scales the speed down to a full stop gate_stop before the mouth while the corrections
//                continue; it releases once on track (no stop when the robot arrives aligned).  A slope
//                uses `ramp_lat_tol` (its face is wide, only the heading really matters there).
//
// A path that starts inside the body (replan after a stop in the zone) is a crossing too: its near
// mouth lies behind the path start.
//
// Phases (zone_state.hpp): approach while the weight ramps up, align once the tracker alone drives
// (s_rel > -(window_before - blend)) until the mouth, traverse in the body, exit after it.
#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"
#include "nav_behaviors/zone_state.hpp"

namespace nav_behaviors
{

struct TrackerParams
{
  double window_before{2.0};
  double window_after{0.9};
  double blend{0.4};
  double exit_turn_span{1.5};  // suffix yaw joins over the shaper lead length, not the weight blend
  double align_before{0.8};    // centre distance: front 0.4 m before the mouth
  double front_extent{0.39};   // footprint reach ahead of the centre along the heading
  double rear_extent{0.31};    // ... and behind it (leading edge when driving backwards)
  double gate_stop{0.03};
  double tunnel_speed{0.6};    // m/s through a tunnel body
  double approach_speed{1.0};
  double decel{1.5};
  double gate_dist{0.3};
  double lat_tol{0.03};
  double yaw_tol{3.0 * M_PI / 180.0};
  double k_lat{3.0};
  double max_lat_speed{0.3};
  double k_yaw{3.0};
  double max_wz{1.0};
  double min_turn_span{0.3};   // entry heading error removed over at least this much arc length, m
  double wedge_yaw{6.0 * M_PI / 180.0};   // leading edge in the mouth with this heading error: wedged, rad
  double wedge_lat{0.06};      // ... or this lateral error, m
  double align_speed{0.7};     // speed cap once the tracker drives alone before the mouth, m/s
  double backoff_speed{0.15};  // wedged: back off along the path at this speed, m/s
  double search_dist{1.0};

  // open directional terrain uses a separate speed profile while sharing the same tracker
  double ramp_speed{1.4};          // m/s through a slope body
  double ramp_approach_speed{1.4}; // m/s on the run-up (no slow crawl before a wide ramp)
  double ramp_decel{2.0};
  double ramp_align_before{0.6};
  double ramp_lat_tol{0.12};       // the face is wide: a few cm off the line is fine
  double ramp_window_before{2.0};
  double ramp_window_after{0.6};

  ChainParams chain;   // zones shaped as one stretch: must match the shaper's
};

struct TrackerOutput
{
  double weight{0.0};            // 0 = inner controller only, 1 = tracker only
  double vx{0.0}, vy{0.0}, wz{0.0};   // robot frame
  double align_vx{0.0}, align_vy{0.0}; // lateral correction without longitudinal advance
  ZonePhase phase{ZonePhase::None};
  ZoneProfile profile{ZoneProfile::NarrowPassage};
  bool zone_mode{false};         // close to / inside the body: /nav_mode tunnel or ramp
  bool in_body{false};
  bool gated{false};
  bool backing_off{false};       // wedged before the mouth, backing off along the path
  std::string zone_id;
  double s_rel{0.0};             // path arc length relative to the near mouth
  double e_lat{0.0};             // robot left of the path, m
  double e_yaw{0.0};             // robot yaw - reference, rad
  double yaw_ref{0.0};           // heading reference this cycle, rad
  double v_ref{0.0};
  double lock{0.0};              // collision-checked path heading held through the body
};

class ZoneTracker
{
public:
  void setZones(const std::vector<Zone> & z)
  {
    zones_ = z;
    crossings_.clear();
  }
  void setParams(const TrackerParams & p) {prm_ = p;}
  const TrackerParams & params() const {return prm_;}
  void setPath(const std::vector<Pose2> & path);
  void resetProgress()
  {
    have_nearest_ = false;
    for (auto & c : crossings_) {
      c.locked = c.backing = c.have_prev = false;
      c.against = 0;
    }
  }
  size_t crossingCount() const {return crossings_.size();}
  TrackerOutput compute(const Pose2 & robot, double v_cap = std::numeric_limits<double>::infinity());
  // path poses within `dist` m of arc length ahead of the last nearest point (obstacle check)
  std::vector<Pose2> ahead(double dist, double spacing = 0.025) const;

private:
  struct Crossing
  {
    size_t zone{0};
    double s0{0.0}, s1{0.0};   // arc length of the near / far mouth on the path (s0 < 0: starts inside)
    double body_yaw{0.0};      // path heading in the body
    int dir{0};
    bool locked{false};
    bool exit_only{false};       // replan begins beyond the far mouth, rear may still be inside
    bool backing{false};       // wedged before the mouth, backing off (hysteresis)
    bool have_prev{false};
    double prev_yaw{0.0}, prev_wz{0.0};
    int against{0};            // consecutive cycles rotating against the commanded turn
    double e0{0.0}, entry_s{0.0};
  };
  double pathYaw(size_t j, double f) const;
  double exitProgress(const Crossing & crossing, const Pose2 & pose) const;

  std::vector<Zone> zones_;
  std::vector<Pose2> path_;
  std::vector<double> s_, yaw_;   // arc length, smoothed path heading
  std::vector<Crossing> crossings_;
  TrackerParams prm_;
  size_t nearest_{0};
  size_t active_crossing_{std::numeric_limits<size_t>::max()};
  double nearest_s_{0.0};
  bool have_nearest_{false};
};

}  // namespace nav_behaviors
