// Deterministic omni tracker for the tunnel window (M3c), no ROS dependency.
//
// Window: from `window_before` m of path before the tunnel mouth to `window_after` m after the far mouth
// (arc length along the current path, which the ZonePathShaper has put on the zone axis).  Inside it the
// output `weight` ramps 0 -> 1 over `blend` m at the start and back at the end; the caller mixes
// (1 - weight) * inner controller + weight * tracker, so the speed never jumps and never drops to zero.
//
// Tracking (world frame, then rotated into the robot frame):
//   translation  v_ref along the path tangent + lateral P correction towards the path (clamped)
//   heading      locked heading = zone yaw or yaw + pi, whichever is closer to the robot when the window
//                starts (omni: no turnaround); turned smoothly from the entry yaw until the centre is
//                `align_before` m before the mouth, with feed-forward, then held until the window ends
//   speed        approach_speed, decelerating (decel) to tunnel_speed at align_before before the mouth,
//                tunnel_speed through the body, accelerating again 0.1 m after it; capped by the path end
//   gate         measured at the robot front (centre + front_extent): within gate_dist before the mouth,
//                |lateral| > lat_tol or |heading| > yaw_tol scales the speed down to a full stop
//                gate_stop before the mouth while the corrections continue; it releases once on track
//                (no stop when the robot arrives aligned)
#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav_behaviors/path_shaper.hpp"

namespace nav_behaviors
{

struct TrackerParams
{
  double window_before{2.0};
  double window_after{0.6};
  double blend{0.4};
  double align_before{0.8};    // centre distance: front 0.4 m before the mouth
  double front_extent{0.39};   // footprint reach ahead of the centre along the heading
  double gate_stop{0.03};
  double tunnel_speed{0.6};
  double approach_speed{1.0};
  double decel{1.5};
  double gate_dist{0.3};
  double lat_tol{0.03};
  double yaw_tol{3.0 * M_PI / 180.0};
  double k_lat{3.0};
  double max_lat_speed{0.3};
  double k_yaw{3.0};
  double max_wz{1.0};
  double search_dist{1.0};
};

struct TrackerOutput
{
  double weight{0.0};            // 0 = inner controller only, 1 = tracker only
  double vx{0.0}, vy{0.0}, wz{0.0};   // robot frame
  bool tunnel_mode{false};       // close to / inside the tunnel body: /nav_mode tunnel
  bool in_body{false};
  bool gated{false};
  std::string tunnel_id;
  double s_rel{0.0};             // path arc length relative to the near mouth
  double e_lat{0.0};             // robot left of the path, m
  double e_yaw{0.0};             // robot yaw - reference, rad
  double v_ref{0.0};
  double lock{0.0};
};

class TunnelTracker
{
public:
  void setTunnels(const std::vector<Tunnel> & t)
  {
    tunnels_ = t;
    crossings_.clear();
  }
  void setParams(const TrackerParams & p) {prm_ = p;}
  const TrackerParams & params() const {return prm_;}
  void setPath(const std::vector<Pose2> & path);
  void resetProgress() {have_nearest_ = false;}
  size_t crossingCount() const {return crossings_.size();}
  TrackerOutput compute(const Pose2 & robot, double v_cap = std::numeric_limits<double>::infinity());

private:
  struct Crossing
  {
    size_t tunnel{0};
    double s0{0.0}, s1{0.0};   // arc length of the first / last path pose inside the body
    int dir{0};
    bool locked{false};
    double lock{0.0}, entry_yaw{0.0}, entry_s{0.0};
  };
  std::vector<Tunnel> tunnels_;
  std::vector<Pose2> path_;
  std::vector<double> s_;
  std::vector<Crossing> crossings_;
  TrackerParams prm_;
  size_t nearest_{0};
  bool have_nearest_{false};
};

}  // namespace nav_behaviors
