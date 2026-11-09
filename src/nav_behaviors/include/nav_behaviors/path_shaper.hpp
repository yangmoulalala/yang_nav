// Tunnel path shaping (M3b), no ROS dependency so it can be unit tested offline.
//
// Every stretch of a planner path that passes through a tunnel zone is replaced by
//   lead-in       cubic Bezier from the planner path onto the axis, `lead_length` of path before the axis
//   entry margin  `axis_margin` along the axis up to the mouth
//   body          the tunnel itself, straight along the axis
//   exit margin   `axis_margin` along the axis after the mouth
//   lead-out      cubic Bezier from the axis back onto the planner path
// The joints are tangent-continuous (G1).  The axis comes from the zone centre and yaw, never from the grid.
//
// Headings are written into the poses.  The locked heading is the zone yaw or yaw + pi, whichever is closer
// to the approach (the long axis of the robot lies along the tunnel); margins, body and lead-out keep it
// (omni chassis: the controller turns back after the stretch).  The lead-in turns from the smoothed path
// tangent to the locked heading over the whole lead (Full), over its first half (Half) or not at all
// (Locked: the robot is already aligned when the stretch starts) - narrow corridors before a mouth only
// fit an aligned robot.
//
// Lateral offsets (tunnel frame, m): the body may sit up to `max_body_offset` off the axis, each margin
// end up to `max_side_offset`, joined by smoothstep ramps.  The search takes the smallest offsets and the
// latest turn that pass the collision check, so a clear map gives v = 0 and a Full turn.  One-sided walls
// that narrow the approach (cloud maps) are cleared by shifting away from them.  If nothing fits, the lead
// grows and the search repeats; if it still fails the planner path is kept and the event says why.
#pragma once

#include <functional>
#include <string>
#include <vector>

namespace nav_behaviors
{

struct Pose2
{
  double x{0.0}, y{0.0}, yaw{0.0};
};

struct Tunnel
{
  std::string id;
  double cx{0.0}, cy{0.0};
  double yaw{0.0};     // passing direction, rad (bidirectional)
  double depth{0.0};   // along yaw, m
  double width{0.0};   // opening across yaw, m

  void toLocal(double x, double y, double & u, double & v) const;
  Pose2 toMap(double u, double v, double yaw_rel) const;
};

enum class TurnMode { Full, Half, Locked };
const char * turnName(TurnMode m);

struct ShaperParams
{
  double axis_margin{0.5};       // straight axis beyond each mouth, m
  double lead_length{1.5};       // planner path replaced by each lead curve, m
  double lead_growth{1.5};       // lead multiplier per retry when nothing fits
  int max_retries{2};
  double spacing{0.025};         // pose spacing of the shaped stretch, m
  double max_body_offset{0.03};  // lateral offset allowed inside the tunnel, m
  double max_side_offset{0.06};  // lateral offset allowed at the outer end of each margin, m
  double offset_step{0.01};
  double tangent_window{0.25};   // planner tangent at the lead ends averaged over +- this arc length, m
};

struct ShapeEvent
{
  std::string tunnel_id;
  int direction{0};       // +1 along the zone yaw, -1 against it
  bool shaped{false};
  double heading{0.0};    // locked heading on the axis
  double lead{0.0};       // lead length used
  double v_in{0.0}, v_body{0.0}, v_out{0.0};   // lateral offsets used (tunnel frame)
  TurnMode turn{TurnMode::Full};
  std::string reason;     // why the planner path was kept
};

using CollisionFree = std::function<bool(const Pose2 &)>;

extern const char * const kCollisionReason;

double normAngle(double a);

std::vector<Pose2> shapePath(
  const std::vector<Pose2> & path, const std::vector<Tunnel> & tunnels, const ShaperParams & params,
  const CollisionFree & collision_free, std::vector<ShapeEvent> * events = nullptr);

}  // namespace nav_behaviors
