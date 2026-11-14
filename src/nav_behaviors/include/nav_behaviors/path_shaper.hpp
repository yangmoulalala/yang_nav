// Direction-constrained zone path shaping. Geometry is ROS-independent for reuse and offline testing.
//
// Every stretch of a planner path that crosses a zone is replaced by
//   lead-in       cubic Bezier from the planner path onto the axis, `lead_length` of path before the axis
//   entry margin  `axis_margin` along the axis up to the mouth
//   body          the zone itself, straight along the axis
//   exit margin   `axis_margin` along the axis after the far mouth
//   lead-out      cubic Bezier from the axis back onto the planner path
// The joints are tangent-continuous (G1).  The axis comes from the zone centre and yaw, never from the grid.
//
// Nearby zones with compatible axes are shaped as one stretch according to ChainParams:
// their margins would otherwise run into each
// other, and a robot leaving a tunnel onto the ramp behind it must not turn between them.  A chain is
// shaped along the narrowest member's axis, its body running from the first near mouth to the last far one.
//
// Headings are written into the poses.  The locked heading is the zone yaw or yaw + pi, whichever is closer
// to the approach (the long axis of the robot lies along a tunnel); margins, body and lead-out keep it
// (omni chassis: the controller turns back after the stretch).  A slope is different: its yaw is the
// uphill azimuth and the robot must charge it nose first, so the heading is the uphill heading with no
// 180 deg flip.  The lead-in turns from the smoothed path tangent to the locked heading over the whole lead
// (Full), over its first half (Half) or not at all (Locked: the robot is already aligned when the stretch
// starts) - narrow corridors before a mouth only fit an aligned robot.
//
// Traversal policy comes from each Zone: signed-direction zones leave opposite-direction descent
// to the planner/MPPI; allow_flip selects the
// permitted body heading, and recentre chooses axis-centering versus preserving the planner's lateral line.
//
// Lateral offsets (zone frame, m): the body may sit up to `max_body_offset` off the axis, each margin
// end up to `max_side_offset`, joined by smoothstep ramps.  The search takes the smallest offsets and the
// latest turn that pass the collision check, so a clear map gives v = 0 and a Full turn.  One-sided walls
// that narrow the approach (cloud maps) are cleared by shifting away from them.  If nothing fits, the lead
// grows and the search repeats. The pure core keeps failed segments and records ShapeEvent reasons;
// the ROS adapter rejects required shaping failures instead of reporting the original path as safe.
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

constexpr double kPi = 3.14159265358979323846;

// Physical tuning profile, deliberately separate from the semantic terrain label in zones.yaml.
enum class ZoneProfile { NarrowPassage = 0, OpenDirectional = 1 };

const char * zoneProfileName(ZoneProfile profile);
const char * zoneModeName(ZoneProfile profile);   // "tunnel" / "ramp" (/nav_mode backstop)

struct Zone
{
  std::string id;
  ZoneProfile profile{ZoneProfile::NarrowPassage};
  double cx{0.0}, cy{0.0};
  double yaw{0.0};     // travel axis; for an annotated slope, the positive axis points uphill
  double depth{0.0};   // along yaw, m
  double width{0.0};   // across yaw, m
  int chain{1};               // zones merged into this stretch (see ChainParams)
  int one_way_dir{0};         // 0 = track both ways; signed direction selects zone shaping/tracking
  bool allow_flip{true};      // tunnel: yaw or yaw + pi, whichever is closer; slope: uphill only
  bool recentre{true};        // tunnel: shape onto the axis; slope: keep the path's lateral offset
  bool axis_reliable{true};   // false when the export saw several uphill directions in one patch

  bool one_way() const {return one_way_dir != 0;}
  void toLocal(double x, double y, double & u, double & v) const;
  Pose2 toMap(double u, double v, double yaw_rel) const;
};

// Zones closer than `gap` along the path, with parallel axes less than `lat` apart, are one stretch.
struct ChainParams
{
  double gap{1.0};                            // m along the axis between the boxes
  double angle{12.0 * kPi / 180.0};           // axes within this of parallel
  double lat{0.30};                           // axes this close side by side, m
};

// One synthetic zone per chain (same order as the first member); ids joined with '+'.
std::vector<Zone> groupZones(const std::vector<Zone> & zones, const ChainParams & chain);

enum class TurnMode { Full, Half, Locked };
const char * turnName(TurnMode m);

struct ShaperParams
{
  double axis_margin{0.5};       // straight axis beyond each mouth, m
  double exit_clearance{0.44};   // fore/aft padded footprint extent + 5 cm; derived by the ROS plugin
  double lead_length{1.5};       // planner path replaced by each lead curve, m
  double lead_growth{1.5};       // lead multiplier per retry when nothing fits
  int max_retries{2};
  double spacing{0.025};         // pose spacing of the shaped stretch, m
  double max_body_offset{0.03};  // lateral offset allowed inside a tunnel, m
  double max_side_offset{0.06};  // lateral offset allowed at the outer end of each margin, m
  double offset_step{0.01};
  double tangent_window{0.25};   // planner tangent at the lead ends averaged over +- this arc length, m
  double early_turn{0.3};        // lead-in from a nearly aligned robot: turn finished within this arc, m
  double early_turn_max_yaw{0.5};   // ... when the robot is at most this far off the locked heading, rad
  double slope_lead_length{1.2};    // slopes are open: a shorter lead is enough
  double slope_axis_margin{0.3};
  double slope_max_body_offset{0.06};   // the path's own line is kept; this much shift around it
  ChainParams chain;
};

struct ShapeEvent
{
  std::string zone_id;    // "tunnel_3", or "tunnel_3+slope_3" for a chain
  ZoneProfile profile{ZoneProfile::NarrowPassage};
  int chain{1};           // zones shaped as one stretch
  int direction{0};       // +1 along the zone yaw, -1 against it
  bool shaped{false};
  double heading{0.0};    // locked heading on the axis
  double lead{0.0};       // lead length used
  double v_in{0.0}, v_body{0.0}, v_out{0.0};   // lateral offsets used (zone frame)
  TurnMode turn{TurnMode::Full};
  std::string reason;     // why the planner path was kept
};

using CollisionFree = std::function<bool(const Pose2 &)>;

extern const char * const kCollisionReason;

// `start_yaw`: the robot heading when the path starts at the robot.  A lead-in that begins at the path start
// turns from it (not from the planner tangent), so the collision check covers the poses the robot will
// really take - near a mouth a diagonal tangent heading swept the footprint corner into the wall.
std::vector<Pose2> shapePath(
  const std::vector<Pose2> & path, const std::vector<Zone> & zones, const ShaperParams & params,
  const CollisionFree & collision_free, std::vector<ShapeEvent> * events = nullptr,
  const double * start_yaw = nullptr);

}  // namespace nav_behaviors
