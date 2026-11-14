// Zone passage phases, shared by the tracker, the controller and the behaviour-tree condition (no ROS).
//
//   None      outside every zone window
//   Approach  window entered, still free to replan; MPPI and the tracker are mixed
//   Align     heading locked on the zone axis; from here the tracker alone owns the robot
//   Traverse  inside the zone body
//   Exit      past the far mouth, blending back to the inner controller
//
// The controller publishes stamped phases on /nav_zone_state ("traverse tunnel_3 @123.4"); the BT
// condition keeps the current path while the phase is Align / Traverse / Exit, so a replan can never
// replace the shaped path half way through a tunnel or up a ramp.
#pragma once

#include <string>

namespace nav_behaviors
{

enum class ZonePhase { None = 0, Approach = 1, Align = 2, Traverse = 3, Exit = 4, Blocked = 5 };

inline const char * zonePhaseName(ZonePhase p)
{
  static const char * names[] = {"none", "approach", "align", "traverse", "exit", "blocked"};
  return names[static_cast<int>(p)];
}

// Phase from the tracker output: `s_rel` measured from the near mouth, `body` the body arc length.
ZonePhase zonePhaseOf(double s_rel, double body, double align_before);

// "traverse tunnel_3 @123.45" -> Traverse (stamp 123.45 s); unknown / empty -> None, no stamp -> 0.
ZonePhase parseZonePhase(const std::string & text, double * stamp = nullptr);

// APPROACH remains replannable. ALIGN/TRAVERSE/EXIT retain the shaped path.
inline bool phasePausesReplan(ZonePhase p)
{
  return p == ZonePhase::Align || p == ZonePhase::Traverse || p == ZonePhase::Exit;
}

// Recovery must not move the robot from ALIGN on.  APPROACH is still free: the robot is outside the
// opening and the mixed controller can be backed away from a bad line-up.  Inside, the heading is
// locked and the body has centimetre margins, so a spin sweeps the footprint corners into the walls
// and a back-up throws away the alignment the shaped path was built for.
inline bool phaseBlocksRecovery(ZonePhase p)
{
  return phasePausesReplan(p) || p == ZonePhase::Blocked;
}

// Bookkeeping of the newest /nav_zone_state sample (pure, unit-tested in test_zone_state.cpp).
//
// The BT is ticked only now and then, so it cannot judge freshness by when it processes a message:
// the stamp inside the message is the controller clock and is compared against the same clock.  A
// sample without a stamp is ignored (freshness unknown) and an out-of-order replay is dropped, so a
// latched sample of an aborted run never looks fresh.
struct PhaseMemory
{
  bool have_phase{false}, paused{false}, have_block{false}, path_blocked{false};
  double phase_stamp{0.0}, block_stamp{0.0};

  void update(ZonePhase p, double stamp);

  bool fresh(double now, double timeout) const;
  bool pausedNow(double now, double timeout) const;
  // ALIGN / TRAVERSE / EXIT within `timeout`: no spin, no back-up
  bool blockedRecently(double now, double timeout) const;
};

}  // namespace nav_behaviors
