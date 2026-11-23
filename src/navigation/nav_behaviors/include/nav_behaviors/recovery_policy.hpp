// Recovery escalation for the behaviour tree (no ROS), unit-tested in test/test_recovery_policy.cpp.
//
// One recovery cycle per navigation failure, escalating instead of repeating the same action:
//
//   attempt 0            ClearCostmaps   drop stale marks, let the sensors refresh; no motion
//   attempt 1            Spin            re-observe by rotating in place
//   attempt 2 .. n-1     BackUp          back away from whatever blocks the path
//   attempt n            GiveUp          retries exhausted, fail the navigation task
//
// The ladder moves the robot only outside a zone.  From ALIGN on the heading is locked and the body
// has centimetre margins, so a spin sweeps the footprint corners into the walls and a back-up drops
// the alignment the shaped path was built for.  Inside a zone the ladder therefore only clears the
// costmaps and waits; if that does not help, GiveUp lets the caller re-task (the controller has
// already replanned from the robot's position, see test_zone_drive ResumeFromInsideTheBody).
#pragma once

namespace nav_behaviors
{

enum class RecoveryAction
{
  ClearCostmaps = 0,
  Spin,
  BackUp,
  GiveUp,
};

inline const char * recoveryActionName(RecoveryAction a)
{
  static const char * names[] = {"clear", "spin", "backup", "give_up"};
  return names[static_cast<int>(a)];
}

struct RecoveryConfig
{
  int retries = 6;        // recovery cycles before GiveUp; 0 fails on the first failure
  int spin_after = 1;     // first attempt that spins
  int backup_after = 2;   // first attempt that backs up
  bool spin = true;
  bool backup = true;
};

// `attempt` is 0 on the first failure of the current navigation task, `in_zone` is true while the
// controller reported ALIGN / TRAVERSE / EXIT recently (phaseBlocksRecovery).
RecoveryAction recoveryAction(const RecoveryConfig & c, int attempt, bool in_zone);

}  // namespace nav_behaviors
