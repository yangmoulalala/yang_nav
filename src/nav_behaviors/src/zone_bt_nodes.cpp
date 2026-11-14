// BT nodes loaded by bt_navigator (plugin_lib_names) for zone-aware navigation.
//
// ZoneReplanPaused: keeps the shaped path while the controller is inside a zone stretch.
// ZoneRecovery:     escalating recovery ladder that does not move the robot inside a zone.
//
// Both are registered here, in one translation unit: BT_REGISTER_NODES defines a single
// BT_RegisterNodesFromPlugin symbol per library, so it may appear only once in libzone_bt_nodes.
#include "behaviortree_cpp/bt_factory.h"
#include "nav_behaviors/zone_recovery.hpp"
#include "nav_behaviors/goal_pending.hpp"
#include "nav_behaviors/zone_path_handoff.hpp"
#include "nav_behaviors/zone_replan_paused.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav_behaviors::PathReachesGoal>("PathReachesGoal");
  factory.registerNodeType<nav_behaviors::KeepOccupiedGoalPending>("KeepOccupiedGoalPending");
  factory.registerNodeType<nav_behaviors::ZoneReplanPaused>("ZoneReplanPaused");
  factory.registerNodeType<nav_behaviors::ZonePathBlocked>("ZonePathBlocked");
  factory.registerNodeType<nav_behaviors::ZoneRecovery>("ZoneRecovery");
  factory.registerNodeType<nav_behaviors::CapturePathGoal>("CapturePathGoal");
  factory.registerNodeType<nav_behaviors::CommitZonePath>("CommitZonePath");
  factory.registerNodeType<nav_behaviors::PathForGoal>("PathForGoal");
  factory.registerNodeType<nav_behaviors::AwaitGoalPath>("AwaitGoalPath");
}
