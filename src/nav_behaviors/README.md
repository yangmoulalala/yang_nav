# nav_behaviors

Nav2 plugins for following paths through annotated, direction-constrained terrain. The package keeps one shared passage pipeline; terrain labels provide geometry policy and select a small set of motion profiles.

## Design

```text
ZoneRecovery (BT root)
  ├─ navigation: plan → ZonePathShaper → ZoneAwareController (MPPI + ZoneTracker)
  │               ZoneReplanPaused retains the shaped path during ALIGN/TRAVERSE/EXIT
  └─ recovery: clear → spin → back up → give up, with physical/phase no-motion protection
```

The pure geometry and tracking code has no ROS dependencies. Nav2-specific lifecycle, costmap, TF, topic, and Behavior Tree integration stays in thin plugin adapters.

## Modules

| Module | Responsibility |
| --- | --- |
| `zones_yaml` | Load the terrain package's zone geometry and traversal policy. |
| `path_shaper` | Detect zone crossings, join adjacent zones, shape smooth axis-aligned paths, and reject colliding variants. |
| `start_aware_planner` | Prepend a fixed-heading escape from an occupied/inflated start, then invoke Smac2D without modifying the map. |
| `overlap_escape` | Bounded, non-growing footprint overlap with the original obstacle component; unknown/new obstacles remain blocked. |
| `zone_path_handoff` | Bind a committed shaped path to its goal, preventing raw-path leakage and stale-goal execution. |
| `zone_tracker` | Track the shaped path, manage the shared approach/align/traverse/exit phases, and calculate velocity commands. |
| `zone_state` | Shared phase definitions, parsing and the stamped phase memory (`PhaseMemory`) used by the BT nodes. |
| `recovery_policy` | Pure recovery ladder: clear costmaps -> spin -> back up -> give up, no motion inside a zone. |
| `zone_path_shaper` | Nav2 smoother adapter: TF start heading, costmap collision checks, and path conversion. |
| `zone_aware_controller` | Nav2 controller adapter: MPPI handoff, tracker commands, dynamic obstacle stop, mode/state and RViz output. |
| `zone_replan_paused` | BT condition that blocks path replacement while the tracker owns the crossing. |
| `zone_recovery` | BT control node that ticks one recovery child per failure and fails the goal when the ladder is exhausted. |

## Recovery

The root of both `navigate_to_pose.xml` and `navigate_through_poses.xml` is `ZoneRecovery` with five children: the navigation subtree, clear
costmaps, spin, back up, give up. `recovery_policy` picks one child per navigation failure, so
retries escalate instead of repeating the same action; `attempt` resets in `halt()`, i.e. per goal.

Before motion recovery, the tree checks robot TF against the configured zones with a 0.55 m
margin. Physical occupancy, missing/stale TF, or a recent ALIGN / TRAVERSE / EXIT phase inhibits
spin and back-up. The inhibition latches for the failure sequence, so phase expiry while stopped
cannot enable motion. A running motion recovery is interrupted if protection becomes active.
The tree clears costmaps and eventually reports failure; a new goal interrupts the old subtree
and resets its recovery ladder. Both single-goal and through-poses trees use this protection.

## Zone policy

`terrain_export.py` writes one `zones.yaml` record per connected directional region. Its `type` is the semantic terrain label. The following fields describe traversal independently of the label:

| Field | Meaning |
| --- | --- |
| `centre`, `yaw_deg`, `size` | Oriented zone box; yaw is its travel axis (for slopes and steps, positive yaw points uphill). |
| `one_way_dir` | `0` shapes either direction; `1` or `-1` selects that signed axis for shaping/tracking. Opposite-direction descent currently stays with ordinary planning/MPPI. |
| `allow_flip` | Whether the shaper may choose either physical heading along the axis. The tracker retains the collision-checked path heading. |
| `recentre` | Whether the shaped path should move to the zone centerline. |
| `warning` | Ambiguous directions disable axis locking instead of choosing an arbitrary average. |

Defaults are exported explicitly: tunnels and undulations are bidirectional; slopes and steps select their annotated positive direction for the zone tracker; tunnels are centered while open terrain retains the planner's lateral line. Old zone files without policy fields receive the same defaults in the loader.

The semantic type selects tuning: `tunnel` uses the narrow-passage speed and footprint margins; other configured directional terrain uses the open-terrain (ramp) profile. Both use the same shaping and tracking algorithms. Add a new terrain label by exporting its policy and selecting it in the plugin `types` parameter; do not add another controller state machine.

## Nav2 parameters

Both the smoother and controller load the same `zones.yaml` and must use matching type lists and chain parameters. The shipped configuration enables:

```yaml
types: [tunnel, slope, step_l1, step_l2, step_high, fly_slope, undulation]
```

The controller's `zone.*` parameters provide two tuning profiles (`tunnel_*` and `ramp_*`), shared alignment gains, collision margins, and chaining thresholds. `/nav_zone_state` reports the current phase; `/nav_mode` applies the matching `cmd_gate` speed backstop.

## Occupied starts and exit replanning

An occupied start does not immediately abort navigation. The planner finds a translation-only
escape (default at most 1.5 m), and the controller executes it at at most 0.15 m/s before normal
tracking resumes. The smoother preserves and validates that prefix. Overlap area with the
original connected obstacle component may stay constant or shrink; unrelated/new obstacle cells,
unknown space and map bounds are never exempted. The controller freezes that component for the
whole episode. The live collision monitor and localization/observation gates remain enabled.
This handles map overlap, not a guarantee that a physically embedded chassis can move.
An escape failure publishes `blocked` so the BT can replan; a newly committed shaped path
replaces the previous escape target and allowance. A changed goal clears tracker progress.

A path starting just beyond a zone's far mouth still creates an EXIT crossing. The tracker
holds the body heading while the complete padded rear clears the zone, including after a cold
restart or replanning that removed every body sample.

Both BTs capture the planning target, keep raw and shaped paths separate, and commit only a
successfully shaped path for the same target. A new goal waits for its own committed path.

## Tests and build

The smoother and zone controller check the complete padded convex footprint, including occupied
cells inside its perimeter. Motion checks interpolate translation and rotation at half-cell steps;
unknown and out-of-map space are rejected. `collision_tolerance` must be zero. A required shaping
failure propagates to Nav2 instead of handing an unshaped path to the tracker. Tracking also checks
the measured and commanded braking trajectories, including lateral motion, rotation and reversing.

Core tests cover path shaping, direction policies, tracking, state transitions, footprint collision,
closed-loop zone traversals, and recovery inside a real behaviour tree (`test_zone_recovery`).
The integration tools in `tools/nav_regress` run real Nav2 nodes with deterministic test inputs.
See the [navigation plan](../../docs/nav_plan.md) and [regression tools](../../tools/nav_regress/README.md)
for current evidence and reproduction commands: 13 integration checks passed in the audit,
but only 8/12 real-map tunnel paths currently pass strict shaping. Ramp physics and full-map
traversability remain unverified; the kinematic fixture does not establish physical clearance.

Build the package with:

```bash
colcon build --packages-select nav_behaviors
```

## Removed legacy interfaces (2026-11-13)

`ZoneReplanPaused` now uses only the latest fresh phase. Its unused `last_phase` port and duplicate
pause-history storage were removed; `ZoneRecovery` retains the separate recent-block history and
physical occupancy interlock. The shipped trees did not use the removed port. Custom trees should
use `ZoneRecovery` for recovery protection instead of requesting a historical replan condition.

The geometry core no longer has a `start_skip` collision bypass. Every candidate pose, including
its actual start, must pass the supplied collision checker. The production smoother already used
zero skip; this also makes offline callers follow the same collision contract.

Unused `parseZoneId`, `phaseMemory`, `body_length`, and the `Zone` height-range copy/aggregation
were removed. Terrain exports may still contain `z_range`; it remains source metadata and is not a
2D path-shaping constraint. Elevation and clearance used by obstacle filtering remain unchanged.
