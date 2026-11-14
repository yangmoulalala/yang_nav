#!/bin/bash
# Full-stack navigation run in the RMUC sim: gz + localization (dual, prior map) + navigation,
# then one or more goals in world coordinates.  Logs and a gz ground-truth trace go to $OUT.
#
#   tools/nav_regress/run_nav.sh "X Y YAW" ["X Y YAW" ...]
#
# env: OUT (default /tmp/nav_run_<timestamp>), MAP (default maps/sim), GUI=true
WS=/home/yang/yang_nav
OUT=${OUT:-/tmp/nav_run_$(date +%Y%m%d_%H%M%S)}; MAP=${MAP:-$WS/maps/sim}; GUI=${GUI:-false}
# TERRAIN: nav_terrain package (terrain_export.py output) used as terrain_dir and for goal registration.
# It lives beside the prior map (maps/sim) or in its own export folder (maps/sim/terrain).
TERRAIN=${TERRAIN:-$(python3 "$WS/tools/nav_regress/terrain_path.py" "$MAP")}
mkdir -p "$OUT"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-78}
export GZ_PARTITION=${GZ_PARTITION:-nav_regress_$$}
set -o pipefail
export MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}
# RVIZ=true opens the navigation RViz (WSLg); HOLD=S keeps everything running S seconds after the goals
RVIZ=${RVIZ:-false}; HOLD=${HOLD:-0}
# UWB=true (default): start the referee UWB sim (uwb_sim). The backend uses UWB only as the
# position prior for SUSPECT/LOST recovery (uwb.enabled in bringup/config/sim/backend*.yaml);
# without it the backend waits in SUSPECT/LOST instead of recovering
UWB=${UWB:-true}
export DISPLAY=${DISPLAY:-:0}
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
set -u   # after sourcing: ROS setup scripts read unset variables

# Refuse to start on top of a leftover stack.  A stale Nav2 from an earlier run keeps
# publishing /nav/obstacles, /cmd_vel and the costmaps; two stacks fighting over the same
# topics look exactly like a navigation bug (robot wandering, goals "no valid path found",
# obstacle marks that the current configuration cannot produce).  One run at a time.
stale=$(ps -eo pid,cmd \
  | grep -E 'nav2_(controller|planner|bt_navigator|collision_monitor|lifecycle)|cmd_gate|gz sim' \
  | grep -v grep || true)
if [ -n "$stale" ]; then
  echo 'ABORT: a navigation/simulation stack is already running; kill it first:'
  echo "$stale" | sed 's/^/  /'
  echo '  Stop the conflicting run through its owning terminal or recorded process group.'
  exit 3
fi

# Every node started by ros2 launch carries --ros-args, so matching it also catches the
# helpers the old list missed (uwb_sim, joint_state_publisher, static_transform_publisher):
# leftover uwb_sim copies all published /uwb_pose and kept the backend from resolving the
# mirrored pose of the point-symmetric field.
PGIDS=()
start() {
  local var=$1 log=$2; shift 2
  setsid "$@" >> "$OUT/$log" 2>&1 &
  printf -v "$var" '%s' "$!"
  PGIDS+=("$!")
}
stop_group() {
  [ -n "$1" ] || return 0
  kill -INT -- -"$1" 2>/dev/null || true
  sleep 2
  kill -TERM -- -"$1" 2>/dev/null || true
}
cleanup() {
  for g in "${PGIDS[@]}"; do kill -INT -- -"$g" 2>/dev/null || true; done
  sleep 3
  for g in "${PGIDS[@]}"; do kill -TERM -- -"$g" 2>/dev/null || true; done
}
TRUTH=; NAV=; LOC=; SIM=
trap cleanup EXIT
trap 'exit 130' INT TERM

start SIM sim.log ros2 launch sentry_gazebo sim.launch.py arena:=rmuc uwb:=$UWB gui:=$GUI
for i in $(seq 1 60); do gz topic -l 2>/dev/null | grep -q /model/sentry/pose && break; sleep 1; done
sleep 3
# Ground truth at 20 Hz including yaw: a tunnel body is 0.35 m, i.e. 0.6 s at 0.6 m/s, so a 2 Hz
# trace left one sample per tunnel and could not measure the in-body error.
start TRUTH truth.log bash -c 'gz topic -e -t /model/sentry/pose | python3 -u "$1/gz_truth.py"' bash "$WS/tools/nav_regress"

start LOC loc.log ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$MAP"
# Do not start navigation on top of a localization that never came up: without
# lio_odom the local costmap times out and lifecycle_manager aborts the whole bringup.
LOC_OK=false
for i in $(seq 1 90); do
  if timeout 3 ros2 topic echo --once /backend_node/localization_state std_msgs/msg/String 2>/dev/null \
      | grep -q 'pose_trusted=true'; then LOC_OK=true; break; fi
  sleep 1
done
echo "localization: $(timeout 3 ros2 topic echo --once /backend_node/localization_state 2>/dev/null | head -1)"
echo "zlio resets during startup: $(grep -c 'IMU input interrupted' "$OUT/loc.log")"
if [ "$LOC_OK" != true ]; then
  # localization fault (see docs/localization_issues.md L5), not a navigation result: restart it once
  echo '=== localization not trusted at startup, restarting localization once'
  stop_group "$LOC"
  start LOC loc.log ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$MAP"
  for i in $(seq 1 90); do
    if timeout 3 ros2 topic echo --once /backend_node/localization_state std_msgs/msg/String 2>/dev/null \
        | grep -q 'pose_trusted=true'; then LOC_OK=true; break; fi
    sleep 1
  done
fi
if [ "$LOC_OK" != true ]; then echo 'ABORT: localization never reported pose_trusted=true'; exit 2; fi
# TF appears a moment after the state goes trusted.  Do not pipe tf2_echo into `grep -q`: under
# `set -o pipefail` grep's early exit SIGPIPEs tf2_echo (141) and the pipeline "fails" although the
# transform was found; python also block-buffers its output when piped, so a short probe can print
# nothing at all.  Capture to a file and grep that.
TF_OK=false
for i in $(seq 1 20); do
  PYTHONUNBUFFERED=1 timeout 8 ros2 run tf2_ros tf2_echo map base_link > "$OUT/tf_probe.log" 2>/dev/null || true
  if grep -q 'Translation' "$OUT/tf_probe.log"; then TF_OK=true; break; fi
  sleep 1
done
if [ "$TF_OK" != true ]; then echo 'ABORT: TF map -> lio_odom -> base_link not available'; exit 2; fi
echo 'TF map -> base_link ok'

start NAV nav.log ros2 launch nav_bringup navigation.launch.py terrain_dir:="$TERRAIN" rviz:=false params_file:="/home/yang/yang_nav/reports/navigation/2026-11-13/navigation_fixes/scripts/trace_nav2.yaml"
for i in $(seq 1 60); do grep -q 'Managed nodes are active' "$OUT/nav.log" && break; sleep 1; done
grep -q 'Managed nodes are active' "$OUT/nav.log" || { echo 'nav2 did not come up'; tail -40 "$OUT/nav.log"; exit 1; }
echo 'nav2 active'
start CHAIN chain.log python3 "/home/yang/yang_nav/reports/navigation/2026-11-13/navigation_fixes/scripts/record_yaw_chain.py" "$OUT"
# OBS_REC=path records /nav/obstacles (terrain_obstacles output) + message stamps for offline
# analysis of what the obstacle filter marks (tools/nav_regress/record_obs.py)
if [ -n "${OBS_REC:-}" ]; then
  start OBS obs_rec.log python3 "$WS/tools/nav_regress/record_obs.py" "$OBS_REC" "${OBS_REC_SECS:-3600}"
  echo "recording /nav/obstacles to $OBS_REC"
fi
# CM_REC=dir records costmap snapshots (record_costmap.py, one npz per snapshot): offline
# analysis of which cells the obstacle layer actually marked during a run
if [ -n "${CM_REC:-}" ]; then
  start CMS cm_rec.log python3 "$WS/tools/nav_regress/record_costmap.py" "$CM_REC" "${CM_REC_SECS:-3600}" \
    /global_costmap/costmap /local_costmap/costmap
  echo "recording costmap snapshots to $CM_REC"
fi
# PLAN_REC=file records /plan and /local_plan: what the planner actually sent the controller.
# Without it a run where the robot drives to the wrong part of the field cannot be explained -
# goals.log only shows the goal that was requested, not the path that was followed.
if [ -n "${PLAN_REC:-}" ]; then
  start PLAN plan_rec.log python3 "$WS/tools/nav_regress/record_plan.py" "$PLAN_REC" "${PLAN_REC_SECS:-3600}"
  echo "recording /plan to $PLAN_REC"
fi
# RViz starts only now, after localization and Nav2 are up, to keep the WSLg GPU load
# off the startup window where ZLIO is most likely to miss IMU samples.
if [ "$RVIZ" = true ]; then
  sleep 5
  start RVIZ_PID rviz.log ros2 run rviz2 rviz2 -d "$WS/install/nav_bringup/share/nav_bringup/rviz/navigation.rviz" \
    --ros-args -p use_sim_time:=true
  echo 'rviz started'
fi
# ZLIO resets when it misses >0.1 s of IMU (seen while many processes start under WSL);
# wait until localization is trusted again before every goal
wait_trusted() {
  for i in $(seq 1 90); do
    timeout 3 ros2 topic echo --once /backend_node/localization_state std_msgs/msg/String 2>/dev/null \
      | grep -q 'pose_trusted=true' && return 0; sleep 1
  done; return 1
}
rc=0; CONSEC=0
for g in "$@"; do
  sleep 3
  if ! wait_trusted && [ "${LOC_RESTARTS:-0}" -lt 2 ]; then
    # The backend can stay LOST (tf_frozen) for good after a ZLIO IMU-gap reset, e.g. under the
    # CPU spike of the Nav2 bringup. That is a localization issue, not navigation: restart the
    # localization stack once more so the navigation test can still run.
    LOC_RESTARTS=$((${LOC_RESTARTS:-0}+1))
    echo "=== restarting localization ($LOC_RESTARTS): backend not trusted"
    stop_group "$LOC"
    start LOC loc.log ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$MAP"
  fi
  if ! wait_trusted; then
    # not a navigation failure: the backend is LOST (tf frozen), so the robot cannot move
    echo "=== goal (world) $g   SKIPPED: localization not trusted (resets so far: $(grep -c 'IMU input interrupted' "$OUT/loc.log"))"
    echo "goal skipped $g: localization not trusted" >> "$OUT/goals.log"
    rc=1; LOCFAIL=$((${LOCFAIL:-0}+1))
    if [ "$LOCFAIL" -ge 2 ]; then echo 'ABORT: localization lost, not a navigation result'; break; fi
    continue
  fi
  echo "=== goal (world) $g   resets so far: $(grep -c 'IMU input interrupted' "$OUT/loc.log")"
  python3 "$WS/tools/nav_regress/goal.py" $g --frame ${GOAL_FRAME:-world} --nav-dir "$TERRAIN" --timeout "${GOAL_TIMEOUT:-180}" | tee -a "$OUT/goals.log"
  if [ "${PIPESTATUS[0]}" = 0 ]; then CONSEC=0; else rc=1; CONSEC=$((CONSEC+1)); fi
  echo "truth: $(tail -1 "$OUT/truth.log")"

done
if [ "$HOLD" -gt 0 ]; then echo "holding $HOLD s for inspection"; sleep "$HOLD"; fi
exit $rc
