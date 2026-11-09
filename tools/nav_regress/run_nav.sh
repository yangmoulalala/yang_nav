#!/bin/bash
# Full-stack navigation run in the RMUC sim: gz + localization (dual, prior map) + navigation,
# then one or more goals in world coordinates.  Logs and a gz ground-truth trace go to $OUT.
#
#   tools/nav_tests/run_nav.sh "X Y YAW" ["X Y YAW" ...]
#
# env: OUT (default /tmp/nav_run), MAP (default maps/sim-rmuc-dual), GUI=true
WS=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${OUT:-/tmp/nav_run}; MAP=${MAP:-$WS/maps/sim}; GUI=${GUI:-false}
# TERRAIN: nav_terrain package (terrain_export.py output) used as terrain_dir and for goal registration
TERRAIN=${TERRAIN:-$MAP}
mkdir -p "$OUT"; rm -f "$OUT"/*.log
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
# Every node started by ros2 launch carries --ros-args, so matching it also catches the
# helpers the old list missed (uwb_sim, joint_state_publisher, static_transform_publisher):
# leftover uwb_sim copies all published /uwb_pose and kept the backend from resolving the
# mirrored pose of the point-symmetric field.
cleanup() {
  kill $TRUTH $NAV $LOC $SIM 2>/dev/null; sleep 2
  pkill -9 -f 'ros2 launch'; pkill -9 -f 'gz sim'; pkill -9 -f 'gz topic'
  pkill -9 -f -- '--ros-args'
}
PLANLOG=; TRUTH=; NAV=; LOC=; SIM=
cleanup >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1
if pgrep -f -- '--ros-args|gz sim' >/dev/null; then
  echo 'stale ROS / gz processes still running:'; pgrep -af -- '--ros-args|gz sim' | cut -c1-120; exit 1
fi
trap cleanup EXIT

ros2 launch sentry_gazebo sim.launch.py arena:=rmuc uwb:=$UWB gui:=$GUI > "$OUT/sim.log" 2>&1 & SIM=$!
for i in $(seq 1 60); do gz topic -l 2>/dev/null | grep -q /model/sentry/pose && break; sleep 1; done
sleep 3
( gz topic -e -t /model/sentry/pose | python3 -u -c '
import sys,re,time
buf=""; last=0
for line in sys.stdin:
    buf+=line
    if line.startswith("}") or "header" in line and buf.count("position")>0:
        m=re.search(r"position \{(.*?)\}",buf,re.S)
        if m and time.time()-last>0.5:
            g=lambda k: float(re.search(k+r": ([-0-9.e]+)",m.group(1)).group(1)) if re.search(k+r": ",m.group(1)) else 0.0
            print("%.1f %.3f %.3f %.3f"%(time.time(),g("x"),g("y"),g("z")),flush=True); last=time.time()
        buf=""
' > "$OUT/truth.log" 2>/dev/null ) & TRUTH=$!

ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$MAP" > "$OUT/loc.log" 2>&1 & LOC=$!
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
  kill $LOC 2>/dev/null; sleep 2
  pkill -9 -f 'bringup sim.launch.py'; pkill -9 -f zlio_node; pkill -9 -f backend_node; pkill -9 -f merge_node
  sleep 2
  ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$MAP" >> "$OUT/loc.log" 2>&1 & LOC=$!
  for i in $(seq 1 90); do
    if timeout 3 ros2 topic echo --once /backend_node/localization_state std_msgs/msg/String 2>/dev/null \
        | grep -q 'pose_trusted=true'; then LOC_OK=true; break; fi
    sleep 1
  done
fi
if [ "$LOC_OK" != true ]; then echo 'ABORT: localization never reported pose_trusted=true'; exit 2; fi
if ! timeout 8 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q 'Translation'; then
  echo 'ABORT: TF map -> lio_odom -> base_link not available'; exit 2
fi
echo 'TF map -> base_link ok'

ros2 launch nav_bringup navigation.launch.py terrain_dir:="$TERRAIN" rviz:=false > "$OUT/nav.log" 2>&1 & NAV=$!
for i in $(seq 1 60); do grep -q 'Managed nodes are active' "$OUT/nav.log" && break; sleep 1; done
grep -q 'Managed nodes are active' "$OUT/nav.log" || { echo 'nav2 did not come up'; tail -40 "$OUT/nav.log"; exit 1; }
echo 'nav2 active'
# RViz starts only now, after localization and Nav2 are up, to keep the WSLg GPU load
# off the startup window where ZLIO is most likely to miss IMU samples.
if [ "$RVIZ" = true ]; then
  sleep 5
  ros2 run rviz2 rviz2 -d "$WS/install/nav_bringup/share/nav_bringup/rviz/navigation.rviz" \
    --ros-args -p use_sim_time:=true > "$OUT/rviz.log" 2>&1 &
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
    kill $LOC 2>/dev/null; sleep 2
    pkill -9 -f 'bringup sim.launch.py'; pkill -9 -f zlio_node; pkill -9 -f backend_node; pkill -9 -f merge_node
    sleep 2
    ros2 launch bringup sim.launch.py lidar:=dual mode:=localization prior_map:="$MAP" >> "$OUT/loc.log" 2>&1 & LOC=$!
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
  python3 "$WS/tools/nav_regress/goal.py" $g --frame ${GOAL_FRAME:-world} --nav-dir "$TERRAIN" | tee -a "$OUT/goals.log"
  if [ "${PIPESTATUS[0]}" = 0 ]; then CONSEC=0; else rc=1; CONSEC=$((CONSEC+1)); fi
  echo "truth: $(tail -1 "$OUT/truth.log")"
  if [ "$CONSEC" -ge 2 ]; then echo "ABORT: 2 consecutive failures, robot likely stuck"; break; fi
done
if [ "$HOLD" -gt 0 ]; then echo "holding $HOLD s for inspection"; sleep "$HOLD"; fi
exit $rc
