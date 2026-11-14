#!/bin/bash
# Planner-only check of the tunnel acceptance goals: nav_map_server + planner_server (global costmap)
# + a static map -> base_link TF, then tools/nav_regress/chaincheck.py plans every leg.
#
#   tools/nav_regress/chaincheck.sh [TERRAIN_DIR] [chaincheck.py args]     (default maps/sim)
#
# No simulator and no localization: it only answers "does the planned path really go through the
# tunnel?" - the failure mode that would otherwise look like a 12/12 pass in Gazebo.
WS=$(cd "$(dirname "$0")/../.." && pwd)
TERRAIN=${1:-$(python3 "$WS/tools/nav_regress/terrain_path.py")}
OUT=${OUT:-/tmp/chaincheck}; mkdir -p "$OUT"
PARAMS=${PARAMS:-$WS/src/nav_bringup/params/sim/nav2.yaml}
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-78}   # keep away from a running sim
PGIDS=()
start() { local log=$1; shift; setsid "$@" > "$OUT/$log" 2>&1 & PGIDS+=($!); }
cleanup() {
  for g in "${PGIDS[@]}"; do kill -- -"$g" 2>/dev/null; done; sleep 1
  for g in "${PGIDS[@]}"; do kill -9 -- -"$g" 2>/dev/null; done
}
trap cleanup EXIT
if pgrep -f '__node:=lm_chain' >/dev/null; then
  echo 'a previous chaincheck run is still alive:'; pgrep -af '__node:=lm_chain' | cut -c1-120; exit 2
fi
start tf.log ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link
start map.log ros2 run nav2_map_server map_server --ros-args -r __node:=nav_map_server --params-file "$PARAMS" \
  -p yaml_filename:="$TERRAIN/nav_map.yaml" -p use_sim_time:=false
start planner.log ros2 run nav2_planner planner_server --ros-args --params-file "$PARAMS" -p use_sim_time:=false
start lm.log ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lm_chain -p autostart:=true \
  -p node_names:="['nav_map_server','planner_server']" -p bond_timeout:=0.0
for i in $(seq 1 40); do grep -q 'Managed nodes are active' "$OUT/lm.log" && break; sleep 0.5; done
if ! grep -q 'Managed nodes are active' "$OUT/lm.log"; then
  echo 'planner did not come up'; tail -n 20 "$OUT"/*.log; exit 2
fi
sleep 3   # global costmap: first static map + inflation
python3 "$WS/tools/nav_regress/chaincheck.py" --terrain-dir "$TERRAIN" "${@:2}"
exit $?
