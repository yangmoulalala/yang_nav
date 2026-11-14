#!/bin/bash
# Planner-only tunnel check, no simulator: nav_map_server + planner_server (global costmap) + a
# static map -> base_link TF, then plan_tunnels.py plans through every tunnel both ways.
#
#   tools/nav_regress/plan_tunnels.sh [TERRAIN_DIR] [plan_tunnels.py args]     (default maps/sim)
WS=/home/yang/yang_nav
TERRAIN=${1:-$(python3 "$WS/tools/nav_regress/terrain_path.py")}
OUT=${OUT:-/tmp/plan_tunnels}; mkdir -p "$OUT"
PARAMS=${PARAMS:-$WS/src/nav_bringup/params/sim/nav2.yaml}
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}   # keep away from a running sim
# Every node runs in its own session: `ros2 run` does not pass SIGTERM on to the node it starts,
# so killing the wrapper PID left the nodes running and the next run's nav_map_server clashed.
PGIDS=()
start() { local log=$1; shift; setsid "$@" > "$OUT/$log" 2>&1 & PGIDS+=($!); }
cleanup() {
  for g in "${PGIDS[@]}"; do kill -- -"$g" 2>/dev/null; done; sleep 1
  for g in "${PGIDS[@]}"; do kill -9 -- -"$g" 2>/dev/null; done
}
trap cleanup EXIT
if pgrep -f '__node:=lm_plan' >/dev/null; then
  echo 'a previous plan_tunnels run is still alive:'; pgrep -af '__node:=lm_plan' | cut -c1-120; exit 2
fi
start tf.log ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link
start map.log ros2 run nav2_map_server map_server --ros-args -r __node:=nav_map_server --params-file "$PARAMS" \
  -p yaml_filename:="$TERRAIN/nav_map.yaml" -p use_sim_time:=false
start planner.log ros2 run nav2_planner planner_server --ros-args --params-file "$PARAMS" -p use_sim_time:=false
NODES="'nav_map_server','planner_server'"
if [ -n "${SMOOTHER:-}" ]; then   # SMOOTHER=zone_shaper: also run smoother_server and shape every plan
  start smoother.log ros2 run nav2_smoother smoother_server --ros-args --params-file "$PARAMS" -p use_sim_time:=false \
    -p zone_shaper.zones_file:="$TERRAIN/zones.yaml"
  NODES="$NODES,'smoother_server'"
fi
start lm.log ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lm_plan -p autostart:=true \
  -p node_names:="[$NODES]" -p bond_timeout:=0.0
for i in $(seq 1 40); do grep -q 'Managed nodes are active' "$OUT/lm.log" && break; sleep 0.5; done
if ! grep -q 'Managed nodes are active' "$OUT/lm.log"; then
  echo 'planner did not come up'; tail -n 20 "$OUT/lm.log" "$OUT/planner.log"; exit 2
fi
sleep 3   # global costmap: first static map + inflation
python3 /tmp/navdev/probe_route_directions.py
rc=$?
exit $rc
