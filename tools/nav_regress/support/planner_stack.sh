#!/usr/bin/env bash
set -eo pipefail
scenario=$1
shift
case "$scenario" in
  chaincheck) manager=lm_chain; domain=78; smoother= ;;
  plan_tunnels) manager=lm_plan; domain=77; smoother=${SMOOTHER:-} ;;
  *) exit 2 ;;
esac
here=$(cd "$(dirname "$0")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$here/../../install/setup.bash"
terrain=${1:-$(python3 "$here/terrain_path.py")}
params=${PARAMS:-$(ros2 pkg prefix --share nav_bringup)/params/nav2.yaml}
out=${OUT:-/tmp/$scenario}
mkdir -p "$out"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$domain}

groups=()
start() {
  local log_name=$1
  shift
  setsid "$@" > "$out/$log_name" 2>&1 &
  groups+=($!)
}
cleanup() {
  local group
  for group in "${groups[@]}"; do kill -- "-$group" 2>/dev/null || true; done
  sleep 1
  for group in "${groups[@]}"; do kill -KILL -- "-$group" 2>/dev/null || true; done
}
if pgrep -f "__node:=$manager" >/dev/null; then
  echo "A $scenario run is still active" >&2
  exit 2
fi
trap cleanup EXIT
start tf.log ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id gimbal_nav
start map.log ros2 run nav2_map_server map_server --ros-args -r __node:=nav_map_server \
  --params-file "$params" -p "yaml_filename:=$terrain/nav_map.yaml" -p use_sim_time:=false
start planner.log ros2 run nav2_planner planner_server --ros-args --params-file "$params" -p use_sim_time:=false
nodes="'nav_map_server','planner_server'"
options=()
if [[ -n "$smoother" ]]; then
  start smoother.log ros2 run nav2_smoother smoother_server --ros-args --params-file "$params" \
    -p use_sim_time:=false -p "zone_shaper.zones_file:=$terrain/zones.yaml"
  nodes+=",'smoother_server'"
  options+=(--smoother "$smoother")
fi
start lm.log ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r "__node:=$manager" \
  -p autostart:=true -p "node_names:=[$nodes]" -p bond_timeout:=0.0
for attempt in {1..40}; do
  grep -q 'Managed nodes are active' "$out/lm.log" && break
  sleep 0.5
done
grep -q 'Managed nodes are active' "$out/lm.log" || { tail -n 20 "$out/lm.log" "$out/planner.log"; exit 2; }
sleep 3
python3 "$here/$scenario.py" --terrain-dir "$terrain" "${options[@]}" "${@:2}"
