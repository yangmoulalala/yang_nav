#!/usr/bin/env bash
# Start navigation against an existing localization/sensor stack.
# Resolve the workspace from this script so callers may run it from any directory.
set -e -o pipefail

usage() {
  cat <<'HELP'
Usage: bash tools/start_navigation.sh [--dry-run] [launch arguments...]

Start Nav2, terrain obstacle filtering and the final command gate in the foreground.
Localization and sensor drivers must already be running in the same ROS domain.
Simulation additionally requires an active /clock publisher.

Examples:
  bash tools/start_navigation.sh rviz:=true
  bash tools/start_navigation.sh profile:=real rviz:=true
  bash tools/start_navigation.sh profile:=real terrain_dir:=maps/real/site_a_001 goal_x:=1.0 goal_y:=1.5
  bash tools/start_navigation.sh --dry-run profile:=real
  ROS_DOMAIN_ID=78 bash tools/start_navigation.sh terrain_dir:=maps/sim/terrain

Options:
  --help       Show this help without loading ROS.
  --dry-run    Load the environment and print the command without launching nodes.

Launch arguments are forwarded to nav_bringup/navigation.launch.py:
  mode:=localization|mapping Default: localization; mapping uses live LiDAR on flat ground
  terrain_dir:=PATH    Default: maps/<profile> (terrain/ when present)
  params_file:=PATH    Full custom YAML; otherwise use installed sim/real profile defaults
  profile:=sim|real    Default: sim (use_sim_time:=false also selects real)
  use_sim_time:=BOOL   Default: true for sim, false for real
  auto_bag:=BOOL       Default: true for real navigation; false disables the lightweight bag
  cmd_vel_topic:=TOPIC Default: /cmd_vel (geometry_msgs/Twist)
  require_trusted:=BOOL Default: true
  rviz:=BOOL           Default: false
  goal_x:=X goal_y:=Y  Optional: send one goal in map (metres) after navigation is ready
  goal_yaw:=RADIANS   Default: 0.0 (faces map +X)
  goal_wait_timeout:=SECONDS Default: 120.0; no goal is sent if readiness times out
  serial_intents:=auto|true|false  Default: auto; map A4 home/target intents from
                       /chassis/nav_intent to the same-named map landmark and send
                       NavigateToPose once per change. auto enables this for the real
                       localization profile when the terrain has home/target landmarks
  landmarks_file:=PATH Annotated terrain.json holding home/target in map frame
                       Default: <terrain_dir>/terrain.json
  serial_goals:=BOOL  Legacy PointStamped input only; not compatible with A4 home/target intents
  serial_target_topic:=TOPIC Default: /chassis/nav_target (geometry_msgs/PointStamped)

Relative paths are resolved from the workspace root. ROS_DOMAIN_ID and ROS_LOG_DIR
are inherited. Ctrl-C stops this launch; it does not stop external localization/sensors.
HELP
}

nav_dry_run=false
nav_args=()
for nav_arg in "$@"; do
  case "$nav_arg" in
    --help|-h) usage; exit 0 ;;
    --dry-run) nav_dry_run=true ;;
    *) nav_args+=("$nav_arg") ;;
  esac
done

NAV_WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
for nav_setup in /opt/ros/jazzy/setup.bash "$NAV_WORKSPACE/install/setup.bash"; do
  if [[ ! -f "$nav_setup" ]]; then
    printf 'Missing setup file: %s. Install ROS Jazzy and build the workspace first.\n' "$nav_setup" >&2
    exit 2
  fi
  # ROS setup scripts may reference unset variables; enable nounset afterwards.
  source "$nav_setup"
done
set -u
cd -- "$NAV_WORKSPACE"
nav_command=(ros2 launch nav_bringup navigation.launch.py "${nav_args[@]}")
printf 'Workspace: %s\nROS_DOMAIN_ID: %s\nCommand:' "$NAV_WORKSPACE" "${ROS_DOMAIN_ID:-0}"
printf ' %q' "${nav_command[@]}"
printf '\n'
if [[ "$nav_dry_run" == true ]]; then
  exit 0
fi
exec "${nav_command[@]}"
