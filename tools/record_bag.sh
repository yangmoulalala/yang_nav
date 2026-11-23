#!/usr/bin/env bash
# Record an existing ROS session: mapping, localization, navigation or debugging.
set -e -o pipefail
RECORD_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# Help and preview require neither ROS nor a built workspace.
for record_arg in "$@"; do
  case "$record_arg" in
    --help|-h|--dry-run) exec python3 "$RECORD_WS/tools/record_bag.py" "$@" ;;
  esac
done
for record_setup in /opt/ros/jazzy/setup.bash "$RECORD_WS/install/setup.bash"; do
  if [[ ! -f "$record_setup" ]]; then
    printf 'Missing setup file: %s. Install ROS Jazzy and build the workspace first.\n' "$record_setup" >&2
    exit 2
  fi
  source "$record_setup"
done
exec python3 "$RECORD_WS/tools/record_bag.py" "$@"
