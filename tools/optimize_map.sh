#!/usr/bin/env bash
set -e -o pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$ws/install/setup.bash" ]]; then
  echo 'Build backend and offline_mapping_optimizer first; see src/mapping/offline_mapping_optimizer/README.md' >&2
  exit 1
fi
source "$ws/install/setup.bash"
exec ros2 run offline_mapping_optimizer optimize_map.py "$@"
