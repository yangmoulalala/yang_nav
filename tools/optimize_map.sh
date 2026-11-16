#!/usr/bin/env bash
set -e -o pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$ws/install/setup.bash" ]]; then
  echo 'Build backend and offline_mapping_optimizer first; see docs/offline_mapping.md' >&2
  exit 1
fi
source "$ws/install/setup.bash"
exec python3 "$ws/tools/optimize_map.py" "$@"
