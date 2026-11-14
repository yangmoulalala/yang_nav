#!/usr/bin/env bash
# Record an existing manual mapping session. This script starts no motion nodes.
set -e -o pipefail
RECORD_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  python3 "$RECORD_WS/tools/record_real_mapping.py" --help
  exit 0
fi
source /opt/ros/jazzy/setup.bash
source "$RECORD_WS/install/setup.bash"
exec python3 "$RECORD_WS/tools/record_real_mapping.py" "$@"
