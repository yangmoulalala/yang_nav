#!/usr/bin/env bash
set -eo pipefail
web_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
for web_setup in /opt/ros/jazzy/setup.bash "$web_workspace/install/setup.bash"; do
  if [[ ! -f "$web_setup" ]]; then
    printf 'Missing %s. Install ROS 2 Jazzy and build the workspace first.\n' "$web_setup" >&2
    exit 2
  fi
  source "$web_setup"
done
cd "$web_workspace"
web_python=${YANG_NAV_WEB_PYTHON:-python3}
if [[ -z ${YANG_NAV_WEB_PYTHON:-} && -z ${VIRTUAL_ENV:-} && -x "$HOME/.venvs/nav-terrain/bin/python" ]]; then
  web_python=$HOME/.venvs/nav-terrain/bin/python
fi
exec "$web_python" -m tools.web_ui.server "$@"
