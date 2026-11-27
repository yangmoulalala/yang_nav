#!/usr/bin/env bash
set -euo pipefail

fox_domain=${ROS_DOMAIN_ID:-0}
fox_port=${1:-8766}
fox_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! $fox_domain =~ ^(0|[1-9][0-9]{0,2})$ ]] || ((fox_domain > 232)) ||
   [[ ! $fox_port =~ ^[1-9][0-9]{0,4}$ ]] || ((fox_port > 65535)); then
  printf 'Usage: ROS_DOMAIN_ID=0 bash tools/install_foxglove_service.sh [port]\n' >&2
  exit 2
fi
if [[ ! -x /opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge ]]; then
  printf 'Install ros-jazzy-foxglove-bridge first.\n' >&2
  exit 2
fi
if [[ ! -f $fox_workspace/install/setup.bash ]]; then
  printf 'Build the yang_nav workspace first.\n' >&2
  exit 2
fi
if [[ $(loginctl show-user "$USER" -p Linger --value) != yes ]]; then
  loginctl enable-linger "$USER"
fi

fox_unit_dir=${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user
mkdir -p "$fox_unit_dir"
cat > "$fox_unit_dir/yang-nav-foxglove.service" <<UNIT
[Unit]
Description=yang_nav Foxglove Bridge

[Service]
Environment=ROS_DOMAIN_ID=$fox_domain
WorkingDirectory=${fox_workspace//%/%%}
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec /opt/ros/jazzy/lib/foxglove_bridge/foxglove_bridge --ros-args -p address:=0.0.0.0 -p port:=$fox_port -p capabilities:="[connectionGraph]"'
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
UNIT
systemctl --user daemon-reload
systemctl --user enable yang-nav-foxglove.service
systemctl --user restart yang-nav-foxglove.service
systemctl --user is-active yang-nav-foxglove.service
