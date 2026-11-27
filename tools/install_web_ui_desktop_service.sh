#!/usr/bin/env bash
set -euo pipefail

web_port=${1:-8080}
web_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! $web_port =~ ^[1-9][0-9]{0,4}$ ]] || ((web_port > 65535)); then
  printf 'Usage: bash tools/install_web_ui_desktop_service.sh [port]\n' >&2
  exit 2
fi
if ! /usr/bin/python3 -c 'import gi; gi.require_version("Gtk", "4.0"); gi.require_version("WebKit", "6.0"); from gi.repository import Gtk, WebKit'; then
  printf 'Install python3-gi gir1.2-gtk-4.0 gir1.2-webkit-6.0 first.\n' >&2
  exit 2
fi

chmod u+x "$web_workspace/tools/web_ui/desktop.py"
web_workspace=${web_workspace//%/%%}
web_unit_dir=${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user
web_service=yang-nav-web-ui-desktop.service
mkdir -p "$web_unit_dir"
cat > "$web_unit_dir/$web_service" <<UNIT
[Unit]
Description=yang_nav WebUI on an external display
After=graphical-session-pre.target
PartOf=graphical-session.target

[Service]
WorkingDirectory=$web_workspace
ExecStart=/usr/bin/python3 -u -m tools.web_ui.desktop --port $web_port
Restart=on-failure
RestartSec=3s

[Install]
WantedBy=graphical-session.target
UNIT
systemd-analyze --user verify "$web_unit_dir/$web_service"
systemctl --user daemon-reload
systemctl --user enable "$web_service"
if systemctl --user is-active --quiet graphical-session.target; then
  systemctl --user restart "$web_service"
fi
