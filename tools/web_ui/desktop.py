#!/usr/bin/python3
"""Show the local navigation workbench on an external monitor."""

import argparse
from http.client import HTTPConnection, HTTPException
from pathlib import Path
import re
import signal
import subprocess
import sys
import time


DRM_ROOT = Path('/sys/class/drm')
EXTERNAL_CONNECTOR = re.compile(r'card\d+-((?:HDMI-A|HDMI-B|DP|DVI-D|DVI-I|DVI-A|VGA)-\d+(?:-\d+)*)')
POLL_SECONDS = 2
READY_POLL_SECONDS = 0.25
READY_BURST_SECONDS = 10


def connector_name(name):
    return re.sub(r'^HDMI-[AB]-', 'HDMI-', name or '')


def external_connectors(drm_root=DRM_ROOT):
    connected = set()
    for connector in drm_root.glob('card*-*'):
        match = EXTERNAL_CONNECTOR.fullmatch(connector.name)
        if not match:
            continue
        try:
            if (connector / 'status').read_text().strip() == 'connected' and (connector / 'edid').read_bytes():
                connected.add(connector_name(match[1]))
        except OSError:
            continue
    return connected


def web_ui_ready(port):
    connection = HTTPConnection('127.0.0.1', port, timeout=1)
    try:
        connection.request('GET', '/')
        return connection.getresponse().status == 200
    except (OSError, HTTPException):
        return False
    finally:
        connection.close()


def stop_viewer(viewer):
    viewer.terminate()
    try:
        viewer.wait(timeout=5)
    except subprocess.TimeoutExpired:
        viewer.kill()
        viewer.wait()


def watch_displays(port):
    viewer = None
    previous = set()
    dismissed = False
    ready_deadline = 0
    try:
        while True:
            if viewer is not None and (exit_code := viewer.poll()) is not None:
                viewer = None
                dismissed = exit_code == 0

            connected = external_connectors()
            added = connected - previous
            if added:
                ready_deadline = time.monotonic() + READY_BURST_SECONDS
            if not connected or added:
                dismissed = False
            if not connected and viewer is not None:
                stop_viewer(viewer)
                viewer = None
            elif connected and viewer is None and not dismissed and web_ui_ready(port):
                viewer = subprocess.Popen([
                    str(Path(__file__).resolve()), '--port', str(port), '--window',
                ])
            previous = connected
            waiting = connected and viewer is None and not dismissed
            fast_poll = waiting and time.monotonic() < ready_deadline
            time.sleep(READY_POLL_SECONDS if fast_poll else POLL_SECONDS)
    finally:
        if viewer is not None:
            stop_viewer(viewer)


def run_window(port):
    import gi

    gi.require_version('Gdk', '4.0')
    gi.require_version('Gtk', '4.0')
    gi.require_version('WebKit', '6.0')
    from gi.repository import Gdk, GLib, Gtk, WebKit

    if not Gtk.init_check():
        print('No graphical display available for WebUI', file=sys.stderr)
        return 1

    url = f'http://127.0.0.1:{port}/'
    loop = GLib.MainLoop()
    monitors = Gdk.Display.get_default().get_monitors()
    window = None
    selected = None
    exit_code = 0

    def close_window(*_args):
        loop.quit()
        return True

    def navigation_policy(_view, decision, decision_type):
        if decision_type == WebKit.PolicyDecisionType.NAVIGATION_ACTION:
            if decision.get_navigation_action().get_request().get_uri().startswith(url):
                return False
        elif decision_type != WebKit.PolicyDecisionType.NEW_WINDOW_ACTION:
            return False
        decision.ignore()
        return True

    def log_load(view, event):
        if event == WebKit.LoadEvent.COMMITTED:
            print(f'WebUI displayed: {view.get_uri()}', flush=True)

    def retry_load(view, _event, failing_uri, error):
        if failing_uri == url:
            print(f'WebUI load failed; retrying: {error.message}', file=sys.stderr, flush=True)
            GLib.timeout_add_seconds(POLL_SECONDS, lambda: view.load_uri(url) or GLib.SOURCE_REMOVE)
            return True
        return False

    def web_process_terminated(_view, reason):
        nonlocal exit_code
        print(f'WebUI renderer exited: {reason.value_nick}', file=sys.stderr)
        exit_code = 1
        loop.quit()

    def update_window():
        nonlocal selected, window
        connected = external_connectors()
        available = [monitor for monitor in monitors if connector_name(monitor.get_connector()) in connected]
        if selected in available or not available:
            return GLib.SOURCE_CONTINUE

        selected = available[0]
        if window is None:
            network = WebKit.NetworkSession.new_ephemeral()
            network.set_proxy_settings(WebKit.NetworkProxyMode.NO_PROXY, None)
            view = WebKit.WebView(network_session=network)
            view.connect('decide-policy', navigation_policy)
            view.connect('load-changed', log_load)
            view.connect('load-failed', retry_load)
            view.connect('web-process-terminated', web_process_terminated)
            window = Gtk.Window(title='yang_nav 工作台', child=view)
            window.connect('close-request', close_window)
            view.load_uri(url)
        print(f'Showing WebUI on {selected.get_connector()}', flush=True)
        window.fullscreen_on_monitor(selected)
        window.present()
        return GLib.SOURCE_CONTINUE

    def terminate_window():
        close_window()
        return GLib.SOURCE_REMOVE

    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, terminate_window)
    monitors.connect('items-changed', lambda *_args: update_window())
    GLib.timeout_add_seconds(POLL_SECONDS, update_window)
    update_window()
    loop.run()
    if window is not None:
        window.destroy()
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--window', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if args.window:
        return run_window(args.port)
    watch_displays(args.port)
    return 0


if __name__ == '__main__':
    sys.exit(main())
