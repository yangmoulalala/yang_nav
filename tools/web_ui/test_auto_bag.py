import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import Mock

import pytest

from tools.record_bag import MCAP_MAGIC, checkpoint_mcap, write_manifest

from .processes import Processes
from .settings import Settings, WORKSPACE


def test_default_enabled_on_both_platforms(tmp_path):
    settings = Settings(tmp_path)
    assert settings.data['sim']['auto_bag'] is True
    assert settings.data['real']['auto_bag'] is True


def manager(tmp_path, profile='sim'):
    settings = Settings(tmp_path)
    settings.setup({'profile': profile, 'auto_bag': True})
    ros = Mock()
    ros.node_names.return_value = set()
    processes = Processes(settings, ros, tmp_path)
    processes.launch = Mock()
    return processes


@pytest.mark.parametrize('profile', ['sim', 'real'])
def test_navigation_auto_record_command(tmp_path, profile):
    processes = manager(tmp_path, profile)
    processes.start_auto_bag()
    name, command, tracked, destination = processes.launch.call_args.args
    assert name == 'bag' and tracked == {'automatic': True}
    assert '--summary' in command and '--all' not in command
    assert ('--use-sim-time' in command) == (profile == 'sim')
    assert Path(destination).parent == WORKSPACE / 'bags' / 'recordings'
    assert Settings(tmp_path).data[profile]['auto_bag']


def test_opt_out_or_existing_recording_does_not_start(tmp_path):
    processes = manager(tmp_path)
    processes.settings.setup({'profile': 'sim', 'auto_bag': False})
    processes.start_auto_bag()
    processes.settings.setup({'profile': 'sim', 'auto_bag': True})
    processes.ros.node_names.return_value = {'/rosbag2_recorder'}
    processes.start_auto_bag()
    processes.ros.node_names.return_value.clear()
    manual = Mock(parameters={'automatic': False}, stopping=False)
    manual.child.poll.return_value = None
    processes.entries['bag'] = manual
    processes.start_auto_bag()
    processes.launch.assert_not_called()


@pytest.mark.parametrize('enabled,existing,expected', [(True,False,'true'), (True,True,'false'), (False,False,'false')])
def test_real_navigation_owns_summary(tmp_path, enabled, existing, expected):
    processes = manager(tmp_path, 'real')
    processes.settings.setup({'profile':'real', 'mode':'mapping', 'auto_bag':enabled})
    processes.ros.node_names.return_value = {'/backend_node'} | ({'/rosbag2_recorder'} if existing else set())
    processes.wait_for_localization = Mock()
    processes.start('navigation', {})
    processes.launch.assert_called_once()
    name, command, *_ = processes.launch.call_args.args
    assert name == 'navigation' and 'auto_bag:=' + expected in command


@pytest.mark.parametrize('profile', ['real', 'sim'])
def test_stack_does_not_wait_for_bag_readiness_and_keeps_recording_on_nav_stop(tmp_path, profile):
    processes = manager(tmp_path, profile)
    processes.settings.setup({'profile': profile, 'mode': 'mapping', 'auto_bag': True})
    sequence = []

    def launch(name, command, parameters, *args):
        sequence.append(name)
        process = Mock(parameters=parameters)
        process.child.poll.return_value = None
        processes.entries[name] = process
        assert name != 'bag' or '--navigation-session' in command

    def start(name, payload):
        sequence.append(name)
        process = Mock(parameters={'mode': 'mapping'})
        process.child.poll.return_value = None
        processes.entries[name] = process

    processes.launch.side_effect = launch
    processes.start = Mock(side_effect=start)
    processes.check_start = Mock(side_effect=lambda delay=0: sequence.append('delay') if delay else None)
    processes.start_stack()
    first_target = 'gazebo' if profile == 'sim' else 'localization'
    assert set(sequence[:2]) == {'bag', first_target}
    assert 'localization' in sequence and 'navigation' in sequence
    assert processes.auto_bag_warning is None
    bag = processes.entries['bag']
    processes.stop('navigation')
    bag.stop.assert_not_called()
    processes.stop('localization')
    bag.stop.assert_called_once_with(0)


def test_stack_does_not_wait_for_previous_recording_to_finalize(tmp_path):
    processes = manager(tmp_path, 'real')
    bag = Mock(parameters={'automatic': True}, stopping=True)
    bag.child.poll.return_value = None
    processes.entries['bag'] = bag
    processes.cancel_start.wait = Mock(side_effect=AssertionError('must not wait for the old recorder'))
    processes.start_auto_bag(stack=True)
    processes.launch.assert_not_called()
    assert processes.auto_bag_warning


def test_stack_localization_failure_finalizes_its_own_recording(tmp_path):
    processes = manager(tmp_path, 'real')
    processes.settings.setup({'profile': 'real', 'mode': 'mapping', 'auto_bag': True})
    bag = Mock(parameters={'automatic': True, 'stack_start': True})
    processes.launch.side_effect = lambda *args: processes.entries.update(bag=bag)
    processes.start = Mock(side_effect=ValueError('localization failed'))
    with pytest.raises(ValueError, match='localization failed'):
        processes.start_stack()
    bag.stop.assert_called_once_with(0)


@pytest.mark.parametrize('automatic', [True, False])
def test_stop_navigation_only_finalizes_its_own_bag(tmp_path, automatic):
    processes = manager(tmp_path)
    bag = Mock(parameters={'automatic': automatic})
    bag.child.poll.return_value = None
    nav = Mock()
    processes.entries.update(bag=bag, navigation=nav)
    processes.stop('navigation')
    nav.stop.assert_called_once()
    if automatic:
        bag.stop.assert_called_once_with(0)
    else:
        bag.stop.assert_not_called()


def test_recorder_spawn_failure_is_not_navigation_failure(tmp_path):
    processes = manager(tmp_path)
    processes.launch.side_effect = OSError('unavailable')
    processes.start_auto_bag()
    assert '导航继续运行' in processes.auto_bag_warning


def test_summary_topics_and_mutual_exclusion():
    command = [sys.executable, str(WORKSPACE / 'tools/record_bag.py'), '--summary', '--dry-run']
    topics = json.loads(subprocess.check_output(command))['topics']
    assert '/navigate_to_pose/_action/status' in topics and '/cmd_vel_nav' in topics
    assert not any('points' in topic or 'costmap' in topic for topic in topics)
    assert '/clock' not in topics
    assert '/clock' in json.loads(subprocess.check_output(command + ['--use-sim-time']))['topics']
    assert subprocess.run(command + ['--all'], capture_output=True).returncode != 0


def test_checkpoint_records_only_sealed_mcap_segments(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    open_segment = bag / 'bag_1.mcap'
    open_segment.write_bytes(MCAP_MAGIC + b'pending')
    (bag / 'bag_0.mcap').write_bytes(MCAP_MAGIC + b'data' + MCAP_MAGIC)
    (bag / 'bag_2.mcap').write_bytes(MCAP_MAGIC)
    manifest = {'completed': False, 'durable_mcap_segments': []}
    write_manifest(tmp_path, manifest, durable=True)
    checkpoint_mcap(tmp_path, manifest)
    path = tmp_path / 'session.json'
    assert json.loads(path.read_text()) == {'completed': False, 'durable_mcap_segments': ['bag_0.mcap']}
    inode = path.stat().st_ino
    checkpoint_mcap(tmp_path, manifest)
    assert path.stat().st_ino == inode
    with open_segment.open('ab') as stream:
        stream.write(MCAP_MAGIC)
    checkpoint_mcap(tmp_path, manifest)
    assert json.loads(path.read_text())['durable_mcap_segments'] == ['bag_0.mcap', 'bag_1.mcap']
    assert not (tmp_path / 'session.json.tmp').exists()
