from unittest.mock import Mock
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from .ros_state import RosState
from .server import Console


def console(tmp_path):
    ros = Mock()
    ros.node_names.return_value = set()
    return Console(tmp_path, ros, 8766)


@pytest.mark.parametrize('enabled', [True, False])
def test_set_serial_intents_accepts_bool(tmp_path, enabled):
    web = console(tmp_path)
    result = web.set_serial_intents({'enabled': enabled})
    web.ros.set_intents_enabled.assert_called_once_with(enabled)
    assert result['enabled'] is enabled
    assert ('开启' if enabled else '关闭') in result['message']


@pytest.mark.parametrize('body', [
    {}, {'enabled': None}, {'enabled': 1}, {'enabled': 0}, {'enabled': 'true'},
    {'enabled': 'false'}, {'enabled': []}, {'enabled': ['true']},
    {'enabled': {'value': True}}, {'enabled': True, 'extra': 1},
    {'enabled': True, 'enabled ': False}, {'enable': True},
])
def test_set_serial_intents_rejects_non_bool(tmp_path, body):
    web = console(tmp_path)
    with pytest.raises(ValueError):
        web.set_serial_intents(body)
    web.ros.set_intents_enabled.assert_not_called()


@pytest.mark.parametrize('enabled', [True, False])
def test_serial_intents_read_and_snapshot_report_state(tmp_path, enabled):
    web = console(tmp_path)
    web.ros.intents_enabled.return_value = enabled
    web.ros.snapshot.return_value = {}
    assert web.serial_intents() == {'enabled': enabled}
    assert web.snapshot()['serial_intents'] is enabled


def test_publisher_topic_and_latched_qos():
    initialized = not rclpy.ok()
    if initialized:
        rclpy.init(args=[])
    try:
        node = RosState()
        try:
            publisher = node.serial_intents
            assert publisher.topic_name == '/navigation/serial_intents_enabled'
            assert publisher.qos_profile.depth == 1
            assert publisher.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            assert publisher.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            assert node.intents_enabled() is True
            assert node.set_intents_enabled(False) is False
            assert node.intents_enabled() is False
        finally:
            node.destroy_node()
    finally:
        if initialized:
            rclpy.shutdown()
