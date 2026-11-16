"""Exercise the actual single-input subscription and PointCloud2 data contract."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from lidar_fusion.merge_node import FIELDS, Input, MergeNode


def cloud(rows, timed=False, frame='mid360_link'):
    fields = FIELDS[:3] + ([PointField(name='time', offset=12,
                                     datatype=PointField.FLOAT64, count=1)] if timed else [])
    header = Header(frame_id=frame)
    header.stamp.sec, header.stamp.nanosec = 10, 250_000_000
    return point_cloud2.create_cloud(header, fields, rows)


def source(field='', scale=1., absolute=False):
    value = Input.__new__(Input)
    value.topic = 'primary/points'
    value.time_field, value.time_scale, value.time_absolute = field, scale, absolute
    return value


@pytest.fixture
def node():
    config = Path(__file__).resolve().parents[1]/'config/example.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config), '-p', 'dual:=false'])
    value = MergeNode()
    try:
        yield value
    finally:
        value.destroy_node()
        rclpy.shutdown()


def test_single_input_does_not_subscribe_to_secondary(node):
    subscriptions = dict(node.get_subscriber_names_and_types_by_node(node.get_name(), '/'))
    assert '/primary/points' in subscriptions
    assert '/secondary/points' not in subscriptions
    assert len(node.inputs) == 1


def test_simultaneous_scan_uses_header_stamp_and_removes_nonfinite_points():
    xyz, stamps = source().read(cloud([(1., 2., 3.), (math.nan, 0., 0.), (math.inf, 0., 0.)]))
    np.testing.assert_allclose(xyz, [[1., 2., 3.]])
    np.testing.assert_array_equal(stamps, [10_250_000_000])


@pytest.mark.parametrize('absolute,values,expected', [
    (False, [0., 100.], [10_250_000_000, 10_250_100_000]),
    (True, [10_250_000., 10_250_100.], [10_250_000_000, 10_250_100_000]),
])
def test_per_point_microseconds_become_absolute_nanoseconds(absolute, values, expected):
    msg = cloud([(1., 0., 0., values[0]), (2., 0., 0., values[1])], timed=True)
    _, stamps = source('time', 1e-6, absolute).read(msg)
    np.testing.assert_array_equal(stamps, expected)


def test_configured_time_field_must_exist():
    with pytest.raises(ValueError, match='has no point time field'):
        source('time').read(cloud([(1., 0., 0.)]))


def test_extrinsic_rotation_translation_and_body_range_filter(node):
    transform = TransformStamped()
    transform.transform.translation.x = 2.
    transform.transform.rotation.z = math.sin(math.pi/4)
    transform.transform.rotation.w = math.cos(math.pi/4)
    node.tf_buffer.lookup_transform = lambda *_: transform
    node.min_range = .5
    xyz, stamps = node.in_output_frame(source(), cloud([(.1, 0., 0.), (1., 0., 0.)], frame='sensor'))
    np.testing.assert_allclose(xyz, [[2., 1., 0.]], atol=1e-7)
    np.testing.assert_array_equal(stamps, [10_250_000_000])


def test_single_scan_publishes_earliest_time_and_nonnegative_offsets(node):
    node.inputs = [source('time', 1e-6)]
    output = []
    node.pub = SimpleNamespace(publish=output.append)
    node.on_clouds(cloud([(1., 0., 0., 100.), (2., 0., 0., 0.)], timed=True))
    assert len(output) == 1
    msg = output[0]
    assert msg.header.frame_id == node.output_frame
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (10, 250_000_000)
    points = point_cloud2.read_points(msg)
    np.testing.assert_allclose(points['x'], [1., 2.])
    np.testing.assert_array_equal(points['offset_time'], [100_000, 0])


def test_all_body_points_produce_no_output(node):
    node.min_range = .5
    output = []
    node.pub = SimpleNamespace(publish=output.append)
    node.on_clouds(cloud([(.1, 0., 0.)]))
    assert output == []
