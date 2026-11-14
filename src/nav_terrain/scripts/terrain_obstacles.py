#!/usr/bin/env python3
"""Filter committed, deskewed frontend observations against the offline ground grid."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from localization_interfaces.msg import FrontendFrame

from obstacle_filter import FilterConfig, GroundFilter
from terrain_map import TerrainMap, _quat_mat


def matrix(transform):
    t, q = transform.translation, transform.rotation
    return _quat_mat(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


class TerrainObstacles(Node):
    def __init__(self):
        super().__init__('terrain_obstacles')
        path = self.declare_parameter('terrain_file', '').value
        cfg = FilterConfig()
        for name in cfg.__dataclass_fields__:
            setattr(cfg, name, self.declare_parameter(name, getattr(cfg, name)).value)
        self.filter = GroundFilter(TerrainMap.load(path), cfg)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.base_ground_offset = self.declare_parameter('base_ground_offset', 0.0).value
        self.max_age = self.declare_parameter('max_age', 0.5).value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.obstacles = self.create_publisher(PointCloud2, '/nav/obstacles', qos_profile_sensor_data)
        self.clearing = self.create_publisher(PointCloud2, '/nav/clearing', qos_profile_sensor_data)
        self.heartbeat = self.create_publisher(Header, '/nav/observation', 5)
        self.pending = None
        self.create_subscription(FrontendFrame, '/frontend/frame', self.receive, qos_profile_sensor_data)
        # TF may arrive after the observation. Retry only the newest frame, without
        # blocking the executor that receives TF, and never replay a stale cloud.
        self.create_timer(0.02, self.process)
        self.get_logger().info('terrain obstacles ready: ' + str(path))

    def receive(self, frame):
        self.pending = frame

    def process(self):
        frame = self.pending
        if frame is None:
            return
        cloud = frame.cloud
        stamp = Time.from_msg(frame.odometry.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age < -0.05 or age > self.max_age:
            self.pending = None
            return
        source = cloud.header.frame_id or frame.odometry.child_frame_id
        try:
            rm, tm = matrix(self.tf.lookup_transform(self.map_frame, source, stamp).transform)
            rb, tb = matrix(self.tf.lookup_transform(self.base_frame, source, stamp).transform)
            _, base = matrix(self.tf.lookup_transform(self.map_frame, self.base_frame, stamp).transform)
        except TransformException:
            return
        self.pending = None
        try:
            points = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.asarray(points).reshape(-1, 3)
            mapped, body = points @ rm.T + tm, points @ rb.T + tb
            hits, rays = self.filter.select(mapped, body, tm, base[2] - self.base_ground_offset)
            header = Header(stamp=stamp.to_msg(), frame_id=source)
            self.obstacles.publish(point_cloud2.create_cloud_xyz32(header, points[hits]))
            self.clearing.publish(point_cloud2.create_cloud_xyz32(header, points[rays]))
            # Header carries acquisition time. cmd_gate holds if processing, TF or
            # the sensor stops, even when the upstream velocity smoother is alive.
            self.heartbeat.publish(header)
        except (ValueError, TypeError, AssertionError) as exc:
            self.get_logger().error('invalid terrain observation: ' + str(exc))


def main():
    rclpy.init()
    node = TerrainObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
