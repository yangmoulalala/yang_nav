"""Filter committed frontend observations against annotated or locally level ground."""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from localization_interfaces.msg import FrontendFrame

from nav_terrain.obstacle_filter import FilterConfig, GroundFilter, LiveGroundFilter
from nav_terrain.mapping_ground import ground_evidence
from nav_terrain.terrain_map import TerrainMap
from nav_terrain.registration import quaternion_matrix


def matrix(transform):
    t, q = transform.translation, transform.rotation
    return quaternion_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


class TerrainObstacles(Node):
    def __init__(self):
        super().__init__('terrain_obstacles')
        path = self.declare_parameter('terrain_file', '').value
        self.mapping = self.declare_parameter('mapping', False).value
        cfg = FilterConfig()
        for name in cfg.__dataclass_fields__:
            setattr(cfg, name, self.declare_parameter(name, getattr(cfg, name)).value)
        self.filter = LiveGroundFilter(cfg) if self.mapping else GroundFilter(TerrainMap.load(path), cfg)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.base_ground_offset = self.declare_parameter('base_ground_offset', 0.0).value
        self.max_age = self.declare_parameter('max_age', 0.5).value
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.obstacles = self.create_publisher(PointCloud2, '/nav/obstacles', qos_profile_sensor_data)
        self.clearing = self.create_publisher(PointCloud2, '/nav/clearing', qos_profile_sensor_data)
        self.heartbeat = self.create_publisher(Header, '/nav/observation', 5)
        if self.mapping:
            self.ground_resolution = self.declare_parameter('mapping_ground_resolution', 0.15).value
            self.map_max_age = self.declare_parameter('mapping_ground_max_age', 6.0).value
            if not np.isfinite(self.map_max_age) or self.map_max_age <= 0.0 or not (
                    0.025 <= self.ground_resolution <= 0.15):
                raise ValueError('invalid mapping ground age or resolution')
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.ground_map = self.create_publisher(OccupancyGrid, '/nav/mapping_ground', latched)
            self.runtime_cloud = None
            self.published_cloud = None
            self.ground_grid = None
            self.create_subscription(PointCloud2, '/backend_node/runtime_map', self.receive_map, latched)
            self.reset_ground()
            self.create_timer(0.5, self.update_ground)
        self.pending = None
        self.create_subscription(FrontendFrame, '/frontend/frame', self.receive, qos_profile_sensor_data)
        # TF may arrive after the observation. Retry only the newest frame, without
        # blocking the executor that receives TF, and never replay a stale cloud.
        self.create_timer(0.02, self.process)
        self.get_logger().info('terrain obstacles ready: ' + ('live flat-ground mode' if self.mapping else path))

    def receive(self, frame):
        self.pending = frame

    def receive_map(self, cloud):
        self.runtime_cloud = cloud

    def set_ground(self, grid, origin, stamp):
        message = OccupancyGrid(header=Header(frame_id=self.map_frame, stamp=stamp))
        message.info.resolution = self.ground_resolution
        message.info.height, message.info.width = grid.shape
        message.info.origin.orientation.w = 1.0
        message.info.origin.position.x, message.info.origin.position.y = map(float, origin)
        message.data = grid.ravel().tolist()
        self.ground_grid = message

    def reset_ground(self):
        self.set_ground(np.full((1, 1), -1, dtype=np.int8), (0.0, 0.0),
                        self.get_clock().now().to_msg())
        self.published_cloud = None

    def update_ground(self):
        self.refresh_ground()
        self.ground_map.publish(self.ground_grid)

    def refresh_ground(self):
        cloud = self.runtime_cloud
        now = self.get_clock().now()

        def current(message):
            if message is None or message.header.frame_id != self.map_frame:
                return False
            age = (now - Time.from_msg(message.header.stamp)).nanoseconds / 1e9
            return -0.05 <= age <= self.map_max_age

        if self.published_cloud is not None and not current(self.published_cloud):
            self.reset_ground()
        if cloud is self.published_cloud or not current(cloud):
            return
        stamp = Time.from_msg(cloud.header.stamp)
        try:
            _, base = matrix(self.tf.lookup_transform(self.map_frame, self.base_frame, stamp).transform)
            points = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
            grid, origin = ground_evidence(points, base, self.base_ground_offset, self.filter.config,
                                           self.ground_resolution, self.filter.config.max_range)
        except (TransformException, ValueError, TypeError, AssertionError):
            return
        self.set_ground(grid, origin, cloud.header.stamp)
        self.published_cloud = cloud

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
            if self.mapping and not rays.size:
                return
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
