"""把主雷达与可选副雷达点云合并为带逐点时间的单一 PointCloud2。

    primary_topic    主雷达点云（输出坐标系默认取其所在 link）
    secondary_topic  副雷达点云；为空时单雷达直通
    output_topic     合并点云，字段 x y z offset_time(uint32, ns，相对合并帧头)

外参只从 tf2 读取（即 URDF / robot_state_publisher），本节点不保存任何数值。

逐点时间（每路单独配置 <prefix>_time_field / _time_scale / _time_absolute）：
  - time_field 为空：整帧点都取帧头时间（Gazebo gpu_lidar 同一时刻渲染）；
  - 否则读该字段，乘 time_scale 换算为秒，time_absolute=false 时再加上帧头时间。
输出帧头取所有输入点中最早的时刻，保证 offset_time 非负。
"""
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

FIELDS = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
          PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
          PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
          PointField(name='offset_time', offset=12, datatype=PointField.UINT32, count=1)]
DTYPE = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32),
                  ('offset_time', np.uint32)])


# 大点云在 best-effort 下会因 UDP 分片丢帧：ros_gz_bridge 发布端为 RELIABLE，
# 这里订阅/发布都用 RELIABLE，深度 5 保证只积压最新几帧。
QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def matrix_of(transform):
    """geometry_msgs/Transform 转 4×4 齐次矩阵。"""
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


class Input:
    """一路输入的逐点时间约定。"""

    def __init__(self, node, prefix, topic):
        self.topic = node.declare_parameter(prefix + '_topic', topic).value
        self.time_field = node.declare_parameter(prefix + '_time_field', '').value
        self.time_scale = node.declare_parameter(prefix + '_time_scale', 1e-9).value
        self.time_absolute = node.declare_parameter(prefix + '_time_absolute', False).value

    def read(self, cloud):
        """返回有限点的 N×3 坐标与绝对时间（ns，int64）。"""
        names = [f.name for f in cloud.fields]
        timed = bool(self.time_field)
        if timed and self.time_field not in names:
            raise ValueError('%s has no point time field %r (fields %s)'
                             % (self.topic, self.time_field, names))
        fields = ('x', 'y', 'z') + ((self.time_field,) if timed else ())
        points = point_cloud2.read_points(cloud, field_names=fields, skip_nans=True)
        xyz = np.stack([points['x'], points['y'], points['z']], axis=-1).astype(np.float64)
        base = stamp_ns(cloud.header.stamp)
        if timed:
            t = points[self.time_field].astype(np.float64) * self.time_scale * 1e9
            t = np.round(t).astype(np.int64) + (0 if self.time_absolute else base)
        else:
            t = np.full(len(xyz), base, dtype=np.int64)
        keep = np.isfinite(xyz).all(axis=1)
        return xyz[keep], t[keep]


class MergeNode(Node):
    def __init__(self):
        super().__init__('lidar_merge')
        self.inputs = [Input(self, 'primary', '/lidar_mid360/points')]
        secondary = Input(self, 'secondary', '')
        if secondary.topic:
            self.inputs.append(secondary)
        self.output_frame = self.declare_parameter('output_frame', 'mid360_link').value
        self.min_range = self.declare_parameter('min_range', 0.3).value
        slop = self.declare_parameter('sync_slop', 0.05).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, self.declare_parameter(
            'output_topic', '/lidar/points').value, QOS)

        if len(self.inputs) == 2:
            subs = [message_filters.Subscriber(self, PointCloud2, i.topic, qos_profile=QOS)
                    for i in self.inputs]
            self.sync = message_filters.ApproximateTimeSynchronizer(subs, 10, slop)
            self.sync.registerCallback(self.on_clouds)
        else:
            self.create_subscription(PointCloud2, self.inputs[0].topic, self.on_clouds, QOS)
        self.get_logger().info('merging %s -> %s' % (
            ' + '.join(i.topic for i in self.inputs), self.output_frame))

    def in_output_frame(self, source, cloud):
        """按 tf2 把点云变换到输出坐标系，并剔除近距离车体点。"""
        xyz, t = source.read(cloud)
        keep = np.linalg.norm(xyz, axis=1) >= self.min_range
        xyz, t = xyz[keep], t[keep]
        if cloud.header.frame_id != self.output_frame:
            m = matrix_of(self.tf_buffer.lookup_transform(
                self.output_frame, cloud.header.frame_id, Time()).transform)
            xyz = xyz @ m[:3, :3].T + m[:3, 3]
        return xyz, t

    def on_clouds(self, *clouds):
        try:
            parts = [self.in_output_frame(i, c) for i, c in zip(self.inputs, clouds)]
        except TransformException as error:
            self.get_logger().warn('waiting for TF: %s' % error, throttle_duration_sec=2.0)
            return
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        xyz = np.concatenate([p[0] for p in parts])
        t = np.concatenate([p[1] for p in parts])
        if not len(t):
            return
        base = int(t.min())
        out = np.empty(len(t), dtype=DTYPE)
        out['x'], out['y'], out['z'] = xyz.T
        out['offset_time'] = t - base
        header = clouds[0].header
        header.frame_id = self.output_frame
        header.stamp.sec, header.stamp.nanosec = divmod(base, 1_000_000_000)
        self.pub.publish(point_cloud2.create_cloud(header, FIELDS, out))


def main():
    rclpy.init()
    node = MergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
