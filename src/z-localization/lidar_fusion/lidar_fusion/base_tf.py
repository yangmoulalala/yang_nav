"""把 LIO 里程计换算到机器人本体系，发布 world → base_link 的 TF。

    odometry_topic   LIO 里程计：header.frame_id 为世界系（lio_odom），
                     child_frame_id 为传感器系（imu）
    base_frame       机器人本体系，默认 base_link

传感器系到 base_link 的外参只从 tf2 读取（URDF / robot_state_publisher），本节点不保存任何数值。
定位结果因此接到 URDF 树上：map → lio_odom → base_link → … → mid360_link → imu。
传感器系在 URDF 树里已有父帧，前端必须关闭自身 TF（zlio publish_tf:=false），否则一帧两个父帧。
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from lidar_fusion.merge_node import matrix_of


def quaternion_of(r):
    """3×3 旋转矩阵转四元数 (x, y, z, w)；Shepperd 法，先开方最大分量，数值稳定。"""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > max(r[0, 0], r[1, 1], r[2, 2]):
        s = 2 * math.sqrt(1 + trace)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0] * 4
        q[i] = s / 4
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
        q[3] = (r[k, j] - r[j, k]) / s
    return tuple(float(v) for v in q)


class BaseTfNode(Node):
    def __init__(self):
        super().__init__('lio_base_tf')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        topic = self.declare_parameter('odometry_topic', '/odometry').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.on_odometry, 100)
        self.get_logger().info('%s -> TF <world> -> %s' % (topic, self.base_frame))

    def on_odometry(self, odom):
        try:
            # base_link 在传感器系中的位姿（即 URDF 外参的逆），转台转动时也随之更新
            sensor_base = matrix_of(self.tf_buffer.lookup_transform(
                odom.child_frame_id, self.base_frame, Time()).transform)
        except TransformException as error:
            self.get_logger().warn('waiting for TF: %s' % error, throttle_duration_sec=2.0)
            return
        p = odom.pose.pose.position
        world_sensor = matrix_of(Transform(rotation=odom.pose.pose.orientation))
        world_sensor[:3, 3] = [p.x, p.y, p.z]
        m = world_sensor @ sensor_base

        out = TransformStamped()
        out.header = odom.header
        out.child_frame_id = self.base_frame
        t, r = out.transform.translation, out.transform.rotation
        t.x, t.y, t.z = (float(v) for v in m[:3, 3])
        r.x, r.y, r.z, r.w = quaternion_of(m[:3, :3])
        self.broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = BaseTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
