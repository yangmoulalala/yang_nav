import math
import threading
import time
from urllib.parse import unquote

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def vector(linear):
    return {'x': linear.x, 'y': linear.y, 'speed': math.hypot(linear.x, linear.y)}


def pose(message):
    position = message.pose.pose.position
    rotation = message.pose.pose.orientation
    yaw = math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                     1 - 2 * (rotation.y**2 + rotation.z**2))
    return {'x': position.x, 'y': position.y, 'yaw': math.degrees(yaw),
            'frame': message.header.frame_id}


def fields(message):
    return {key: unquote(value) for key, value in
            (item.split('=', 1) for item in message.data.split(';') if '=' in item)}


class RosState(Node):
    def __init__(self):
        super().__init__('yang_nav_web_ui')
        self.lock = threading.Lock()
        self.values = {}
        self.last_lifecycle = None
        self.lifecycle_timeout = 0.0
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.trigger_clients = {
            'save_map': self.create_client(Trigger, '/backend_node/save_map'),
            'relocalize': self.create_client(Trigger, '/backend_node/global_relocalize'),
            'cancel_goal': self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal'),
        }
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.watch('localization', String, '/backend_node/localization_state', latched, fields, 2.0)
        self.watch('mode', String, '/nav_mode', latched, lambda msg: msg.data, None)
        self.watch('pose', PoseWithCovarianceStamped, '/backend_node/localization_pose',
                   qos_profile_sensor_data, pose, 2.0)
        self.watch('measured', Odometry, '/localization/odometry', qos_profile_sensor_data,
                   lambda msg: vector(msg.twist.twist.linear), 2.0)
        self.watch('command', Twist, '/cmd_vel', qos_profile_sensor_data,
                   lambda msg: vector(msg.linear), 0.6)
        self.watch('chassis', Bool, '/chassis/big_yaw_online', latched,
                   lambda msg: msg.data, None)
        self.watch('serial_target', PointStamped, '/chassis/nav_target', qos_profile_sensor_data,
                   lambda msg: {'x': msg.point.x, 'y': msg.point.y, 'frame': msg.header.frame_id}, None)
        self.watch('serial_velocity', TwistStamped, '/chassis/tx_velocity', qos_profile_sensor_data,
                   lambda msg: {**vector(msg.twist.linear), 'frame': msg.header.frame_id}, 0.1)
        self.create_timer(1.0, self.query_lifecycle)

    def watch(self, name, ros_type, topic, qos, transform, expires):
        def receive(message):
            with self.lock:
                self.values[name] = (transform(message), time.monotonic(), expires, topic)

        self.create_subscription(ros_type, topic, receive, qos)

    def query_lifecycle(self):
        if not self.lifecycle.service_is_ready():
            return
        if self.last_lifecycle and not self.last_lifecycle.done():
            if time.monotonic() < self.lifecycle_timeout:
                return
            self.last_lifecycle.cancel()
        self.lifecycle_timeout = time.monotonic() + 2
        self.last_lifecycle = self.lifecycle.call_async(GetState.Request())
        self.last_lifecycle.add_done_callback(self.save_lifecycle)

    def save_lifecycle(self, future):
        if future.cancelled() or future.exception() is not None:
            return
        with self.lock:
            self.values['nav2'] = (future.result().current_state.label, time.monotonic(), 3.0, None)

    def node_names(self):
        return {namespace.rstrip('/') + '/' + name for name, namespace in self.get_node_names_and_namespaces()}

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            current = dict(self.values)
        data = {key: value for key, (value, received, expires, topic) in current.items()
                if (expires is None or now - received < expires)
                and (topic is None or self.count_publishers(topic) > 0)}
        data['services'] = {name: client.service_is_ready() for name, client in self.trigger_clients.items()}
        return data

    def call(self, action):
        client = self.trigger_clients[action]
        if not client.service_is_ready():
            raise ValueError('ROS 服务未就绪，请检查对应进程和 ROS_DOMAIN_ID')
        request = CancelGoal.Request() if action == 'cancel_goal' else Trigger.Request()
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(120 if action == 'save_map' else 5):
            future.cancel()
            raise ValueError('等待 ROS 服务响应超时，请查看节点日志确认结果')
        result = future.result()
        if action == 'cancel_goal':
            if not result.goals_canceling:
                raise ValueError('Nav2 当前没有可取消的 NavigateToPose 目标')
            return f'Nav2 正在取消 {len(result.goals_canceling)} 个目标'
        if not result.success:
            raise ValueError(result.message or 'ROS 服务拒绝了请求')
        return result.message
