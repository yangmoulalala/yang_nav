"""真实前后端通过生产组合 launch 验证停车协议和运动暖重置的地图连续性。"""

import atexit
from contextlib import ExitStack
import math
import os
from pathlib import Path
import struct
import tempfile
import time
import unittest

from domain_coordinator import domain_id
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from localization_interfaces.msg import FrontendFrame, FrontendOdometry, FrontendSession
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
import yaml

_resources = ExitStack()
atexit.register(_resources.close)


@launch_testing.parametrize('scenario', ['cold', 'moving', 'recovery', 'startup'])
def generate_test_description(scenario):
    """为停车、实际运动、运行恢复和启动中断分别启动独立的前后端进程。"""
    os.environ['ROS_DOMAIN_ID'] = str(_resources.enter_context(domain_id()))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    root = Path(_resources.enter_context(tempfile.TemporaryDirectory(prefix='bringup-protocol-')))
    os.environ['ROS_LOG_DIR'] = str(root / 'logs')
    configuration = root / 'backend.yaml'
    configuration.write_text(yaml.safe_dump({
        'common.frontend': 'zlio', 'common.odom_frame': 'odom',
        'common.input_cloud_in_local_frame': True, 'uwb.enabled': False,
        'loop.enabled': scenario == 'moving',
        'common.scan_context.exclude_recent': 10000,
        'trajectory.save_directory': str(root / 'map'), 'trajectory.save_on_shutdown': False,
        'trajectory.keyframe.min_interval': 0.0, 'trajectory.keyframe.max_interval': .1,
        'reset.service': '/zlio/reset_internal', 'reset.min_gap': 0.0,
        'visualization.enabled': False,
    }), encoding='utf-8')
    entry = Path(__file__).resolve().parents[1] / 'launch/bringup.launch.py'
    system = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(entry)), launch_arguments={
        'frontend_config': LaunchConfiguration('config'), 'backend_config': str(configuration),
        'mode': 'mapping', 'rviz': 'false', 'input_type': 'pointcloud2',
        'wait_for_start': 'true' if scenario == 'cold' else 'false',
        'reset_mode': 'cold' if scenario == 'cold' else 'warm',
        'lidar_topic': '/test/lidar', 'imu_topic': '/test/imu',
        'imu_acceleration_scale': '1.0',
    }.items())
    return launch.LaunchDescription([system, launch_testing.actions.ReadyToTest()]), {'root': root}


def stamp_ns(stamp):
    """将 ROS 时间转换为整数纳秒用于精确配对输入和输出。"""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def motion_at(seconds):
    """定义完成静止初始化后的匀加速平移与恒定偏航角速度。"""
    elapsed = max(0.0, seconds - 4.5)
    return .2 * elapsed * elapsed, .2 * elapsed


def yaw_of(pose):
    """从消息姿态读取绕重力方向的航向角。"""
    q = pose.orientation
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class TestResetProtocol(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('coordinated_reset_observer')
        self.addCleanup(self.node.destroy_node)
        retained = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sessions, self.states, self.odometry = [], [], []
        self.node.create_subscription(
            FrontendSession, '/frontend/session', self.sessions.append, retained)
        self.node.create_subscription(
            String, '/backend_node/localization_state', self.observe_state, retained)
        self.node.create_subscription(Odometry, '/localization/odometry', self.odometry.append, 20)
        self.observed_frames = []
        self.node.create_subscription(
            FrontendFrame, '/frontend/frame', self.observed_frames.append, 100)
        self.frames = self.node.create_publisher(FrontendFrame, '/frontend/frame', 20)
        self.predictions = self.node.create_publisher(FrontendOdometry, '/frontend/odometry', 20)

    def observe_state(self, message):
        self.states.append(dict(
            field.split('=', 1) for field in message.data.split(';') if '=' in field))

    def wait_until(self, predicate, drive=None, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if drive is not None:
                drive()
            rclpy.spin_once(self.node, timeout_sec=.05)
        self.assertTrue(predicate(), 'coordinated protocol did not reach expected state')

    def call(self, name):
        client = self.node.create_client(Trigger, name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10))
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=45)
            self.assertTrue(future.done())
            return future.result()
        finally:
            self.node.destroy_client(client)

    def reset_to(self, epoch):
        # 服务发现有独立到达次序；预检拒绝没有副作用，直到发送成功才等待边界。
        deadline = time.monotonic() + 10
        accepted = False
        while not accepted and time.monotonic() < deadline:
            accepted = self.call('/localization_reset').success
        self.assertTrue(accepted)
        self.wait_until(lambda: self.sessions[-1].epoch == epoch
                        and self.states[-1].get('epoch') == str(epoch))

    def observation(self, epoch, sequence, stamp, x):
        message = FrontendFrame(instance_id=self.sessions[-1].instance_id,
                                epoch=epoch, sequence=sequence)
        message.odometry.header = Header(frame_id='odom')
        message.odometry.header.stamp.sec = stamp
        message.begin = message.odometry.header.stamp
        message.odometry.child_frame_id = 'imu'
        message.odometry.pose.pose.orientation.w = 1.0
        message.odometry.pose.pose.position.x = x
        for axis in range(6):
            message.odometry.pose.covariance[7*axis] = .01
        message.cloud = point_cloud2.create_cloud_xyz32(
            Header(frame_id='imu', stamp=message.odometry.header.stamp),
            [(1.0 + .1*i, .1*j, 1.0) for i in range(8) for j in range(8)])
        return message

    def publish_observation(self, message):
        self.wait_until(lambda: self.frames.get_subscription_count() > 0
                        and self.predictions.get_subscription_count() > 0)
        prediction = FrontendOdometry(instance_id=message.instance_id, epoch=message.epoch,
                                      odometry=message.odometry)

        def publish():
            self.frames.publish(message)
            self.predictions.publish(prediction)

        stamp = message.odometry.header.stamp.sec
        self.wait_until(
            lambda: any(item.header.stamp.sec == stamp for item in self.odometry), publish)
        return next(item for item in self.odometry if item.header.stamp.sec == stamp)

    def check_cold_protocol(self, root):
        """停车重置保持地图位置，并拒绝来自旧代次的同步帧。"""
        self.wait_until(lambda: self.sessions and self.states)
        instance = self.sessions[-1].instance_id
        self.reset_to(1)
        self.assertEqual(self.sessions[-1].instance_id, instance)
        self.publish_observation(self.observation(1, 1, 10, 1.0))
        self.frames.publish(self.observation(0, 99, 11, 100.0))
        self.publish_observation(self.observation(1, 2, 12, 2.0))
        self.assertTrue(self.call('/backend_node/save_map').success)
        rows = [line.split() for line in (root / 'map/keyframes.txt').read_text().splitlines()
                if line and not line.startswith('#')]
        self.assertEqual([float(row[1]) for row in rows], [10, 12])
        self.reset_to(2)
        current = self.publish_observation(self.observation(2, 1, 13, 0.0))
        self.assertAlmostEqual(current.pose.pose.position.x, 2.0, places=7)
        self.assertEqual(self.states[-1].get('status'), 'NORMAL')
        self.assertTrue(self.call('/backend_node/save_map').success)
        rows = [line.split() for line in (root / 'map/keyframes.txt').read_text().splitlines()
                if line and not line.startswith('#')]
        self.assertEqual([float(row[1]) for row in rows], [10, 12, 13])
        self.assertEqual(self.sessions[-1].instance_id, instance)

    def moving_scan(self, end):
        """按每个回波的真实时刻变换房间几何，再应用传感器外参的逆变换。"""
        count = len(self.room_points)
        begin = end - 100_000_000
        records = []
        for index, point in enumerate(self.room_points):
            offset = (index + 1) * 100_000_000 // count
            position_x, yaw = ((0.0, 0.0) if self.stationary
                               else motion_at((begin + offset) * 1e-9))
            c, s = math.cos(yaw), math.sin(yaw)
            x, y, z = point
            body = (c*(x-position_x) + s*y, -s*(x-position_x) + c*y, z)
            shifted = tuple(value - origin
                            for value, origin in zip(body, self.body_from_lidar_translation))
            lidar = [sum(self.body_from_lidar_rotation[row][column] * shifted[row]
                         for row in range(3))
                     for column in range(3)]
            records.append(struct.pack('<fffI', *lidar, offset))
        cloud = PointCloud2(height=1, width=count, point_step=16, row_step=16*count, is_dense=True)
        cloud.header.frame_id = 'lidar'
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(begin, 1_000_000_000)
        cloud.fields = [PointField(name=name, offset=4*index, count=1,
                                   datatype=PointField.FLOAT32 if index < 3 else PointField.UINT32)
                        for index, name in enumerate(('x', 'y', 'z', 'offset_time'))]
        cloud.data = b''.join(records)
        return cloud

    def feed_motion(self, samples, lidar=True):
        """发送与逐点房间扫描一致的解析 IMU，传感器时间连续推进。"""
        final_step = self.step + samples

        def publish():
            """发布一个解析惯性采样，并按扫描周期补充逐点运动几何。"""
            self.step += 1
            timestamp = 1_000_000_000 + self.step * 10_000_000
            imu = Imu()
            imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
            _, yaw = motion_at(timestamp * 1e-9)
            moving = not self.stationary and timestamp > 4_500_000_000
            imu.linear_acceleration.x = .4 * math.cos(yaw) if moving else 0.0
            imu.linear_acceleration.y = -.4 * math.sin(yaw) if moving else 0.0
            imu.linear_acceleration.z = self.gravity
            imu.angular_velocity.z = .2 if moving else 0.0
            self.imus.publish(imu)
            if lidar and self.step % 10 == 0:
                self.clouds.publish(self.moving_scan(timestamp))
            if self.step == final_step:
                timer.cancel()

        timer = self.node.create_timer(.005, publish)
        try:
            self.wait_until(lambda: self.step == final_step, timeout=30)
        finally:
            self.node.destroy_timer(timer)

    def current_motion_output(self, epoch, status='NORMAL'):
        """等待该代次真实同步帧入图，再以新 IMU 时间配对后端输出。"""
        last_scan = 1_000_000_000 + self.step // 10 * 100_000_000
        self.wait_until(
            lambda: any(frame.epoch == epoch and
                        stamp_ns(frame.odometry.header.stamp) >= last_scan - 50_000_000
                        for frame in self.observed_frames), timeout=45)
        self.wait_until(lambda: self.states[-1].get('epoch') == str(epoch)
                        and self.states[-1].get('status') == status)
        self.feed_motion(1, lidar=False)
        timestamp = 1_000_000_000 + self.step * 10_000_000
        self.wait_until(lambda: any(stamp_ns(item.header.stamp) == timestamp
                                   for item in self.odometry))
        return next(item for item in reversed(self.odometry)
                    if stamp_ns(item.header.stamp) == timestamp)

    def check_motion(self, before, after):
        """用解析物理运动比较相对位移和航向，不依赖前端自身位姿作为真值。"""
        start = stamp_ns(before.header.stamp) * 1e-9
        finish = stamp_ns(after.header.stamp) * 1e-9
        start_x, start_yaw = motion_at(start)
        finish_x, finish_yaw = motion_at(finish)
        heading = yaw_of(before.pose.pose)
        c, s = math.cos(heading), math.sin(heading)
        dx = after.pose.pose.position.x - before.pose.pose.position.x
        dy = after.pose.pose.position.y - before.pose.pose.position.y
        actual_translation = (c*dx + s*dy, -s*dx + c*dy)
        expected_translation = ((finish_x-start_x)*math.cos(start_yaw),
                                -(finish_x-start_x)*math.sin(start_yaw))
        self.assertLess(math.hypot(actual_translation[0]-expected_translation[0],
                                   actual_translation[1]-expected_translation[1]), .025)
        actual_yaw_change = yaw_of(after.pose.pose) - heading
        yaw_error = actual_yaw_change - (finish_yaw-start_yaw)
        self.assertLess(abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error))), .015)
        self.assertTrue(all(math.isfinite(value) for value in after.pose.covariance))

    def prepare_motion(self, config, stationary=False):
        """按实际传感器配置建立几何与发布器，并等待会话和传感器连接就绪。"""
        self.stationary = stationary
        sensor = yaml.safe_load(Path(config).read_text(encoding='utf-8'))['sensor']
        extrinsic = sensor['body_from_lidar']
        self.gravity = float(sensor['gravity'])
        self.body_from_lidar_translation = extrinsic['translation']
        x, y, z, w = extrinsic['rotation_xyzw']
        self.body_from_lidar_rotation = [
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        self.room_points = []
        for i in range(-20, 21):
            for j in range(-20, 21):
                u, v = .1*i, .1*j
                self.room_points.extend(((4, u, v), (u, 4, v), (u, v, -2)))
        self.step = 0
        self.imus = self.node.create_publisher(Imu, '/test/imu', 1000)
        self.clouds = self.node.create_publisher(PointCloud2, '/test/lidar', 100)
        self.wait_until(lambda: self.sessions and self.states
                        and self.imus.get_subscription_count() > 0
                        and self.clouds.get_subscription_count() > 0)

    def check_saved_motion(self, root, before):
        """从实际保存的图关键帧核对运动，覆盖持久化后的地图坐标。"""
        self.assertTrue(self.call('/backend_node/save_map').success)
        rows = [list(map(float, line.split()))
                for line in (root / 'map/keyframes.txt').read_text().splitlines()
                if line and not line.startswith('#')]
        self.assertGreater(len(rows), 3)
        later = [row for row in rows if row[1] > stamp_ns(before.header.stamp) * 1e-9]
        self.assertGreaterEqual(len(later), 2)
        for row in later:
            saved = Odometry()
            saved.header.stamp.sec, saved.header.stamp.nanosec = divmod(
                round(row[1] * 1_000_000_000), 1_000_000_000)
            (saved.pose.pose.position.x, saved.pose.pose.position.y,
             saved.pose.pose.position.z) = row[2:5]
            (saved.pose.pose.orientation.x, saved.pose.pose.orientation.y,
             saved.pose.pose.orientation.z, saved.pose.pose.orientation.w) = row[5:9]
            self.check_motion(before, saved)

    def check_moving_protocol(self, root, config):
        """真实前端服务和后端图在两次运动暖重置后保留边界到首帧间的位移。"""
        self.prepare_motion(config)
        self.feed_motion(550)
        before = self.current_motion_output(0)
        self.assertGreater(math.hypot(before.twist.twist.linear.x,
                                     before.twist.twist.linear.y), .5)
        instance = self.sessions[-1].instance_id
        for epoch in (1, 2):
            self.reset_to(epoch)
            self.assertEqual(self.sessions[-1].instance_id, instance)
            # 暖重置后继续真实运动，但延迟 0.2 秒才恢复扫描，明确分离边界与首帧。
            self.feed_motion(20, lidar=False)
            self.feed_motion(60)
            current = self.current_motion_output(epoch)
            self.check_motion(before, current)
            self.assertEqual(self.states[-1].get('status'), 'NORMAL')
            self.assertTrue(any(frame.epoch == epoch and frame.lidar_updated and frame.matches > 0
                                for frame in self.observed_frames))

        self.check_saved_motion(root, before)

    def check_startup_recovery(self, config):
        """持续发送具有采样缺口的输入，验证前端在原代次重试且不请求后端重置。"""
        self.prepare_motion(config, stationary=True)
        instance = self.sessions[-1].instance_id
        self.assertEqual(self.sessions[-1].epoch, 0)
        notifications = len(self.sessions)

        def publish_gap_sample():
            """任意两个已接收样本均间隔至少 0.2 秒，容忍中间样本丢失。"""
            self.step += 20
            imu = Imu()
            timestamp = 1_000_000_000 + self.step*10_000_000
            imu.header.stamp.sec, imu.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
            imu.linear_acceleration.z = self.gravity
            self.imus.publish(imu)

        timer = self.node.create_timer(.02, publish_gap_sample)
        try:
            self.wait_until(lambda: len(self.sessions) > notifications)
        finally:
            self.node.destroy_timer(timer)
        self.assertEqual(self.sessions[-1].epoch, 0)
        self.assertEqual(self.sessions[-1].instance_id, instance)
        self.assertFalse(self.sessions[-1].reset_required)
        self.assertFalse(self.sessions[-1].failed)
        self.assertEqual(self.states[-1].get('epoch'), '0')
        self.assertFalse(self.observed_frames)
        self.feed_motion(350)
        self.current_motion_output(0)
        self.assertTrue(all(session.epoch == 0 and session.instance_id == instance
                            and not session.reset_required and not session.failed
                            for session in self.sessions))

    def check_automatic_recovery(self, config):
        """运行故障通过会话通知完成无人工请求的协调重置。"""
        self.prepare_motion(config, stationary=True)
        instance = self.sessions[-1].instance_id
        self.assertEqual(self.sessions[-1].epoch, 0)
        self.feed_motion(350)
        before = self.current_motion_output(0)
        self.assertFalse(any(session.reset_required for session in self.sessions))
        self.assertEqual(self.sessions[-1].instance_id, instance)

        # 故障期间停止发布样本；由后端收到旧代次故障通知后完成唯一协调事务。
        self.step += 19
        self.feed_motion(1, lidar=False)
        self.wait_until(lambda: any(session.epoch == 0 and session.reset_required
                                   for session in self.sessions))
        self.wait_until(lambda: self.sessions[-1].epoch == 1
                        and not self.sessions[-1].reset_required
                        and self.states[-1].get('epoch') == '1')
        self.assertEqual(self.sessions[-1].instance_id, instance)
        self.feed_motion(350)
        # 冷启动没有跨代运动边界；即使测试中停车，也不能据此声称系统已验证连续性。
        after = self.current_motion_output(1, status='LOST')
        delta = [getattr(after.pose.pose.position, axis) - getattr(before.pose.pose.position, axis)
                 for axis in ('x', 'y', 'z')]
        self.assertLess(math.sqrt(sum(value*value for value in delta)), .025)
        self.assertTrue(any(frame.epoch == 1 and frame.lidar_updated and frame.matches > 0
                            for frame in self.observed_frames))
        self.assertEqual(self.states[-1].get('status'), 'LOST')
        self.assertEqual(self.states[-1].get('pose_trusted'), 'false')

        # 主动停止不发布恢复请求，后端不得因继续收到传感器数据而另开重置事务。
        self.assertTrue(self.call('/stop').success)
        self.step += 19
        self.feed_motion(60)
        self.assertTrue(self.call('/stop').success)
        self.assertEqual(self.sessions[-1].epoch, 1)
        self.assertEqual(self.states[-1].get('epoch'), '1')
        self.assertFalse(any(session.epoch == 1 and session.reset_required
                             for session in self.sessions))

        # 普通处理失败通知后端进入 LOST，但不得触发自动冷启动循环。
        self.reset_to(2)
        self.step += 1
        invalid = Imu()
        timestamp = 1_000_000_000 + self.step * 10_000_000
        invalid.header.stamp.sec, invalid.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
        invalid.linear_acceleration.z = float('nan')
        self.imus.publish(invalid)
        self.wait_until(lambda: self.sessions[-1].epoch == 2 and self.sessions[-1].failed
                        and self.states[-1].get('status') == 'LOST')
        self.assertFalse(self.sessions[-1].reset_required)
        self.feed_motion(60)
        self.assertEqual(self.sessions[-1].epoch, 2)
        self.assertEqual(self.states[-1].get('epoch'), '2')
        self.assertEqual(self.states[-1].get('pose_trusted'), 'false')

    def test_coordinated_reset_contract(self, root, scenario, test_args):
        """分别执行停车身份、解析运动及自动故障协调场景。"""
        if scenario == 'cold':
            self.check_cold_protocol(root)
        elif scenario == 'moving':
            self.check_moving_protocol(root, test_args['config'])
        elif scenario == 'startup':
            self.check_startup_recovery(test_args['config'])
        else:
            self.check_automatic_recovery(test_args['config'])


@launch_testing.post_shutdown_test()
class TestSystemExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
