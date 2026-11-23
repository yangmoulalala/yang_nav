"""示例配置恰好等于节点消费的参数集（不含 launch 独占参数）：节点严格拒绝缺失与未知参数，
因此通过生产 launch 加载 example.yaml 并成功启动即证明两者一致。"""

import atexit
from contextlib import ExitStack
import os
from pathlib import Path
import tempfile
import time
import unittest

from domain_coordinator import domain_id
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

_resources = ExitStack()
atexit.register(_resources.close)
PACKAGE = Path(__file__).resolve().parents[2]


def generate_test_description():
    """以示例配置和恒等 imu <- body TF 启动建图模式后端。"""
    os.environ['ROS_DOMAIN_ID'] = str(_resources.enter_context(domain_id()))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    root = Path(_resources.enter_context(tempfile.TemporaryDirectory(prefix='backend-example-')))
    (root / 'logs').mkdir()
    os.environ['ROS_LOG_DIR'] = str(root / 'logs')
    extrinsic = Node(
        package='tf2_ros', executable='static_transform_publisher', name='imu_body_tf',
        arguments=['--frame-id', 'imu', '--child-frame-id', 'body'])
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(PACKAGE / 'launch/backend.launch.py')),
        launch_arguments={
            'params_file': str(PACKAGE / 'config/example.yaml'), 'mode': 'mapping',
            'save_dir': str(root / 'saved'),
        }.items())
    return launch.LaunchDescription([extrinsic, backend, launch_testing.actions.ReadyToTest()]), {}


class TestExampleConfig(unittest.TestCase):
    def test_node_starts_with_example_yaml(self):
        """服务在参数校验与 TF 外参读取完成后才创建，可用即表示启动成功。"""
        rclpy.init()
        node = rclpy.create_node('example_config_observer')
        try:
            client = node.create_client(Trigger, '/backend_node/save_map')
            self.assertTrue(client.wait_for_service(timeout_sec=20),
                            'backend_node did not start with config/example.yaml')
            # 两个话题都须向晚加入的订阅者补发初始化快照；不依赖传感器输入。
            received = {}
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            for name in ('localization_state', 'localization_diagnostics'):
                node.create_subscription(String, '/backend_node/' + name,
                                         lambda msg, key=name: received.update({key: msg.data}), qos)
            deadline = time.monotonic() + 10
            while len(received) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            self.assertEqual(set(received), {'localization_state', 'localization_diagnostics'})
            state = dict(part.split('=', 1) for part in received['localization_state'].split(';'))
            detail = dict(part.split('=', 1) for part in received['localization_diagnostics'].split(';'))
            self.assertEqual(set(state), {'status', 'pose_trusted', 'tf_frozen', 'epoch',
                                          'generation', 'estimator_failure_count', 'reason'})
            for key in state:
                self.assertEqual(state[key], detail[key])
            self.assertEqual(state['status'], 'INITIALIZING')
            self.assertEqual(state['pose_trusted'], 'false')
            self.assertTrue({'tf_published', 'pose_revision', 'stamp_ns',
                             'last_estimator_failure'}.issubset(detail))
            self.assertLess(len(received['localization_state']),
                            len(received['localization_diagnostics']))
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestExampleExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        """确认后端未因参数错误异常退出。"""
        launch_testing.asserts.assertExitCodes(proc_info)
