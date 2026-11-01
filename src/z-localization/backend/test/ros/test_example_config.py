"""示例配置恰好等于节点消费的参数集（不含 launch 独占参数）：节点严格拒绝缺失与未知参数，
因此通过生产 launch 加载 example.yaml 并成功启动即证明两者一致。"""

import atexit
from contextlib import ExitStack
import os
from pathlib import Path
import tempfile
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
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestExampleExit(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        """确认后端未因参数错误异常退出。"""
        launch_testing.asserts.assertExitCodes(proc_info)
