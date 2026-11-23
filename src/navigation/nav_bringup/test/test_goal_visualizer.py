"""Verify retained goal display in a separate ROS domain without commanding Nav2."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def test_goal_display_survives_sender_exit_and_accepts_interactive_updates():
    domain = 187
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('goal_visualizer_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    accepted = node.create_publisher(PoseStamped, '/navigation/goal_accepted', retained)
    interactive = node.create_publisher(
        PoseStamped, '/goal_pose', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    script = Path(__file__).resolve().parents[1] / 'scripts' / 'goal_visualizer.py'
    process = subprocess.Popen(
        [sys.executable, str(script)], env=dict(os.environ, ROS_DOMAIN_ID=str(domain)),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def wait_for(predicate):
        deadline = time.monotonic() + 8
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate()

    try:
        wait_for(lambda: accepted.get_subscription_count() == 1)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = 1.0, 1.5
        pose.pose.orientation.w = 1.0
        accepted.publish(pose)
        # Wait for reliable handoff, then remove the original publisher entirely.
        from rclpy.duration import Duration
        assert accepted.wait_for_all_acked(Duration(seconds=2))
        node.destroy_publisher(accepted)
        messages = []
        subscription = node.create_subscription(
            MarkerArray, '/navigation/goal_marker', messages.append, retained)
        wait_for(lambda: bool(messages))
        arrow, point, label = messages[-1].markers
        assert (arrow.type, point.type, label.type) == (Marker.ARROW, Marker.SPHERE, Marker.TEXT_VIEW_FACING)
        assert arrow.header.frame_id == 'map'
        assert arrow.pose.position.x == 1.0
        assert arrow.pose.position.y == 1.5
        assert arrow.pose.position.z == pytest.approx(0.2)
        assert label.text == 'Last goal (1.00, 1.50)'
        assert node.count_publishers('/goal_pose') == 1  # Only this test's input publisher.
        # A later interactive goal replaces the retained Action goal, including orientation.
        pose.pose.position.x = -2.0
        pose.pose.orientation.z, pose.pose.orientation.w = 2.0, 0.0
        wait_for(lambda: interactive.get_subscription_count() == 1)
        interactive.publish(pose)
        wait_for(lambda: messages[-1].markers[0].pose.position.x == -2.0)
        assert messages[-1].markers[0].pose.orientation.z == pytest.approx(1.0)
        # Invalid messages must not replace the last useful goal.
        pose.pose.position.x = float('nan')
        interactive.publish(pose)
        before = len(messages)
        wait_for(lambda: len(messages) > before)
        assert messages[-1].markers[0].pose.position.x == -2.0
        node.destroy_subscription(subscription)
        # A client joining after both inputs stop still receives a marker.
        late = []
        node.create_subscription(MarkerArray, '/navigation/goal_marker', late.append, 10)
        wait_for(lambda: bool(late))
        assert late[-1].markers[0].pose.position.x == -2.0
    finally:
        process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=6)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    assert process.returncode == 0, output
