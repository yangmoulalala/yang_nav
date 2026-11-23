#!/usr/bin/env python3
"""Retain the last requested navigation goal for visualization only."""

import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class GoalVisualizer(Node):
    def __init__(self):
        super().__init__('navigation_goal_visualizer')
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/navigation/goal_marker', retained)
        self.interactive = self.create_subscription(
            PoseStamped, '/goal_pose', self.on_goal,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.accepted = self.create_subscription(
            PoseStamped, '/navigation/goal_accepted', self.on_goal, retained)
        self.goal = None
        # Also support clients whose subscriptions are volatile, and keep the TF
        # lookup time fresh when the browser connects long after the goal sender exits.
        self.timer = self.create_timer(1.0, self.publish)

    def on_goal(self, goal):
        p, q = goal.pose.position, goal.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v * v for v in (q.x, q.y, q.z, q.w)))
        if not goal.header.frame_id or not all(map(math.isfinite, values)) or norm < 1e-6:
            self.get_logger().warning('Ignoring invalid goal visualization pose')
            return
        self.goal = copy.deepcopy(goal)
        q = self.goal.pose.orientation
        q.x, q.y, q.z, q.w = (v / norm for v in (q.x, q.y, q.z, q.w))
        self.publish()

    def publish(self):
        if self.goal is None:
            return
        array = MarkerArray()
        for marker_id, marker_type in enumerate((Marker.ARROW, Marker.SPHERE, Marker.TEXT_VIEW_FACING)):
            marker = Marker()
            marker.header.frame_id = self.goal.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'last_navigation_goal'
            marker.id = marker_id
            marker.type = marker_type
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(self.goal.pose)
            # Raise the visual above the occupancy grid without changing the goal.
            marker.pose.position.z += 0.2
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.957, 0.447, 0.714, 1.0)
            if marker_type == Marker.ARROW:
                marker.scale.x, marker.scale.y, marker.scale.z = (0.9, 0.15, 0.2)
            elif marker_type == Marker.SPHERE:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            else:
                marker.pose.position.z += 0.65
                marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.z = 0.24
                p = self.goal.pose.position
                marker.text = f'Last goal ({p.x:.2f}, {p.y:.2f})'
            array.markers.append(marker)
        self.markers.publish(array)


def main():
    rclpy.init()
    node = GoalVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
