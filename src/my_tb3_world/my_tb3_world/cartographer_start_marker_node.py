#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return math.sin(half_yaw), math.cos(half_yaw)


class CartographerStartMarkerNode(Node):
    def __init__(self):
        super().__init__('cartographer_start_marker_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            MarkerArray,
            '/cartographer_start/marker_array',
            qos,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(rate, 0.1), self.publish_marker)
        self.publish_marker()

        self.get_logger().info(
            'Cartographer start marker publishing on /cartographer_start/marker_array'
        )

    def publish_marker(self):
        frame_id = self.get_parameter('frame_id').value
        x = float(self.get_parameter('start_x').value)
        y = float(self.get_parameter('start_y').value)
        yaw = float(self.get_parameter('start_yaw').value)
        qz, qw = yaw_to_quaternion(yaw)
        stamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        marker_array.markers.append(self._origin_disk(frame_id, stamp, x, y))
        marker_array.markers.append(self._heading_arrow(frame_id, stamp, x, y, qz, qw))
        marker_array.markers.append(self._label(frame_id, stamp, x, y, yaw))
        self.pub.publish(marker_array)

    def _base_marker(self, frame_id, stamp, marker_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'cartographer_start'
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime.sec = 0
        marker.frame_locked = True
        return marker

    def _origin_disk(self, frame_id, stamp, x, y):
        marker = self._base_marker(frame_id, stamp, 0)
        marker.type = Marker.CYLINDER
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.015
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.03
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.75
        return marker

    def _heading_arrow(self, frame_id, stamp, x, y, qz, qw):
        marker = self._base_marker(frame_id, stamp, 1)
        marker.type = Marker.ARROW
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 0.75
        marker.scale.y = 0.09
        marker.scale.z = 0.09
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 0.95
        return marker

    def _label(self, frame_id, stamp, x, y, yaw):
        marker = self._base_marker(frame_id, stamp, 2)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f'Cartographer start\\n({x:.2f}, {y:.2f}, yaw {yaw:.2f})'
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = CartographerStartMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
