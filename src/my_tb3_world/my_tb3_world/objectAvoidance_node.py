#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy


class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('objectAvoidance_node')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.safe_distance = 0.4      # meters - stop if obstacle closer than this
        self.front_angle_deg = 25     # degrees - cone in front of robot to check

        self.get_logger().info('Object Avoidance Node started')

    def lidar_callback(self, msg: LaserScan):

        # Get distances in front arc
        front_distances = self.get_front_arc_distances(msg, self.front_angle_deg)

        # Filter out invalid readings
        valid_ranges = [
            r for r in front_distances
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        # If no valid readings, stop for safety
        if not valid_ranges:
            self.get_logger().warn('No valid front range readings - stopping')
            self.publish_cmd(0.0, 0.0)
            return

        min_front = min(valid_ranges)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'

        if min_front < self.safe_distance:
            # Obstacle detected - turn left until clear
            self.get_logger().info(f'Obstacle detected at {min_front:.2f}m - turning')
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.6
        else:
            # Path clear - move forward
            twist_msg.twist.linear.x = 0.2
            twist_msg.twist.angular.z = 0.0

        self.publisher.publish(twist_msg)

    def publish_cmd(self, linear: float, angular: float):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = linear
        twist_msg.twist.angular.z = angular
        self.publisher.publish(twist_msg)

    def get_front_arc_distances(self, scan_msg: LaserScan, front_angle_deg: float) -> List[float]:
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        front_angle_rad = math.radians(front_angle_deg)
        selected = []

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if -front_angle_rad <= angle <= front_angle_rad:
                selected.append(distance)

        return selected


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()