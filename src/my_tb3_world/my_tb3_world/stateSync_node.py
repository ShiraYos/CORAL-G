#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy


class StateSyncNode(Node):
    def __init__(self):
        super().__init__('state_sync_node')

        self.declare_parameter('storage_capacity_items', 3)
        self.declare_parameter('fuel_drain_rate', 0.05)   # % per second
        self.declare_parameter('fuel_low_threshold', 20.0)
        self.declare_parameter('base_radius_m', 0.5)

        self.max_capacity = self.get_parameter('storage_capacity_items').value
        self.fuel_drain_rate = self.get_parameter('fuel_drain_rate').value
        self.fuel_low_threshold = self.get_parameter('fuel_low_threshold').value
        self.base_radius_m = self.get_parameter('base_radius_m').value

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile)

        self.nav_status_subscription = self.create_subscription(
            String, '/coral_g/navigation_status', self.nav_status_callback, 10)

        self.state_publisher = self.create_publisher(String, '/coral_g/robot_state', 10)

        self.timer = self.create_timer(0.5, self.publish_state)
        self.fuel_timer = self.create_timer(1.0, self.drain_fuel)

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0

        # Storage (integer count, normalized to 0.0–1.0 on publish)
        self.storage_count = 0

        # Fuel (0–100 internally, normalized to 0.0–1.0 on publish)
        self.fuel_pct = 100.0
        self.fuel_low = False

        # Nav status mirror
        self.last_nav_status = ''

        self.get_logger().info('State Sync Node started')
        self.get_logger().info(f'Storage capacity: {self.max_capacity} items')
        self.get_logger().info(f'Fuel low threshold: {self.fuel_low_threshold}%')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny_cosp, cosy_cosp)

        self.velocity = msg.twist.twist.linear.x

    def nav_status_callback(self, msg: String):
        status = msg.data

        if status == self.last_nav_status:
            return
        self.last_nav_status = status

        if status == 'succeeded':
            if self.is_at_base():
                self.get_logger().info('Returned to base — resetting storage and fuel')
                self.storage_count = 0
                self.fuel_pct = 100.0
                self.fuel_low = False
            else:
                self.storage_count = min(self.storage_count + 1, self.max_capacity)
                self.get_logger().info(
                    f'Collection: storage {self.storage_count}/{self.max_capacity}'
                )

    def drain_fuel(self):
        if self.fuel_pct > 0.0:
            self.fuel_pct = max(0.0, self.fuel_pct - self.fuel_drain_rate)

        if self.fuel_pct <= self.fuel_low_threshold and not self.fuel_low:
            self.fuel_low = True
            self.get_logger().warn(f'Fuel low: {self.fuel_pct:.1f}%')

    def is_at_base(self) -> bool:
        return math.sqrt(self.x ** 2 + self.y ** 2) < self.base_radius_m

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def publish_state(self):
        storage_fill = round(self.storage_count / self.max_capacity, 3)
        fuel_level = round(self.fuel_pct / 100.0, 3)

        state = {
            'schema': 'coral_g.robot_state.v1',
            'stamp': self._stamp(),
            'source': 'robot_state_node',
            'pose': {
                'x': round(self.x, 3),
                'y': round(self.y, 3),
                'heading': round(self.heading, 3),
            },
            'velocity': round(self.velocity, 3),
            'storage_fill': storage_fill,
            'storage_count': self.storage_count,
            'storage_capacity': self.max_capacity,
            'fuel_level': fuel_level,
            'at_base': self.is_at_base(),
            'navigation_status': self.last_nav_status,
        }

        msg = String()
        msg.data = json.dumps(state)
        self.state_publisher.publish(msg)

        self.get_logger().info(
            f'pos=({state["pose"]["x"]}, {state["pose"]["y"]}) '
            f'fuel={fuel_level:.2f} storage={storage_fill:.2f} '
            f'at_base={state["at_base"]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StateSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
