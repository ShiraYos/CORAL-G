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

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ── Subscribers ──────────────────────────────────────────────
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        self.nav_status_subscription = self.create_subscription(
            String,
            '/coral_g/navigation_status',
            self.nav_status_callback,
            10
        )

        # ── Publishers ───────────────────────────────────────────────
        self.state_publisher = self.create_publisher(
            String,
            '/coral_g/robot_state',
            10
        )

        # ── Timer — publish state every 0.5 seconds ──────────────────
        self.timer = self.create_timer(0.5, self.publish_state)

        # ── Robot state variables ────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0

        # Container
        self.container_capacity = 0
        self.max_capacity = 3
        self.container_full = False

        # Battery
        self.battery_level = 100.0
        self.battery_low = False
        self.battery_low_threshold = 20.0
        self.battery_drain_rate = 0.05  # % per second — drains slowly

        # Battery drain timer — runs every second
        self.battery_timer = self.create_timer(1.0, self.drain_battery)

        # Track navigation status
        self.last_nav_status = ''
        self.collections_since_last_return = 0

        self.get_logger().info('State Sync Node started')
        self.get_logger().info(f'Max container capacity: {self.max_capacity}')
        self.get_logger().info(f'Battery low threshold: {self.battery_low_threshold}%')

    def odom_callback(self, msg: Odometry):
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract heading from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny_cosp, cosy_cosp)

        # Extract velocity
        self.velocity = msg.twist.twist.linear.x

    def nav_status_callback(self, msg: String):
        status = msg.data

        # Avoid processing same status twice
        if status == self.last_nav_status:
            return
        self.last_nav_status = status

        if status == 'succeeded':
            # Check if robot returned to origin
            if self.is_at_origin():
                self.get_logger().info('Robot returned to origin - resetting container and battery')
                self.container_capacity = 0
                self.container_full = False
                self.battery_level = 100.0
                self.battery_low = False
                self.collections_since_last_return = 0
            else:
                # Robot arrived at waste zone - increment container
                self.container_capacity += 1
                self.collections_since_last_return += 1
                self.get_logger().info(
                    f'Waste collected! Container: {self.container_capacity}/{self.max_capacity}'
                )

                if self.container_capacity >= self.max_capacity:
                    self.container_full = True
                    self.get_logger().info('Container full! Robot should return to origin')

    def drain_battery(self):
        if self.battery_level > 0:
            self.battery_level -= self.battery_drain_rate
            self.battery_level = max(0.0, self.battery_level)

        if self.battery_level <= self.battery_low_threshold and not self.battery_low:
            self.battery_low = True
            self.get_logger().warn(
                f'Battery low! {self.battery_level:.1f}% - Robot should return to origin'
            )

    def is_at_origin(self) -> bool:
        # Check if robot is within 0.5m of origin point (0, 0)
        distance_to_origin = math.sqrt(self.x ** 2 + self.y ** 2)
        return distance_to_origin < 0.5

    def publish_state(self):
        state = {
            'x': round(self.x, 3),
            'y': round(self.y, 3),
            'heading': round(self.heading, 3),
            'velocity': round(self.velocity, 3),
            'container_capacity': self.container_capacity,
            'max_capacity': self.max_capacity,
            'container_full': self.container_full,
            'battery_level': round(self.battery_level, 1),
            'battery_low': self.battery_low,
            'should_return_to_origin': self.container_full or self.battery_low
        }

        msg = String()
        msg.data = json.dumps(state)
        self.state_publisher.publish(msg)

        self.get_logger().info(
            f'State: pos=({state["x"]}, {state["y"]}) '
            f'battery={state["battery_level"]}% '
            f'container={state["container_capacity"]}/{self.max_capacity} '
            f'return={state["should_return_to_origin"]}'
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