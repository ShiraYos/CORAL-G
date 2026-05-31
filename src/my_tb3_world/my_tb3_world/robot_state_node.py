#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')

        self.declare_parameter('storage_capacity_items', 3)
        self.declare_parameter('fuel_drain_rate', 0.05)   # % per second
        self.declare_parameter('fuel_low_threshold', 20.0)
        self.declare_parameter('base_radius_m', 0.5)

        self.max_capacity = self.get_parameter('storage_capacity_items').value
        self.fuel_drain_rate = self.get_parameter('fuel_drain_rate').value
        self.fuel_low_threshold = self.get_parameter('fuel_low_threshold').value
        self.base_radius_m = self.get_parameter('base_radius_m').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(String, '/collection_event', self._collection_event_cb, 10)
        self.create_subscription(String, '/base_pose', self._base_pose_cb, 10)

        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        self.create_timer(0.5, self._publish)
        self.create_timer(1.0, self._drain_fuel)

        # Pose (from odom)
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.velocity = 0.0
        self.odom_distance_m = 0.0
        self._prev_x = None
        self._prev_y = None

        # Base pose (from /base_pose — default to origin)
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        self.base_status = 'default'

        # Storage
        self.storage_count = 0

        # Fuel (0–100 internally)
        self.fuel_pct = 100.0
        self.fuel_low = False

        # At-base tracking for reset detection
        self._was_at_base = True  # start True so first departure doesn't trigger reset

        self.get_logger().info('RobotStateNode started')

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny, cosy)
        self.velocity = msg.twist.twist.linear.x

        if self._prev_x is not None:
            self.odom_distance_m += math.sqrt(
                (self.x - self._prev_x) ** 2 + (self.y - self._prev_y) ** 2
            )
        self._prev_x = self.x
        self._prev_y = self.y

    def _collection_event_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /collection_event')
            return

        count = event.get('count', 1)
        self.storage_count = min(self.storage_count + count, self.max_capacity)
        self.get_logger().info(
            f'Collection event: +{count} items → storage {self.storage_count}/{self.max_capacity}'
        )

    def _base_pose_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /base_pose')
            return

        pose = data.get('pose', {})
        self.base_x = pose.get('x', 0.0)
        self.base_y = pose.get('y', 0.0)
        self.base_yaw = pose.get('yaw', 0.0)
        self.base_status = data.get('status', 'known')

    def _drain_fuel(self):
        if self.fuel_pct > 0.0:
            self.fuel_pct = max(0.0, self.fuel_pct - self.fuel_drain_rate)
        if self.fuel_pct <= self.fuel_low_threshold and not self.fuel_low:
            self.fuel_low = True
            self.get_logger().warn(f'Fuel low: {self.fuel_pct:.1f}%')

    def _is_at_base(self) -> bool:
        return math.sqrt(
            (self.x - self.base_x) ** 2 + (self.y - self.base_y) ** 2
        ) < self.base_radius_m

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _publish(self):
        at_base = self._is_at_base()

        # Reset storage and fuel when robot arrives at base
        if at_base and not self._was_at_base:
            self.get_logger().info('Returned to base — resetting storage and fuel')
            self.storage_count = 0
            self.fuel_pct = 100.0
            self.fuel_low = False
        self._was_at_base = at_base

        storage_fill = round(self.storage_count / self.max_capacity, 3)
        fuel_level = round(self.fuel_pct / 100.0, 3)

        state = {
            'schema': 'dtas.robot_state.v1',
            'stamp': self._stamp(),
            'source': 'robot_state_node',
            'pose': {
                'x': round(self.x, 3),
                'y': round(self.y, 3),
                'yaw': round(self.heading, 3),
            },
            'velocity': round(self.velocity, 3),
            'odom_distance_m': round(self.odom_distance_m, 3),
            'storage_fill': storage_fill,
            'storage_count': self.storage_count,
            'storage_capacity': self.max_capacity,
            'fuel_level': fuel_level,
            'at_base': at_base,
            'base_pose': {
                'x': self.base_x,
                'y': self.base_y,
                'yaw': self.base_yaw,
                'status': self.base_status,
            },
        }

        msg = String()
        msg.data = json.dumps(state)
        self.state_pub.publish(msg)

        self.get_logger().info(
            f'pos=({state["pose"]["x"]}, {state["pose"]["y"]}) '
            f'fuel={fuel_level:.2f} storage={storage_fill:.2f} at_base={at_base}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
