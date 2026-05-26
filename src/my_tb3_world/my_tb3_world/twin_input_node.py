#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String


ARENA_MIN = -3.5
ARENA_MAX = 3.5
CELL_SIZE = 1.0


def stamp_now(node):
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


def world_to_cell(wx, wy):
    cx = math.floor(wx / CELL_SIZE) * CELL_SIZE + CELL_SIZE / 2.0
    cy = math.floor(wy / CELL_SIZE) * CELL_SIZE + CELL_SIZE / 2.0
    return round(cx, 3), round(cy, 3)


def in_arena(cx, cy):
    return ARENA_MIN < cx < ARENA_MAX and ARENA_MIN < cy < ARENA_MAX


class TwinInputNode(Node):
    def __init__(self):
        super().__init__('twin_input_node')

        self.declare_parameter('cell_size_m', CELL_SIZE)
        self.declare_parameter('tick_rate_hz', 1.0)
        self.declare_parameter('zone_width_m', 7.0)
        self.declare_parameter('zone_height_m', 7.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos)

        self.env_pub = self.create_publisher(String, '/coral_g/environment_field', 10)
        self.map_pub = self.create_publisher(String, '/coral_g/map_sync_update', 10)

        tick_rate = self.get_parameter('tick_rate_hz').value
        self.timer = self.create_timer(1.0 / tick_rate, self.publish_environment_field)

        # Robot pose state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.pose_received = False

        # Latest scan-derived map evidence
        self.latest_obstacles = []
        self.latest_free_cells = []
        self.scan_received = False

        self.env_field = self._build_preset_environment()

        self.get_logger().info('TwinInputNode started')
        self.get_logger().info(f'Publishing environment_field at {tick_rate} Hz')

    def _build_preset_environment(self):
        """Build a static mock ocean current/wind/wave grid over the arena."""
        cells = []
        x = ARENA_MIN + CELL_SIZE / 2.0
        while x < ARENA_MAX:
            y = ARENA_MIN + CELL_SIZE / 2.0
            while y < ARENA_MAX:
                cx, cy = round(x, 3), round(y, 3)
                # Mock ocean current: gentle flow toward +x, +y corner
                current_mag = 0.3
                current_angle = math.radians(45)
                cells.append({
                    'x': cx,
                    'y': cy,
                    'current_x': round(current_mag * math.cos(current_angle), 3),
                    'current_y': round(current_mag * math.sin(current_angle), 3),
                    'wind_x': 0.1,
                    'wind_y': 0.05,
                    'wave_height': 0.2,
                })
                y += CELL_SIZE
            x += CELL_SIZE

        return {
            'schema': 'coral_g.environment_field.v1',
            'source': 'twin_input_node',
            'mode': 'preset',
            'frame_id': 'map',
            'cell_size_m': CELL_SIZE,
            'zone_width_m': self.get_parameter('zone_width_m').value,
            'zone_height_m': self.get_parameter('zone_height_m').value,
            'cells': cells,
        }

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_heading = math.atan2(siny_cosp, cosy_cosp)

        self.pose_received = True

    def scan_callback(self, msg: LaserScan):
        obstacles = set()
        free_cells = set()

        angle = msg.angle_min
        for r in msg.ranges:
            angle_norm = math.atan2(math.sin(angle), math.cos(angle))

            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                # Obstacle at end of ray
                world_angle = self.robot_heading + angle_norm
                ox = self.robot_x + r * math.cos(world_angle)
                oy = self.robot_y + r * math.sin(world_angle)
                cx, cy = world_to_cell(ox, oy)
                if in_arena(cx, cy):
                    obstacles.add((cx, cy))

                # Free cells along the ray (sample every 0.5 m)
                steps = int(r / 0.5)
                for s in range(1, steps):
                    d = s * 0.5
                    fx = self.robot_x + d * math.cos(world_angle)
                    fy = self.robot_y + d * math.sin(world_angle)
                    fcx, fcy = world_to_cell(fx, fy)
                    if in_arena(fcx, fcy) and (fcx, fcy) not in obstacles:
                        free_cells.add((fcx, fcy))

            angle += msg.angle_increment

        self.latest_obstacles = [{'x': c[0], 'y': c[1]} for c in obstacles]
        self.latest_free_cells = [{'x': c[0], 'y': c[1]} for c in free_cells - obstacles]
        self.scan_received = True

        self._publish_map_sync()

    def _publish_map_sync(self):
        if not self.pose_received:
            status = 'waiting_for_pose'
        elif not self.scan_received:
            status = 'waiting_for_scan'
        else:
            status = 'ready'

        confidence = 0.7 if self.scan_received and self.pose_received else 0.0

        msg = {
            'schema': 'coral_g.map_sync_update.v1',
            'stamp': stamp_now(self),
            'source': 'twin_input_node',
            'frame_id': 'map',
            'robot_pose': {
                'x': round(self.robot_x, 3),
                'y': round(self.robot_y, 3),
                'heading': round(self.robot_heading, 3),
            },
            'observed_obstacles': self.latest_obstacles,
            'free_space_cells': self.latest_free_cells,
            'map_confidence': confidence,
            'status': status,
        }

        out = String()
        out.data = json.dumps(msg)
        self.map_pub.publish(out)

    def publish_environment_field(self):
        self.env_field['stamp'] = stamp_now(self)
        out = String()
        out.data = json.dumps(self.env_field)
        self.env_pub.publish(out)

        self.get_logger().info(
            f'env_field published ({len(self.env_field["cells"])} cells) | '
            f'obstacles={len(self.latest_obstacles)} free={len(self.latest_free_cells)} '
            f'scan={"yes" if self.scan_received else "no"}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwinInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
