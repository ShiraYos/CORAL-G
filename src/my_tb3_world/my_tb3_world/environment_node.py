#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


ARENA_MIN = -2.0
ARENA_MAX = 2.0
CELL_SIZE = 1.0

# Physical waste clusters — positions and types match new_world.world (4x4m arena)
CLUSTERS = [
    {'id': 'cluster_1', 'x':  0.88, 'y':  1.04, 'type': 'A', 'count': 4, 'collected': False},
    {'id': 'cluster_2', 'x': -1.2,  'y': -1.2,  'type': 'B', 'count': 4, 'collected': False},
    {'id': 'cluster_3', 'x': -1.2,  'y':  1.2,  'type': 'C', 'count': 3, 'collected': False},
]


class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')

        self.declare_parameter('cell_size_m', CELL_SIZE)
        self.declare_parameter('tick_rate_hz', 1.0)
        self.declare_parameter('collection_radius_m', 1.0)

        self.collection_radius = self.get_parameter('collection_radius_m').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)

        self.env_pub = self.create_publisher(String, '/environment_observation', 10)
        self.collection_pub = self.create_publisher(String, '/collection_event', 10)

        tick_rate = self.get_parameter('tick_rate_hz').value
        self.create_timer(1.0 / tick_rate, self._tick)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.pose_received = False

        self.clusters = [dict(c) for c in CLUSTERS]  # mutable copy
        self._env_cells = self._build_env_cells()

        self.get_logger().info('EnvironmentNode started')
        self.get_logger().info(
            f'Tracking {len(self.clusters)} waste clusters, '
            f'collection_radius={self.collection_radius}m'
        )

    def _build_env_cells(self):
        """Static ocean current/wind/wave grid over the arena."""
        cells = []
        x = ARENA_MIN + CELL_SIZE / 2.0
        while x < ARENA_MAX:
            y = ARENA_MIN + CELL_SIZE / 2.0
            while y < ARENA_MAX:
                cells.append({
                    'x': round(x, 3),
                    'y': round(y, 3),
                    'current_x': round(0.3 * math.cos(math.radians(45)), 3),
                    'current_y': round(0.3 * math.sin(math.radians(45)), 3),
                    'wind_x': 0.1,
                    'wind_y': 0.05,
                    'wave_height': 0.2,
                })
                y += CELL_SIZE
            x += CELL_SIZE
        return cells

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.pose_received = True

    def _check_collections(self):
        for cluster in self.clusters:
            if cluster['collected']:
                continue
            dist = math.sqrt(
                (self.robot_x - cluster['x']) ** 2 +
                (self.robot_y - cluster['y']) ** 2
            )
            if dist <= self.collection_radius:
                cluster['collected'] = True
                self._publish_collection_event(cluster)

    def _publish_collection_event(self, cluster):
        waste_type = cluster['type']
        count = cluster['count']

        materials = {'A': 0, 'B': 0, 'C': 0}
        materials[waste_type] = count

        event = {
            'schema': 'dtas.collection_event.v1',
            'stamp': self._stamp(),
            'source': 'environment_node',
            'cluster_id': cluster['id'],
            'location': {'x': cluster['x'], 'y': cluster['y']},
            'count': count,
            'items': [{'type': waste_type, 'count': count}],
            'materials': materials,
            'collection_radius_m': self.collection_radius,
            'status': 'collected',
        }

        msg = String()
        msg.data = json.dumps(event)
        self.collection_pub.publish(msg)
        self.get_logger().info(
            f'Collection event: {cluster["id"]} (Type {waste_type}, {count} items)'
        )

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _tick(self):
        if self.pose_received:
            self._check_collections()

        status = 'ready' if self.pose_received else 'waiting_for_pose'

        observation = {
            'schema': 'dtas.environment_observation.v1',
            'stamp': self._stamp(),
            'source': 'environment_node',
            'frame_id': 'map',
            'cell_size_m': CELL_SIZE,
            'environment_status': status,
            'environment_source': 'preset',
            'confidence': 0.9 if self.pose_received else 0.0,
            'cells': self._env_cells,
            'warnings': [],
        }

        msg = String()
        msg.data = json.dumps(observation)
        self.env_pub.publish(msg)

        collected = sum(1 for c in self.clusters if c['collected'])
        self.get_logger().info(
            f'env_obs published ({len(self._env_cells)} cells) | '
            f'clusters collected: {collected}/{len(self.clusters)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()