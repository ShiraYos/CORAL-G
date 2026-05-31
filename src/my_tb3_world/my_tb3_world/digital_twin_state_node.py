#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


ARENA_MIN = -2.0
ARENA_MAX = 2.0
CELL_SIZE = 1.0
MAX_COLLECTION_HISTORY = 20


class DigitalTwinStateNode(Node):
    def __init__(self):
        super().__init__('digital_twin_state_node')

        self.declare_parameter('publish_rate_hz', 1.0)

        # Input state
        self.robot_state = None
        self.environment_observation = None
        self.base_pose = None
        self.map_cells = []          # arena cells derived from OccupancyGrid
        self.map_received = False
        self.collection_events = []  # rolling history
        self.material_evidence = {'A': 0, 'B': 0, 'C': 0}

        # /map uses TRANSIENT_LOCAL so we receive it even if published before we start
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(String, '/robot_state', self._robot_state_cb, 10)
        self.create_subscription(String, '/environment_observation', self._env_obs_cb, 10)
        self.create_subscription(String, '/base_pose', self._base_pose_cb, 10)
        self.create_subscription(String, '/collection_event', self._collection_event_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)

        self.twin_pub = self.create_publisher(String, '/twin_state', 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('DigitalTwinStateNode started')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _robot_state_cb(self, msg: String):
        try:
            self.robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /robot_state')

    def _env_obs_cb(self, msg: String):
        try:
            self.environment_observation = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /environment_observation')

    def _base_pose_cb(self, msg: String):
        try:
            self.base_pose = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /base_pose')

    def _collection_event_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /collection_event')
            return

        self.collection_events.append(event)
        if len(self.collection_events) > MAX_COLLECTION_HISTORY:
            self.collection_events.pop(0)

        # Accumulate observed material evidence
        for mat, count in event.get('materials', {}).items():
            if mat in self.material_evidence:
                self.material_evidence[mat] += count

        self.get_logger().info(
            f'Collection event recorded — material_evidence so far: {self.material_evidence}'
        )

    def _map_cb(self, msg: OccupancyGrid):
        """Convert OccupancyGrid to arena 1m grid cells."""
        cells = []
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        width = msg.info.width

        x = ARENA_MIN + CELL_SIZE / 2.0
        while x < ARENA_MAX:
            y = ARENA_MIN + CELL_SIZE / 2.0
            while y < ARENA_MAX:
                gx = int((x - ox) / res)
                gy = int((y - oy) / res)

                if 0 <= gx < msg.info.width and 0 <= gy < msg.info.height:
                    val = msg.data[gy * width + gx]
                    if val == 0:
                        occupancy = 'free'
                    elif val == -1:
                        occupancy = 'unknown'
                    else:
                        occupancy = 'blocked'
                else:
                    occupancy = 'unknown'

                cells.append({
                    'x': round(x, 3),
                    'y': round(y, 3),
                    'occupancy': occupancy,
                })
                y += CELL_SIZE
            x += CELL_SIZE

        self.map_cells = cells
        self.map_received = True
        free = sum(1 for c in cells if c['occupancy'] == 'free')
        blocked = sum(1 for c in cells if c['occupancy'] == 'blocked')
        self.get_logger().info(f'Map updated: {free} free, {blocked} blocked cells')

    # ── Publish ────────────────────────────────────────────────────────────────

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _sync_status(self) -> str:
        missing = []
        if self.robot_state is None:
            missing.append('robot_state')
        if self.environment_observation is None:
            missing.append('environment_observation')
        if self.base_pose is None:
            missing.append('base_pose')
        if not self.map_received:
            missing.append('map')
        return 'ready' if not missing else f'partial (waiting: {", ".join(missing)})'

    def _publish(self):
        env = self.environment_observation or {}
        robot = self.robot_state or {}
        base = self.base_pose or {'pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 'status': 'default'}

        # Merge environment data into map cells if available
        env_cells_by_pos = {}
        for cell in env.get('cells', []):
            key = (round(cell['x'], 1), round(cell['y'], 1))
            env_cells_by_pos[key] = cell

        map_cells_with_env = []
        for cell in self.map_cells:
            key = (round(cell['x'], 1), round(cell['y'], 1))
            env_data = env_cells_by_pos.get(key, {})
            merged = dict(cell)
            if cell['occupancy'] == 'free' and env_data:
                merged['current_x'] = env_data.get('current_x', 0.0)
                merged['current_y'] = env_data.get('current_y', 0.0)
                merged['wind_x'] = env_data.get('wind_x', 0.0)
                merged['wind_y'] = env_data.get('wind_y', 0.0)
                merged['wave_height'] = env_data.get('wave_height', 0.0)
            map_cells_with_env.append(merged)

        known = [c for c in self.map_cells if c['occupancy'] != 'unknown']
        blocked = [c for c in self.map_cells if c['occupancy'] == 'blocked']
        total = len(self.map_cells) if self.map_cells else 1

        twin_state = {
            'schema': 'dtas.twin_state.v1',
            'stamp': self._stamp(),
            'source': 'digital_twin_state_node',
            'sync_status': self._sync_status(),
            'map': {
                'cells': map_cells_with_env,
                'known_area_ratio': round(len(known) / total, 3),
                'obstacle_count': len(blocked),
            },
            'base': base,
            'robot': {
                'pose': robot.get('pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
                'velocity': robot.get('velocity', 0.0),
                'odom_distance_m': robot.get('odom_distance_m', 0.0),
                'fuel_level': robot.get('fuel_level', 1.0),
                'storage_fill': robot.get('storage_fill', 0.0),
                'storage_count': robot.get('storage_count', 0),
                'storage_capacity': robot.get('storage_capacity', 3),
                'at_base': robot.get('at_base', False),
            },
            'environment_status': env.get('environment_status', 'unknown'),
            'environment_source': env.get('environment_source', 'unknown'),
            'material_evidence': dict(self.material_evidence),
            'collection_events': self.collection_events[-5:],  # last 5 only in payload
        }

        out = String()
        out.data = json.dumps(twin_state)
        self.twin_pub.publish(out)

        pose = twin_state['robot']['pose']
        self.get_logger().info(
            f'[{twin_state["sync_status"]}] '
            f'pos=({pose["x"]:.2f}, {pose["y"]:.2f}) '
            f'fuel={twin_state["robot"]["fuel_level"]:.2f} '
            f'storage={twin_state["robot"]["storage_fill"]:.2f} '
            f'map_cells={len(self.map_cells)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()