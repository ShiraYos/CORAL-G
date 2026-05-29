#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DigitalTwinStateNode(Node):
    def __init__(self):
        super().__init__('digital_twin_state_node')

        self.declare_parameter('publish_rate_hz', 1.0)

        self.robot_state = None
        self.environment_field = None
        self.map_sync = None

        self.create_subscription(
            String, '/coral_g/robot_state', self._robot_state_cb, 10)
        self.create_subscription(
            String, '/coral_g/environment_field', self._env_field_cb, 10)
        self.create_subscription(
            String, '/coral_g/map_sync_update', self._map_sync_cb, 10)

        self.twin_pub = self.create_publisher(String, '/coral_g/twin_state', 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('DigitalTwinStateNode started')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _robot_state_cb(self, msg: String):
        try:
            self.robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /coral_g/robot_state')

    def _env_field_cb(self, msg: String):
        try:
            self.environment_field = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /coral_g/environment_field')

    def _map_sync_cb(self, msg: String):
        try:
            self.map_sync = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /coral_g/map_sync_update')

    # ── Publish ───────────────────────────────────────────────────────────────

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _status(self) -> str:
        if self.robot_state is None:
            return 'waiting_for_robot_state'
        if self.environment_field is None:
            return 'waiting_for_environment'
        if self.map_sync is None:
            return 'waiting_for_map'
        return 'ready'

    def _publish(self):
        status = self._status()

        env = self.environment_field or {}
        mp = self.map_sync or {}
        robot = self.robot_state or {}

        twin_state = {
            'schema': 'coral_g.twin_state.v1',
            'stamp': self._stamp(),
            'source': 'digital_twin_state_node',
            'status': status,
            'robot': {
                'pose': robot.get('pose', {'x': 0.0, 'y': 0.0, 'heading': 0.0}),
                'velocity': robot.get('velocity', 0.0),
                'fuel_level': robot.get('fuel_level', 1.0),
                'storage_fill': robot.get('storage_fill', 0.0),
                'storage_count': robot.get('storage_count', 0),
                'storage_capacity': robot.get('storage_capacity', 3),
                'at_base': robot.get('at_base', False),
                'navigation_status': robot.get('navigation_status', ''),
            },
            'environment': {
                'mode': env.get('mode', 'unknown'),
                'cell_size_m': env.get('cell_size_m', 1.0),
                'cells': env.get('cells', []),
            },
            'map': {
                'observed_obstacles': mp.get('observed_obstacles', []),
                'free_space_cells': mp.get('free_space_cells', []),
                'map_confidence': mp.get('map_confidence', 0.0),
                'status': mp.get('status', 'unknown'),
            },
        }

        out = String()
        out.data = json.dumps(twin_state)
        self.twin_pub.publish(out)

        pose = twin_state['robot']['pose']
        self.get_logger().info(
            f'[{status}] '
            f'pos=({pose["x"]:.2f}, {pose["y"]:.2f}) '
            f'fuel={twin_state["robot"]["fuel_level"]:.2f} '
            f'storage={twin_state["robot"]["storage_fill"]:.2f} '
            f'obstacles={len(twin_state["map"]["observed_obstacles"])}'
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
