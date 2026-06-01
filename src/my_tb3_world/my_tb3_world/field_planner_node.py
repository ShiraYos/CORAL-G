#!/usr/bin/env python3
"""
Autonomous mission planner for CORAL-G.

Reads /twin_state and /debris_density_map, decides the next action, and
publishes to /next_cell_goal (consumed by mission_planner_node).

Decision priority:
  1. fuel < threshold  → return_to_base
  2. storage full      → return_to_base
  3. no clusters left  → idle (mission complete)
  4. otherwise         → cleanup goal at highest-utility uncollected cell

Goals are only published when the decision changes, avoiding cancel/resend
churn in mission_planner_node.
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_MAX_ARENA_DIST = math.sqrt(2) * 4.0  # diagonal of the 4×4 m arena


class FieldPlannerNode(Node):
    def __init__(self):
        super().__init__('field_planner_node')

        self.declare_parameter('plan_rate_hz', 0.5)
        self.declare_parameter('fuel_return_threshold', 0.15)
        self.declare_parameter('storage_return_threshold', 1.0)

        self._fuel_thresh = float(self.get_parameter('fuel_return_threshold').value)
        self._storage_thresh = float(self.get_parameter('storage_return_threshold').value)

        self.twin_state = None
        self.density_map = None

        # Track last published decision to avoid duplicate goals
        self._last_mode: str | None = None
        self._last_target: tuple | None = None  # (x, y) for cleanup

        self.create_subscription(String, '/twin_state', self._twin_state_cb, 10)
        self.create_subscription(String, '/debris_density_map', self._density_map_cb, 10)

        self.goal_pub = self.create_publisher(String, '/next_cell_goal', 10)

        rate = float(self.get_parameter('plan_rate_hz').value)
        self.create_timer(1.0 / rate, self._plan)

        self.get_logger().info('FieldPlannerNode started')

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _twin_state_cb(self, msg: String):
        try:
            self.twin_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /twin_state')

    def _density_map_cb(self, msg: String):
        try:
            self.density_map = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /debris_density_map')

    # ── Planning ───────────────────────────────────────────────────────────────

    def _plan(self):
        if self.twin_state is None or self.density_map is None:
            return

        robot = self.twin_state.get('robot', {})
        fuel = robot.get('fuel_level', 1.0)
        storage_fill = robot.get('storage_fill', 0.0)
        robot_x = robot.get('pose', {}).get('x', 0.0)
        robot_y = robot.get('pose', {}).get('y', 0.0)

        clusters_remaining = self.density_map.get('clusters_remaining', 0)

        # ── Decide mode ──────────────────────────────────────────────────────

        if fuel < self._fuel_thresh or storage_fill >= self._storage_thresh:
            mode = 'return_to_base'
            target = None
            best_cell = None
            best_utility = 0.0

        elif clusters_remaining == 0:
            mode = 'idle'
            target = None
            best_cell = None
            best_utility = 0.0

        else:
            # Greedy nearest-highest-density cell
            best_cell = None
            best_utility = -1.0
            for cell in self.density_map.get('cells', []):
                if cell.get('density', 0.0) == 0.0:
                    continue
                dist = math.sqrt((robot_x - cell['x']) ** 2 + (robot_y - cell['y']) ** 2)
                proximity = 1.0 - min(dist / _MAX_ARENA_DIST, 1.0)
                utility = 0.7 * cell['density'] + 0.3 * proximity
                if utility > best_utility:
                    best_utility = utility
                    best_cell = cell

            if best_cell is None:
                mode = 'idle'
                target = None
            else:
                mode = 'cleanup'
                # Use actual cluster position, not cell center — cell centers can fall inside
                # wall inflation zones (e.g. wall_v1_top inflates to x=0.545, cell center at 0.5)
                target = (
                    best_cell.get('cluster_x', best_cell['x']),
                    best_cell.get('cluster_y', best_cell['y']),
                )

        # ── Only publish when the decision changes ───────────────────────────

        if mode == self._last_mode and target == self._last_target:
            return

        self._last_mode = mode
        self._last_target = target

        if mode == 'idle':
            self.get_logger().info('Mission complete — all clusters collected, going idle')
            self._publish({'mode': 'idle', 'status': 'idle'})

        elif mode == 'return_to_base':
            reason = 'fuel low' if fuel < self._fuel_thresh else 'storage full'
            self.get_logger().info(
                f'Return to base ({reason}): fuel={fuel:.2f} storage={storage_fill:.2f}'
            )
            self._publish({
                'status': 'active',
                'mode': 'return_to_base',
                'goal': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                'components': {},
                'return_feasible': True,
                'reason': 'field_planner',
            })

        else:
            waste_type = best_cell.get('waste_type', 'unknown')
            cluster_id = best_cell.get('cluster_id', '?')
            self.get_logger().info(
                f'New cleanup goal: {cluster_id} ({target[0]:.2f}, {target[1]:.2f}) '
                f'type={waste_type} utility={best_utility:.3f}'
            )
            self._publish({
                'status': 'active',
                'mode': 'cleanup',
                'goal': {'x': target[0], 'y': target[1], 'yaw': 0.0},
                'waste_type': waste_type,
                'utility': round(best_utility, 3),
                'components': {},
                'return_feasible': True,
                'reason': 'field_planner',
            })

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _publish(self, payload: dict):
        payload.update({'schema': 'dtas.next_cell_goal.v1', 'stamp': self._stamp(),
                        'source': 'field_planner_node'})
        msg = String()
        msg.data = json.dumps(payload)
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FieldPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
