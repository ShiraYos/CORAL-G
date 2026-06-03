#!/usr/bin/env python3
"""
Autonomous mission planner for CORAL-G.

Reads /twin_state and /debris_density_map, decides the next action, and
publishes to /next_cell_goal (consumed by mission_planner_node).

Decision priority:
  1. fuel < threshold  → return_to_base
  2. storage full      → return_to_base
  3. no predicted debris mass left  → idle (mission complete)
  4. otherwise         → cleanup goal at highest-utility uncollected cell

Goals are republished every republish_interval_sec even if unchanged,
so late-joining subscribers (e.g. mission_planner_node after Nav2 activation)
always receive the current decision within that window.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


_MAX_ARENA_DIST = math.sqrt(2) * 4.0  # diagonal of the 4×4 m arena


class FieldPlannerNode(Node):
    def __init__(self):
        super().__init__('field_planner_node')

        self.declare_parameter('plan_rate_hz', 0.5)
        self.declare_parameter('fuel_return_threshold', 0.25)
        self.declare_parameter('storage_return_threshold', 0.8)
        self.declare_parameter('density_reward_weight', 1.0)
        self.declare_parameter('travel_cost_weight', 0.2)
        self.declare_parameter('storage_penalty_weight', 0.5)
        self.declare_parameter('fuel_penalty_weight', 0.5)
        self.declare_parameter('map_risk_weight', 0.5)
        self.declare_parameter('return_reserve', 0.2)
        self.declare_parameter('min_density_reward', 0.1)
        self.declare_parameter('min_goal_distance_m', 0.5)
        self.declare_parameter('republish_interval_sec', 10.0)
        self.declare_parameter('lock_timeout_sec', 130.0)

        self._fuel_thresh = float(self.get_parameter('fuel_return_threshold').value)
        self._storage_thresh = float(self.get_parameter('storage_return_threshold').value)
        self._density_weight = float(self.get_parameter('density_reward_weight').value)
        self._travel_weight = float(self.get_parameter('travel_cost_weight').value)
        self._storage_weight = float(self.get_parameter('storage_penalty_weight').value)
        self._fuel_weight = float(self.get_parameter('fuel_penalty_weight').value)
        self._map_risk_weight = float(self.get_parameter('map_risk_weight').value)
        self._return_reserve = float(self.get_parameter('return_reserve').value)
        self._min_density_reward = float(self.get_parameter('min_density_reward').value)
        self._min_goal_distance = float(self.get_parameter('min_goal_distance_m').value)
        self._republish_interval = float(self.get_parameter('republish_interval_sec').value)
        self._lock_timeout_sec = float(self.get_parameter('lock_timeout_sec').value)

        self.twin_state = None
        self.density_map = None

        # Track last published decision to avoid duplicate goals
        self._last_mode: str | None = None
        self._last_target: tuple | None = None  # (x, y) for cleanup
        self._last_publish_time: float = 0.0

        # Lock onto a target until collection confirms it was picked up.
        # Uses position-based unlock since density cells carry no cluster_id.
        self._locked_target = None  # (cluster_id, x, y)
        self._lock_acquired_at = None   # time.monotonic() when lock was set
        self._collected_ids: set = set()

        self.create_subscription(String, '/twin_state', self._twin_state_cb, 10)
        self.create_subscription(String, '/debris_density_map', self._density_map_cb, 10)
        self.create_subscription(String, '/collection_event', self._collection_cb, 10)

        goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_pub = self.create_publisher(String, '/next_cell_goal', goal_qos)

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

    def _collection_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        cid = event.get('cluster_id')
        if cid:
            self._collected_ids.add(cid)
            if self._locked_target and self._locked_target[0] == cid:
                self.get_logger().info(f'Target {cid} collected — unlocking')
                self._locked_target = None
                self._lock_acquired_at = None
                self._last_mode = None
                self._last_target = None
        elif self._locked_target is not None and self._locked_target[0] is None:
            # Density cells carry no cluster_id — unlock by proximity to locked position
            loc = event.get('location', {})
            lx = loc.get('x', float('inf'))
            ly = loc.get('y', float('inf'))
            _, tx, ty = self._locked_target
            if math.sqrt((lx - tx) ** 2 + (ly - ty) ** 2) < 1.0:
                self.get_logger().info('Collection near locked target — unlocking')
                self._locked_target = None
                self._lock_acquired_at = None
                self._last_mode = None
                self._last_target = None

    # ── Planning ───────────────────────────────────────────────────────────────

    def _density_cells(self):
        return self.density_map.get('density_cells') or self.density_map.get('cells', [])

    def _remaining_debris_mass(self):
        counts = self.density_map.get('prediction_counts') or {}
        if 'active' in counts:
            return int(counts.get('active') or 0)
        if 'prediction_particle_count' in self.density_map:
            return int(self.density_map.get('prediction_particle_count') or 0)
        if 'clusters_remaining' in self.density_map:
            return int(self.density_map.get('clusters_remaining') or 0)
        return sum(1 for cell in self._density_cells() if cell.get('density', 0.0) > 0.0)

    def _base_pose(self):
        base = self.twin_state.get('base', {})
        pose = base.get('pose', {})
        return (
            float(pose.get('x', 0.0)),
            float(pose.get('y', 0.0)),
            float(pose.get('yaw', 0.0)),
        )

    def _map_confidence(self):
        twin_map = self.twin_state.get('map', {})
        return max(0.0, min(float(twin_map.get('known_area_ratio', 1.0)), 1.0))

    def _map_cell_lookup(self):
        cells = self.twin_state.get('map', {}).get('cells', [])
        return {
            (round(cell.get('x', 0.0), 3), round(cell.get('y', 0.0), 3)): cell
            for cell in cells
        }

    def _is_allowed_density_cell(self, cell, map_lookup):
        if not map_lookup:
            return True
        twin_cell = map_lookup.get((round(cell.get('x', 0.0), 3), round(cell.get('y', 0.0), 3)))
        return bool(twin_cell and twin_cell.get('occupancy') == 'free')

    def _normalized_distance(self, ax, ay, bx, by):
        return min(math.sqrt((ax - bx) ** 2 + (ay - by) ** 2) / _MAX_ARENA_DIST, 1.0)

    def _score_cell(self, cell, robot_x, robot_y, base_x, base_y, fuel, storage_fill, map_confidence):
        density = max(0.0, min(float(cell.get('density', 0.0)), 1.0))
        density_reward = self._density_weight * density
        if density_reward < self._min_density_reward:
            return None

        # Skip cells the robot is already standing on
        actual_dist = math.sqrt((robot_x - cell['x']) ** 2 + (robot_y - cell['y']) ** 2)
        if actual_dist < self._min_goal_distance:
            return None

        travel_distance = self._normalized_distance(robot_x, robot_y, cell['x'], cell['y'])
        return_distance = self._normalized_distance(cell['x'], cell['y'], base_x, base_y)
        fuel_margin = fuel - return_distance - self._return_reserve
        return_feasible = fuel_margin >= 0.0
        if not return_feasible:
            return None

        travel_cost = self._travel_weight * travel_distance
        storage_penalty = self._storage_weight * max(0.0, min(float(storage_fill), 1.0))
        fuel_penalty = self._fuel_weight * max(0.0, 1.0 - float(fuel))
        map_risk = self._map_risk_weight * (1.0 - map_confidence)
        utility = density_reward - travel_cost - storage_penalty - fuel_penalty - map_risk

        return {
            'utility': round(utility, 3),
            'return_feasible': return_feasible,
            'components': {
                'density_reward': round(density_reward, 3),
                'travel_cost': round(travel_cost, 3),
                'storage_penalty': round(storage_penalty, 3),
                'fuel_penalty': round(fuel_penalty, 3),
                'map_risk': round(map_risk, 3),
                'return_cost': round(return_distance, 3),
                'fuel_margin': round(fuel_margin, 3),
            },
        }

    def _plan(self):
        if self.twin_state is None or self.density_map is None:
            self.get_logger().info(
                f'Waiting for data: twin_state={self.twin_state is not None} '
                f'density_map={self.density_map is not None}',
                throttle_duration_sec=5.0,
            )
            return

        # Don't plan until all digital twin inputs are live — prevents sending
        # goals before AMCL has converged at startup.
        sync_status = self.twin_state.get('sync_status', 'unknown')
        if sync_status != 'ready':
            self.get_logger().info(
                f'Waiting for system ready: {sync_status}',
                throttle_duration_sec=5.0,
            )
            return

        # Hold current target until _collection_cb clears it via proximity match,
        # or until lock_timeout_sec elapses (handles Nav2 failure/cancel with no
        # collection event — without this the planner freezes permanently).
        if self._locked_target is not None:
            if (self._lock_acquired_at is not None and
                    time.monotonic() - self._lock_acquired_at > self._lock_timeout_sec):
                self.get_logger().warn(
                    f'Goal lock timed out after {self._lock_timeout_sec:.0f}s — '
                    f'force-unlocking ({self._locked_target[1]:.2f}, {self._locked_target[2]:.2f})'
                )
                self._locked_target = None
                self._lock_acquired_at = None
                self._last_mode = None
                self._last_target = None
            else:
                return

        robot = self.twin_state.get('robot', {})
        fuel = robot.get('fuel_level', 1.0)
        storage_fill = robot.get('storage_fill', 0.0)
        robot_x = robot.get('pose', {}).get('x', 0.0)
        robot_y = robot.get('pose', {}).get('y', 0.0)
        base_x, base_y, base_yaw = self._base_pose()
        map_confidence = self._map_confidence()

        remaining_debris_mass = self._remaining_debris_mass()
        self.get_logger().info(
            f'Plan tick: fuel={fuel:.2f} storage={storage_fill:.2f} '
            f'debris_mass={remaining_debris_mass} '
            f'density_cells={len(self._density_cells())} '
            f'map_cells={len(self.twin_state.get("map", {}).get("cells", []))}',
            throttle_duration_sec=5.0,
        )

        # ── Decide mode ──────────────────────────────────────────────────────

        if fuel < self._fuel_thresh or storage_fill >= self._storage_thresh:
            mode = 'return_to_base'
            target = (base_x, base_y)
            best_cell = None
            best_utility = 0.0
            best_score = {
                'utility': 0.0,
                'return_feasible': True,
                'components': {
                    'density_reward': 0.0,
                    'travel_cost': 0.0,
                    'storage_penalty': round(self._storage_weight * storage_fill, 3),
                    'fuel_penalty': round(self._fuel_weight * max(0.0, 1.0 - fuel), 3),
                    'map_risk': round(self._map_risk_weight * (1.0 - map_confidence), 3),
                    'return_cost': round(
                        self._normalized_distance(robot_x, robot_y, base_x, base_y), 3,
                    ),
                    'fuel_margin': round(fuel - self._return_reserve, 3),
                },
            }

        elif remaining_debris_mass == 0:
            mode = 'idle'
            target = None
            best_cell = None
            best_utility = 0.0
            best_score = None

        else:
            best_cell = None
            best_utility = -1.0
            best_score = None
            map_lookup = self._map_cell_lookup()
            for cell in self._density_cells():
                if not self._is_allowed_density_cell(cell, map_lookup):
                    continue
                score = self._score_cell(
                    cell, robot_x, robot_y, base_x, base_y,
                    fuel, storage_fill, map_confidence,
                )
                if score is None:
                    continue
                utility = score['utility']
                if utility > best_utility:
                    best_utility = utility
                    best_cell = cell
                    best_score = score

            if best_cell is None:
                mode = 'idle'
                target = None
            else:
                mode = 'cleanup'
                target = (best_cell['x'], best_cell['y'])
                self._locked_target = (best_cell.get('cluster_id'), target[0], target[1])
                self._lock_acquired_at = time.monotonic()

        # ── Publish when decision changes OR republish interval has elapsed ──

        now = self.get_clock().now().nanoseconds / 1e9
        decision_unchanged = (mode == self._last_mode and target == self._last_target)
        time_to_republish = (now - self._last_publish_time) >= self._republish_interval

        if decision_unchanged and not time_to_republish:
            return

        self._last_mode = mode
        self._last_target = target
        self._last_publish_time = now

        if mode == 'idle':
            self.get_logger().info('Mission complete — no predicted debris mass, going idle')
            self._publish({'mode': 'idle', 'status': 'idle'})

        elif mode == 'return_to_base':
            reason = 'fuel low' if fuel < self._fuel_thresh else 'storage full'
            self.get_logger().info(
                f'Return to base ({reason}): fuel={fuel:.2f} storage={storage_fill:.2f}'
            )
            self._publish({
                'status': 'active',
                'mode': 'return_to_base',
                'goal': {
                    'frame_id': 'map',
                    'x': base_x,
                    'y': base_y,
                    'yaw': base_yaw,
                },
                'utility': best_score['utility'],
                'components': best_score['components'],
                'return_feasible': best_score['return_feasible'],
                'reason': reason,
            })

        else:
            self.get_logger().info(
                f'New cleanup goal: density_cell ({target[0]:.2f}, {target[1]:.2f}) '
                f'utility={best_utility:.3f}'
            )
            self._publish({
                'status': 'selected',
                'mode': 'cleanup',
                'goal': {
                    'frame_id': 'map',
                    'x': target[0],
                    'y': target[1],
                    'yaw': 0.0,
                },
                'waste_type': best_cell.get('waste_type', 'unknown'),
                'utility': best_score['utility'],
                'components': best_score['components'],
                'return_feasible': best_score['return_feasible'],
                'reason': 'highest utility reachable cell with return reserve',
            })

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _publish(self, payload: dict):
        payload.update({
            'schema': 'dtas.next_cell_goal.v1',
            'stamp': self._stamp(),
            'source': 'field_planner_node',
        })
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
