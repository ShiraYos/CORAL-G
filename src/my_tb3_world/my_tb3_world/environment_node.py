#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from my_tb3_world.environment_field import (
    DEFAULT_CELL_SIZE,
    build_environment_cells,
    observation_cells,
)
from my_tb3_world.debris_particles import (
    MATERIALS,
    advance_particle,
    debris_motion_from_parameters,
    declare_debris_motion_parameters,
    normalize_material,
    particle_counts,
    public_particle,
    seed_physical_particles,
)
from my_tb3_world_interfaces.srv import GenerateEnvironmentField


CELL_SIZE = DEFAULT_CELL_SIZE
COLLECTION_RADIUS_M = 0.2
PHYSICAL_DEBRIS_COUNT = 100
DEBRIS_DRIFT_SCALE = 0.015
MAX_RECENT_EVENTS = 30
RESET_COLLECTION_GRACE_SEC = 8.0


class EnvironmentNode(Node):
    def __init__(self):
        super().__init__('environment_node')

        self.declare_parameter('cell_size_m', CELL_SIZE)
        self.declare_parameter('tick_rate_hz', 1.0)
        self.declare_parameter('debris_drift_enabled', True)
        self.declare_parameter('debris_drift_scale', DEBRIS_DRIFT_SCALE)
        self.declare_parameter('physical_debris_seed', 23)
        declare_debris_motion_parameters(self)

        self.cell_size = self.get_parameter('cell_size_m').value
        self.collection_radius = COLLECTION_RADIUS_M
        self.debris_drift_enabled = self.get_parameter('debris_drift_enabled').value
        self.debris_drift_scale = self.get_parameter('debris_drift_scale').value
        self.physical_debris_count = PHYSICAL_DEBRIS_COUNT
        self.physical_debris_seed = self.get_parameter('physical_debris_seed').value
        self.material_response, self.boid_rules = debris_motion_from_parameters(self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(String, '/debug_reset', self._debug_reset_cb, 10)

        self.env_pub = self.create_publisher(String, '/environment_observation', 10)
        self.collection_pub = self.create_publisher(String, '/collection_event', 10)
        self.dashboard_pub = self.create_publisher(String, '/dashboard', 10)
        self.environment_client = self.create_client(
            GenerateEnvironmentField,
            'generate_environment_field',
        )

        tick_rate = self.get_parameter('tick_rate_hz').value
        self.create_timer(1.0 / tick_rate, self._tick)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.pose_received = False

        self._field_cells = build_environment_cells(None, self.cell_size)
        self._next_particle_index = 1
        self._event_counts = {'collected': 0, 'washed_out': 0, 'respawned': 0}
        self._recent_events = []
        self._collection_resume_time = self._field_time_sec() + RESET_COLLECTION_GRACE_SEC
        self.particles = self._new_physical_particles(self.physical_debris_count)
        self._env_cells = observation_cells(self._field_cells)
        self._env_source = 'preset_fallback'
        self._env_confidence = 0.9
        self._env_warnings = ['generate_environment_field not used yet']
        self._field_service_available = False
        self._pending_field_request = None
        self._seeded_from_service_field = False

        self.get_logger().info('EnvironmentNode started')
        self.get_logger().info(
            f'Tracking {len(self.particles)} hidden physical debris particles, '
            f'collection_radius={self.collection_radius}m'
        )

    def _new_physical_particles(self, count):
        particles = seed_physical_particles(
            self._field_cells,
            count,
            self.physical_debris_seed,
            self._next_particle_index,
        )
        self._next_particle_index += int(count)
        return particles

    def _reset_physical_particles(self):
        self._next_particle_index = 1
        self._event_counts = {'collected': 0, 'washed_out': 0, 'respawned': 0}
        self._recent_events = []
        self.particles = self._new_physical_particles(self.physical_debris_count)
        self.pose_received = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self._collection_resume_time = self._field_time_sec() + RESET_COLLECTION_GRACE_SEC
        self.get_logger().info('Physical debris particles reset')

    def _debug_reset_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {}
        if payload.get('scope', 'all') in ('all', 'physical'):
            self._reset_physical_particles()

    def _field_time_sec(self):
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec / 1_000_000_000.0

    def _field_time_controls(self):
        return {'field_time_sec': self._field_time_sec()}

    def _refresh_fallback_field(self):
        self._field_cells = build_environment_cells(
            None,
            self.cell_size,
            self._field_time_controls(),
        )
        self._env_cells = observation_cells(self._field_cells)

    def _build_env_cells(self):
        """Build fallback ocean current/wind/wave grid over the arena."""
        return observation_cells(build_environment_cells(None, self.cell_size))

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.pose_received = True

    def _check_collections(self):
        if self._field_time_sec() < self._collection_resume_time:
            return
        collected_particles = []
        for particle in self.particles:
            if particle['status'] != 'active':
                continue
            dist = math.sqrt(
                (self.robot_x - particle['x']) ** 2 +
                (self.robot_y - particle['y']) ** 2
            )
            if dist <= self.collection_radius:
                particle['status'] = 'collected'
                self._event_counts['collected'] += 1
                collected_particles.append(particle)

        if collected_particles:
            self._publish_collection_event(collected_particles)

    def _cell_for_position(self, x, y):
        if not self._field_cells:
            return None
        return min(
            self._field_cells,
            key=lambda cell: (x - cell['x']) ** 2 + (y - cell['y']) ** 2,
        )

    def _is_free_position(self, x, y):
        if not self._is_within_field_bounds(x, y):
            return False
        nearest = self._cell_for_position(x, y)
        return bool(nearest and nearest.get('has_environment'))

    def _is_within_field_bounds(self, x, y):
        if not self._field_cells:
            return False
        half = self.cell_size / 2.0
        min_x = min(cell['x'] for cell in self._field_cells) - half
        max_x = max(cell['x'] for cell in self._field_cells) + half
        min_y = min(cell['y'] for cell in self._field_cells) - half
        max_y = max(cell['y'] for cell in self._field_cells) + half
        return min_x <= x <= max_x and min_y <= y <= max_y

    def _advance_debris_truth(self):
        if not self.debris_drift_enabled:
            return

        for particle in self.particles:
            if particle['status'] != 'active':
                continue

            field_cell = self._cell_for_position(particle['x'], particle['y'])
            if not field_cell or not field_cell.get('has_environment'):
                particle['status'] = 'washed_out'
                self._event_counts['washed_out'] += 1
                self._record_event('washed_out', particle)
                continue

            next_state = advance_particle(
                particle,
                self.particles,
                field_cell,
                self.debris_drift_scale,
                self.material_response,
                self.boid_rules,
            )
            if not self._is_free_position(next_state['x'], next_state['y']):
                particle['status'] = 'washed_out'
                self._event_counts['washed_out'] += 1
                self._record_event('washed_out', particle)
                continue

            particle.update(next_state)

    def _respawn_to_target(self):
        active_particles = [
            particle for particle in self.particles
            if particle.get('status') == 'active'
        ]
        missing = max(0, int(self.physical_debris_count) - len(active_particles))
        if missing == 0:
            self.particles = active_particles
            return

        new_particles = self._new_physical_particles(missing)
        self.particles = active_particles + new_particles
        self._event_counts['respawned'] += missing
        self._record_event('respawned', None, material='mixed')
        self.get_logger().info(
            f'Respawned {missing} physical debris particles; '
            f'active={len(self.particles)}'
        )

    def _record_event(self, event_type, particle=None, material=None):
        event = {
            'type': event_type,
            'source': 'physical_truth',
            'stamp': self._stamp(),
            'status': event_type,
            'material': material or normalize_material((particle or {}).get('material')),
        }
        if particle:
            event['particle'] = particle.get('id')
        self._recent_events.append(event)
        self._recent_events = self._recent_events[-MAX_RECENT_EVENTS:]

    def _publish_collection_event(self, particles):
        materials = {material: 0 for material in MATERIALS}
        for particle in particles:
            material = normalize_material(particle.get('material'))
            materials[material] = materials.get(material, 0) + 1
            self._record_event('collected', particle, material)
        count = len(particles)
        location = {
            'x': round(sum(p['x'] for p in particles) / count, 3),
            'y': round(sum(p['y'] for p in particles) / count, 3),
        }
        items = [
            {'type': material, 'count': material_count}
            for material, material_count in sorted(materials.items())
            if material_count > 0
        ]

        event = {
            'schema': 'dtas.collection_event.v1',
            'stamp': self._stamp(),
            'source': 'environment_node',
            'location': location,
            'count': count,
            'items': items,
            'materials': materials,
            'collection_radius_m': self.collection_radius,
            'status': 'collected',
        }

        msg = String()
        msg.data = json.dumps(event)
        self.collection_pub.publish(msg)
        self.get_logger().info(
            f'Collection event: {count} physical debris particles'
        )

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _request_environment_field(self):
        self._field_service_available = self.environment_client.service_is_ready()
        if not self._field_service_available or self._pending_field_request is not None:
            return

        request = GenerateEnvironmentField.Request()
        request.frame_id = 'map'
        request.cell_size_m = self.cell_size
        request.generation_controls_json = json.dumps(self._field_time_controls())
        self._pending_field_request = self.environment_client.call_async(request)

    def _poll_environment_field(self):
        if self._pending_field_request is None:
            return
        if not self._pending_field_request.done():
            return

        future = self._pending_field_request
        self._pending_field_request = None
        response = future.result()
        if response is None:
            self._env_warnings = ['generate_environment_field returned no response']
            return
        if not response.success or not response.cells:
            self._env_source = 'preset_fallback'
            self._env_confidence = 0.9
            self._env_warnings = list(response.warnings)
            return

        self._field_cells = [self._field_cell_to_dict(cell) for cell in response.cells]
        self._env_cells = observation_cells(self._field_cells)
        self._env_source = response.environment_source
        self._env_confidence = response.confidence
        self._env_warnings = list(response.warnings)
        if not self._seeded_from_service_field:
            self._reset_physical_particles()
            self._seeded_from_service_field = True

    def _field_cell_to_dict(self, cell):
        return {
            'x': round(cell.x, 3),
            'y': round(cell.y, 3),
            'occupancy': cell.occupancy,
            'has_environment': cell.has_environment,
            'current_x': round(cell.current_x, 3),
            'current_y': round(cell.current_y, 3),
            'wind_x': round(cell.wind_x, 3),
            'wind_y': round(cell.wind_y, 3),
            'wave_height': round(cell.wave_height, 3),
            'confidence': round(cell.confidence, 3),
        }

    def _dashboard_environment(self):
        cells = list(self._field_cells)
        return {
            'service': 'generate_environment_field',
            'service_available': self._field_service_available,
            'request_pending': self._pending_field_request is not None,
            'status': 'ready' if cells else 'waiting_for_field',
            'source': self._env_source,
            'confidence': self._env_confidence,
            'warnings': self._env_warnings,
            'cell_size_m': self.cell_size,
            'cell_count': len(cells),
            'environment_cell_count': sum(
                1 for cell in cells if cell.get('has_environment', True)
            ),
            'blocked_cell_count': sum(
                1 for cell in cells if cell.get('occupancy') == 'blocked'
            ),
            'unknown_cell_count': sum(
                1 for cell in cells if cell.get('occupancy') == 'unknown'
            ),
            'cells': cells,
        }

    def _publish_dashboard(self, stamp, status, counts, material_counts):
        particles = [public_particle(particle) for particle in self.particles]

        msg = String()
        msg.data = json.dumps({
            'schema': 'dtas.dashboard.v1',
            'stamp': stamp,
            'source': 'environment_node',
            'frame_id': 'map',
            'status': status,
            'debug_only': True,
            'robot': {
                'pose': {
                    'x': round(self.robot_x, 3),
                    'y': round(self.robot_y, 3),
                },
                'pose_received': self.pose_received,
            },
            'collection_radius_m': self.collection_radius,
            'physical_debris': {
                'drift_enabled': self.debris_drift_enabled,
                'drift_scale': self.debris_drift_scale,
                'hidden_truth_only': True,
                'model': 'particles',
                'target_active_count': self.physical_debris_count,
                'counts': counts,
                'lifetime_counts': dict(self._event_counts),
                'material_counts': material_counts,
                'material_response': self.material_response,
                'boid_rules': self.boid_rules,
            },
            'physical_particles': particles,
            'physical_events': list(self._recent_events),
            'clusters': [],
            'environment': self._dashboard_environment(),
            'collected_count': counts['collected'],
            'remaining_count': counts['active'],
            'washed_out_count': counts['washed_out'],
        })
        self.dashboard_pub.publish(msg)

    def _tick(self):
        self.material_response, self.boid_rules = debris_motion_from_parameters(self)
        self._poll_environment_field()
        if self._env_source == 'preset_fallback' and self._field_service_available:
            self._refresh_fallback_field()
        self._request_environment_field()
        self._advance_debris_truth()

        if self.pose_received:
            self._check_collections()
        self._respawn_to_target()

        status = 'ready' if self.pose_received else 'waiting_for_pose'
        stamp = self._stamp()

        observation = {
            'schema': 'dtas.environment_observation.v1',
            'stamp': stamp,
            'source': 'environment_node',
            'frame_id': 'map',
            'cell_size_m': self.cell_size,
            'environment_status': status,
            'environment_source': self._env_source,
            'confidence': self._env_confidence if self.pose_received else 0.0,
            'cells': self._env_cells,
            'warnings': [],
        }

        msg = String()
        msg.data = json.dumps(observation)
        self.env_pub.publish(msg)

        active_counts, material_counts = particle_counts(self.particles)
        counts = dict(active_counts)
        counts['collected'] = self._event_counts['collected']
        counts['washed_out'] = self._event_counts['washed_out']
        counts['respawned'] = self._event_counts['respawned']
        self._publish_dashboard(stamp, status, counts, material_counts)

        self.get_logger().info(
            f'env_obs published ({len(self._env_cells)} cells) | '
            f'particles active={counts["active"]} '
            f'collected={counts["collected"]} washed_out={counts["washed_out"]}'
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
