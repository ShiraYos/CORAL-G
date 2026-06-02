#!/usr/bin/env python3

import json
import math
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from my_tb3_world.debris_particles import (
    MATERIALS,
    UNKNOWN_MATERIAL,
    advance_particle,
    debris_motion_from_parameters,
    declare_debris_motion_parameters,
    normalize_material,
)
from my_tb3_world.environment_field import (
    DEFAULT_CELL_SIZE,
    fallback_bounds,
    simulation_cell_centers,
)

CELL_SIZE = DEFAULT_CELL_SIZE
PREDICTION_MATERIALS = (UNKNOWN_MATERIAL,) + MATERIALS
MAX_RECENT_EVENTS = 20
COLLECTION_UPDATE_SIGMA_M = CELL_SIZE
MATERIAL_REASSIGN_FRACTION = 0.25
PREDICTION_DRIFT_SCALE = 0.015


def _nearest_cell(cells, x, y):
    if not cells:
        return None
    return min(cells, key=lambda cell: (x - cell['x']) ** 2 + (y - cell['y']) ** 2)


def _is_within_cell_bounds(cells, x, y, cell_size_m):
    if not cells:
        return False
    half = cell_size_m / 2.0
    min_x = min(cell['x'] for cell in cells) - half
    max_x = max(cell['x'] for cell in cells) + half
    min_y = min(cell['y'] for cell in cells) - half
    max_y = max(cell['y'] for cell in cells) + half
    return min_x <= x <= max_x and min_y <= y <= max_y


def _fallback_density_cells():
    return [
        {
            'x': x,
            'y': y,
            'occupancy': 'free',
            'density': 0.0,
        }
        for x, y in simulation_cell_centers(fallback_bounds(), CELL_SIZE)
    ]


def _free_cells(cells):
    return [cell for cell in cells if cell.get('occupancy') == 'free']


def _material_from_weights(index, weights):
    weights = weights or {material: 1.0 for material in PREDICTION_MATERIALS}
    weighted = [
        (material, max(0.0, float(weights.get(material, 0.0))))
        for material in PREDICTION_MATERIALS
    ]
    total = sum(weight for _, weight in weighted)
    if total <= 0.0:
        return UNKNOWN_MATERIAL

    target = (index * 0.61803398875 % 1.0) * total
    running = 0.0
    for material, weight in weighted:
        running += weight
        if target <= running:
            return material
    return weighted[-1][0]


def seed_prediction_particles(cells, count=100, random_seed=23, start_index=1, material_weights=None):
    """Seed independent digital-belief particles from valid free cells."""
    free = _free_cells(cells) or _fallback_density_cells()
    rng = random.Random(f'{random_seed}:{start_index}:{count}')
    particles = []
    shuffled_free = list(free)
    rng.shuffle(shuffled_free)
    for offset in range(int(count)):
        index = start_index + offset
        cell = shuffled_free[offset % len(shuffled_free)]
        jitter_x = rng.uniform(-CELL_SIZE * 0.35, CELL_SIZE * 0.35)
        jitter_y = rng.uniform(-CELL_SIZE * 0.35, CELL_SIZE * 0.35)
        particles.append({
            'id': f'prediction_particle_{index:03d}',
            'material': _material_from_weights(index, material_weights),
            'x': round(cell['x'] + jitter_x, 3),
            'y': round(cell['y'] + jitter_y, 3),
            'initial_x': cell['x'],
            'initial_y': cell['y'],
            'vx': 0.0,
            'vy': 0.0,
            'ax': 0.0,
            'ay': 0.0,
            'status': 'active',
        })
    return particles


class DebrisPredictionNode(Node):
    def __init__(self):
        super().__init__('debris_prediction_node')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('prediction_debris_count', 100)
        self.declare_parameter('simulation_horizon_sec', 60.0)
        self.declare_parameter('random_seed', 23)
        self.declare_parameter('prediction_drift_enabled', True)
        self.declare_parameter('prediction_drift_scale', PREDICTION_DRIFT_SCALE)
        declare_debris_motion_parameters(self)

        self.prediction_debris_count = self.get_parameter('prediction_debris_count').value
        self.simulation_horizon_sec = self.get_parameter('simulation_horizon_sec').value
        self.random_seed = self.get_parameter('random_seed').value
        self.prediction_drift_enabled = self.get_parameter(
            'prediction_drift_enabled'
        ).value
        self.prediction_drift_scale = self.get_parameter(
            'prediction_drift_scale'
        ).value
        self.material_response, self.boid_rules = debris_motion_from_parameters(self)

        self.twin_map_cells = []
        self.material_evidence = {material: 0 for material in MATERIALS}
        self._material_weights = self._material_belief_weights()
        self._next_particle_index = 1
        self._event_counts = {'observed_removed': 0, 'washed_out': 0, 'respawned': 0}
        self._recent_events = []
        self._belief_cells = {}
        self.prediction_particles = self._new_prediction_particles(self.prediction_debris_count)
        self._seeded_from_twin_map = False
        self._processed_collection_keys = set()
        self._initialize_belief_from_particles()

        self.create_subscription(String, '/twin_state', self._twin_state_cb, 10)
        self.create_subscription(String, '/collection_event', self._collection_event_cb, 10)
        self.create_subscription(String, '/debug_reset', self._debug_reset_cb, 10)

        self.density_pub = self.create_publisher(String, '/debris_density_map', 10)
        self.prediction_dashboard_pub = self.create_publisher(
            String,
            '/prediction_dashboard',
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'DebrisPredictionNode: tracking {len(self.prediction_particles)} prediction particles'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _source_cells(self):
        return self.twin_map_cells or _fallback_density_cells()

    def _new_prediction_particles(self, count):
        particles = seed_prediction_particles(
            self._source_cells(),
            count,
            self.random_seed,
            self._next_particle_index,
            self._material_weights,
        )
        self._next_particle_index += int(count)
        return particles

    def _record_event(self, event_type, count, location=None):
        if count <= 0:
            return
        event = {
            'type': event_type,
            'count': int(count),
            'stamp': self._stamp(),
        }
        if location:
            event['location'] = {
                'x': round(location.get('x', 0.0), 3),
                'y': round(location.get('y', 0.0), 3),
            }
        self._recent_events.append(event)
        self._recent_events = self._recent_events[-MAX_RECENT_EVENTS:]

    def _reset_prediction(self):
        self._next_particle_index = 1
        self._event_counts = {'observed_removed': 0, 'washed_out': 0, 'respawned': 0}
        self._recent_events = []
        self.material_evidence = {material: 0 for material in MATERIALS}
        self._material_weights = self._material_belief_weights()
        self.prediction_particles = self._new_prediction_particles(self.prediction_debris_count)
        self._initialize_belief_from_particles()
        self._processed_collection_keys = set()
        self.get_logger().info('Prediction particles reset')

    def _debug_reset_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {}
        if payload.get('scope', 'all') in ('all', 'prediction'):
            self._reset_prediction()

    def _twin_state_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Bad JSON on /twin_state')
            return

        self.twin_map_cells = data.get('map', {}).get('cells', [])
        if self.twin_map_cells and not self._seeded_from_twin_map:
            self._reset_prediction()
            self._seeded_from_twin_map = True
        if 'material_evidence' in data:
            self._set_material_evidence(data.get('material_evidence', {}))

        for event in data.get('collection_events', []):
            self._apply_collection_event(event)

    def _collection_event_cb(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._apply_collection_event(event)

    # ── Prediction ─────────────────────────────────────────────────────────────

    def _material_belief_weights(self):
        weights = {UNKNOWN_MATERIAL: 1.0}
        for material in MATERIALS:
            weights[material] = max(0, int(self.material_evidence.get(material, 0)))
        return weights

    def _set_material_evidence(self, evidence):
        next_evidence = {material: 0 for material in MATERIALS}
        for material, count in (evidence or {}).items():
            normalized = normalize_material(material)
            if normalized in next_evidence:
                next_evidence[normalized] += max(0, int(count))
        if next_evidence == self.material_evidence:
            return

        self.material_evidence = next_evidence
        self._material_weights = self._material_belief_weights()
        self._apply_material_belief_to_existing_particles()

    def _add_material_evidence_from_event(self, event):
        materials = event.get('materials', {})
        if not materials:
            return
        evidence = dict(self.material_evidence)
        for material, count in materials.items():
            normalized = normalize_material(material)
            if normalized in evidence:
                evidence[normalized] += max(0, int(count))
        self._set_material_evidence(evidence)

    def _material_belief_summary(self):
        weights = dict(self._material_weights)
        total = sum(weights.values()) or 1.0
        return {
            'prior': UNKNOWN_MATERIAL,
            'evidence': dict(self.material_evidence),
            'weights': {key: round(value, 3) for key, value in weights.items()},
            'probabilities': {
                key: round(value / total, 3)
                for key, value in weights.items()
            },
            'debug_only': True,
        }

    def _apply_material_belief_to_existing_particles(self):
        active = [
            particle for particle in self.prediction_particles
            if particle.get('status') == 'active'
        ]
        if not active:
            return
        limit = max(1, int(len(active) * MATERIAL_REASSIGN_FRACTION))
        for offset, particle in enumerate(active[:limit]):
            particle['material'] = _material_from_weights(
                self._next_particle_index + offset,
                self._material_weights,
            )

    def _free_cell_map(self):
        cells = {}
        for cell in self._source_cells():
            if cell.get('occupancy') != 'free':
                continue
            key = (round(cell['x'], 3), round(cell['y'], 3))
            cells[key] = {
                'x': key[0],
                'y': key[1],
                'density': 0.0,
            }
        return cells

    def _normalize_belief(self):
        total = sum(max(0.0, cell.get('density', 0.0)) for cell in self._belief_cells.values())
        if total <= 0.0:
            return
        for cell in self._belief_cells.values():
            cell['density'] = round(max(0.0, cell.get('density', 0.0)) / total, 6)

    def _initialize_belief_from_particles(self):
        self._belief_cells = self._free_cell_map()
        if not self._belief_cells:
            return

        active = [
            particle for particle in self.prediction_particles
            if particle.get('status') == 'active'
        ]
        if not active:
            mass = 1.0 / len(self._belief_cells)
            for cell in self._belief_cells.values():
                cell['density'] = mass
            self._normalize_belief()
            return

        lookup_cells = list(self._belief_cells.values())
        mass = 1.0 / len(active)
        for particle in active:
            target_cell = _nearest_cell(lookup_cells, particle['x'], particle['y'])
            if target_cell is None:
                continue
            key = (target_cell['x'], target_cell['y'])
            self._belief_cells[key]['density'] += mass
        self._normalize_belief()

    def _ensure_belief_cells(self):
        free_cells = self._free_cell_map()
        if not free_cells:
            self._belief_cells = {}
            return
        if set(free_cells.keys()) != set(self._belief_cells.keys()):
            self._initialize_belief_from_particles()
            return
        for key, cell in free_cells.items():
            self._belief_cells[key]['x'] = cell['x']
            self._belief_cells[key]['y'] = cell['y']

    def _collection_key(self, event):
        location = event.get('location', {})
        return (
            event.get('stamp', ''),
            round(location.get('x', 0.0), 3),
            round(location.get('y', 0.0), 3),
            int(event.get('count', 1)),
        )

    def _apply_collection_event(self, event):
        key = self._collection_key(event)
        if key in self._processed_collection_keys:
            return
        self._processed_collection_keys.add(key)

        location = event.get('location', {})
        x = location.get('x')
        y = location.get('y')
        if x is None or y is None:
            return

        count = max(1, int(event.get('count', 1)))
        self._add_material_evidence_from_event(event)

        active = [
            particle for particle in self.prediction_particles
            if particle.get('status') == 'active'
        ]
        nearest = sorted(
            active,
            key=lambda particle: (particle['x'] - x) ** 2 + (particle['y'] - y) ** 2,
        )
        removed = nearest[:count]
        for particle in removed:
            particle['status'] = 'observed_removed'
        self._event_counts['observed_removed'] += len(removed)
        self._record_event('observed_removed', len(removed), location)

    def _apply_collection_to_belief(self, x, y, count):
        self._ensure_belief_cells()
        if not self._belief_cells:
            return

        sigma = max(COLLECTION_UPDATE_SIGMA_M, 0.001)
        weighted_cells = []
        for key, cell in self._belief_cells.items():
            density = max(0.0, cell.get('density', 0.0))
            if density <= 0.0:
                continue
            dx = cell['x'] - x
            dy = cell['y'] - y
            kernel = math.exp(-((dx * dx + dy * dy) / (2.0 * sigma * sigma)))
            weight = density * kernel
            if weight > 0.0:
                weighted_cells.append((key, weight))

        if not weighted_cells:
            nearest = _nearest_cell(list(self._belief_cells.values()), x, y)
            if nearest is None:
                return
            weighted_cells = [((nearest['x'], nearest['y']), 1.0)]

        total_weight = sum(weight for _, weight in weighted_cells)
        if total_weight <= 0.0:
            return

        removal_mass = min(1.0, float(count) / max(1.0, float(self.prediction_debris_count)))
        for key, weight in weighted_cells:
            cell = self._belief_cells[key]
            decrement = removal_mass * (weight / total_weight)
            cell['density'] = max(0.0, cell.get('density', 0.0) - decrement)
        self._normalize_belief()

    def _stamp(self) -> str:
        t = self.get_clock().now().to_msg()
        return f'{t.sec}.{t.nanosec:09d}'

    def _has_environment_vector(self, cell):
        return (
            cell.get('occupancy') == 'free'
            and 'current_x' in cell
            and 'current_y' in cell
        )

    def _can_predict_at(self, x, y):
        if not _is_within_cell_bounds(self.twin_map_cells, x, y, CELL_SIZE):
            return False
        nearest = _nearest_cell(self.twin_map_cells, x, y)
        return bool(nearest and nearest.get('occupancy') == 'free')

    def _advance_predictions(self):
        if not self.prediction_drift_enabled or not self.twin_map_cells:
            return

        for particle in self.prediction_particles:
            if particle.get('status') != 'active':
                continue
            field_cell = _nearest_cell(self.twin_map_cells, particle['x'], particle['y'])
            if not field_cell:
                particle['status'] = 'washed_out'
                self._event_counts['washed_out'] += 1
                self._record_event('washed_out', 1, particle)
                continue
            if field_cell.get('occupancy') != 'free':
                particle['status'] = 'washed_out'
                self._event_counts['washed_out'] += 1
                self._record_event('washed_out', 1, particle)
                continue
            if not self._has_environment_vector(field_cell):
                continue

            next_state = advance_particle(
                particle,
                self.prediction_particles,
                field_cell,
                self.prediction_drift_scale,
                self.material_response,
                self.boid_rules,
            )
            if not self._can_predict_at(next_state['x'], next_state['y']):
                particle['status'] = 'washed_out'
                self._event_counts['washed_out'] += 1
                self._record_event('washed_out', 1, next_state)
                continue

            particle.update(next_state)

    def _respawn_to_target(self):
        active_particles = [
            particle for particle in self.prediction_particles
            if particle.get('status') == 'active'
        ]
        missing = max(0, int(self.prediction_debris_count) - len(active_particles))
        if missing == 0:
            self.prediction_particles = active_particles
            return

        new_particles = self._new_prediction_particles(missing)
        self.prediction_particles = active_particles + new_particles
        self._event_counts['respawned'] += missing
        self._record_event('respawned', missing)

    def _density_cells(self):
        self._ensure_belief_cells()
        return self._belief_cells

    def _counts(self):
        counts = {
            'active': 0,
            'observed_removed': 0,
            'washed_out': 0,
            'total': len(self.prediction_particles),
        }
        for particle in self.prediction_particles:
            status = particle.get('status', 'active')
            counts[status] = counts.get(status, 0) + 1
        counts['observed_removed'] = self._event_counts['observed_removed']
        counts['washed_out'] = self._event_counts['washed_out']
        counts['respawned'] = self._event_counts['respawned']
        return counts

    def _debug_particles(self):
        return [
            {
                'id': particle['id'],
                'material': normalize_material(particle.get('material')),
                'x': round(particle['x'], 3),
                'y': round(particle['y'], 3),
                'vx': round(particle.get('vx', 0.0), 3),
                'vy': round(particle.get('vy', 0.0), 3),
                'status': particle.get('status', 'active'),
            }
            for particle in self.prediction_particles
        ]

    def _publish_prediction_dashboard(self, stamp, counts):
        out = String()
        out.data = json.dumps({
            'schema': 'dtas.prediction_dashboard.v1',
            'stamp': stamp,
            'source': 'debris_prediction_node',
            'debug_only': True,
            'forbidden_as_planner_input': True,
            'prediction_particle_count': len(self.prediction_particles),
            'prediction_counts': counts,
            'lifetime_counts': dict(self._event_counts),
            'events': list(self._recent_events),
            'belief_model': {
                'type': 'active_prediction_particle_density',
                'normalized_for_planner_reward': True,
                'counts_are_debug_status': True,
            },
            'material_belief': self._material_belief_summary(),
            'material_response': self.material_response,
            'boid_rules': self.boid_rules,
            'prediction_particles': self._debug_particles(),
        })
        self.prediction_dashboard_pub.publish(out)

    def _publish(self):
        self.material_response, self.boid_rules = debris_motion_from_parameters(self)
        self._advance_predictions()
        self._respawn_to_target()
        self._initialize_belief_from_particles()
        cells = self._density_cells()
        density_cells = sorted(cells.values(), key=lambda c: (c['x'], c['y']))
        counts = self._counts()
        stamp = self._stamp()

        out = String()
        out.data = json.dumps({
            'schema': 'dtas.debris_density_map.v1',
            'stamp': stamp,
            'source': 'debris_prediction_node',
            'frame_id': 'map',
            'cell_size_m': CELL_SIZE,
            'simulation_horizon_sec': self.simulation_horizon_sec,
            'status': 'ready' if self.twin_map_cells else 'degraded_fallback_grid',
            'prediction_particle_count': len(self.prediction_particles),
            'prediction_counts': counts,
            'prediction_drift_enabled': self.prediction_drift_enabled,
            'prediction_drift_scale': self.prediction_drift_scale,
            'density_cells': density_cells,
            # Compatibility alias for the current field planner and preview.
            'cells': density_cells,
            'clusters_remaining': counts['active'],
        })
        self.density_pub.publish(out)
        self._publish_prediction_dashboard(stamp, counts)

        self.get_logger().info(
            f'Debris density: active={counts["active"]} '
            f'washed_out={counts["washed_out"]} cells={len(density_cells)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DebrisPredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
