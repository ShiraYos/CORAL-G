#!/usr/bin/env python3

import math
import random


MATERIALS = ('plastic', 'wood', 'metal')
UNKNOWN_MATERIAL = 'unknown'

MATERIAL_ALIASES = {
    'A': 'plastic',
    'B': 'wood',
    'C': 'metal',
}

MATERIAL_RESPONSE = {
    'plastic': {'current': 1.0, 'wind': 0.65, 'wave': 0.35, 'cohesion': 0.25, 'separation': 0.18},
    'wood': {'current': 0.8, 'wind': 0.25, 'wave': 0.25, 'cohesion': 0.4, 'separation': 0.2},
    'metal': {'current': 0.35, 'wind': 0.02, 'wave': 0.08, 'cohesion': 0.12, 'separation': 0.25},
    'unknown': {'current': 0.65, 'wind': 0.25, 'wave': 0.2, 'cohesion': 0.25, 'separation': 0.2},
}
MATERIAL_RESPONSE_FIELDS = ('current', 'wind', 'wave', 'cohesion', 'separation')
BOID_RULES = {
    'neighbor_radius_m': 0.3,
    'separation_scale': 0.01,
    'water_damping': 0.65,
    'wave_jitter_scale': 0.1,
}
LEGACY_VELOCITY_MEMORY = 1.0 - BOID_RULES['water_damping']


def normalize_material(material):
    material = str(material or UNKNOWN_MATERIAL)
    return MATERIAL_ALIASES.get(material, material if material in MATERIAL_RESPONSE else UNKNOWN_MATERIAL)


def _copy_material_response():
    return {
        material: dict(response)
        for material, response in MATERIAL_RESPONSE.items()
    }


def _copy_boid_rules():
    return dict(BOID_RULES)


def declare_debris_motion_parameters(node):
    for material, response in MATERIAL_RESPONSE.items():
        for field, value in response.items():
            node.declare_parameter(f'debris_material.{material}.{field}', value)
    for field, value in BOID_RULES.items():
        node.declare_parameter(f'debris_boid.{field}', value)
    node.declare_parameter('debris_boid.velocity_memory', LEGACY_VELOCITY_MEMORY)


def debris_motion_from_parameters(node):
    material_response = _copy_material_response()
    for material in MATERIAL_RESPONSE:
        for field in MATERIAL_RESPONSE_FIELDS:
            value = node.get_parameter(f'debris_material.{material}.{field}').value
            material_response[material][field] = max(0.0, float(value))

    boid_rules = _copy_boid_rules()
    for field in BOID_RULES:
        value = node.get_parameter(f'debris_boid.{field}').value
        if field == 'water_damping':
            boid_rules[field] = min(1.0, max(0.0, float(value)))
        else:
            boid_rules[field] = max(0.0, float(value))
    legacy_velocity_memory = max(0.0, float(node.get_parameter('debris_boid.velocity_memory').value))
    if (
        legacy_velocity_memory != LEGACY_VELOCITY_MEMORY
        and boid_rules['water_damping'] == BOID_RULES['water_damping']
    ):
        boid_rules['water_damping'] = min(1.0, max(0.0, 1.0 - legacy_velocity_memory))
    return material_response, boid_rules


def _fallback_seed_cells():
    cells = []
    x = -1.75
    while x <= 1.75:
        y = -1.75
        while y <= 1.75:
            cells.append({'x': round(x, 3), 'y': round(y, 3), 'occupancy': 'free'})
            y += 0.5
        x += 0.5
    return cells


def _free_seed_cells(cells):
    return [
        cell for cell in (cells or [])
        if cell.get('occupancy', 'free') == 'free'
        and cell.get('has_environment', True)
    ]


def seed_physical_particles(seed_cells=None, count=100, random_seed=23, start_index=1):
    """Create deterministic hidden physical debris particles over free cells."""
    free_cells = _free_seed_cells(seed_cells) or _fallback_seed_cells()
    rng = random.Random(f'{random_seed}:{start_index}:{count}')
    shuffled_cells = list(free_cells)
    rng.shuffle(shuffled_cells)
    particles = []
    for offset in range(int(count)):
        index = start_index + offset
        cell = shuffled_cells[offset % len(shuffled_cells)]
        jitter_x = rng.uniform(-0.08, 0.08)
        jitter_y = rng.uniform(-0.08, 0.08)
        x = round(cell['x'] + jitter_x, 3)
        y = round(cell['y'] + jitter_y, 3)
        particles.append({
            'id': f'physical_particle_{index:03d}',
            'material': MATERIALS[(index - 1) % len(MATERIALS)],
            'x': x,
            'y': y,
            'initial_x': x,
            'initial_y': y,
            'vx': 0.0,
            'vy': 0.0,
            'ax': 0.0,
            'ay': 0.0,
            'status': 'active',
        })
    return particles


def particle_counts(particles):
    counts = {'active': 0, 'collected': 0, 'washed_out': 0, 'total': len(particles)}
    materials = {}
    for particle in particles:
        status = particle.get('status', 'active')
        counts[status] = counts.get(status, 0) + 1
        material = normalize_material(particle.get('material', UNKNOWN_MATERIAL))
        materials.setdefault(material, {'active': 0, 'collected': 0, 'washed_out': 0, 'total': 0})
        materials[material]['total'] += 1
        materials[material][status] = materials[material].get(status, 0) + 1
    return counts, materials


def _stable_wave_sign(particle_id):
    return 1.0 if sum(ord(ch) for ch in particle_id) % 2 == 0 else -1.0


def boid_adjustment(particle, particles, material_response=None, boid_rules=None):
    boid_rules = boid_rules or BOID_RULES
    neighbor_radius = boid_rules.get('neighbor_radius_m', BOID_RULES['neighbor_radius_m'])
    cohesion_x = 0.0
    cohesion_y = 0.0
    separation_x = 0.0
    separation_y = 0.0
    neighbors = 0

    for other in particles:
        if other is particle or other.get('status') != 'active':
            continue
        dx = other['x'] - particle['x']
        dy = other['y'] - particle['y']
        dist = math.sqrt(dx * dx + dy * dy)
        if dist == 0.0 or dist > neighbor_radius:
            continue
        neighbors += 1
        cohesion_x += dx
        cohesion_y += dy
        separation_x -= dx / (dist * dist)
        separation_y -= dy / (dist * dist)

    if neighbors == 0:
        return 0.0, 0.0

    responses = material_response or MATERIAL_RESPONSE
    response = responses.get(normalize_material(particle.get('material')), responses[UNKNOWN_MATERIAL])
    separation_scale = boid_rules.get('separation_scale', BOID_RULES['separation_scale'])
    return (
        (cohesion_x / neighbors) * response['cohesion'] + separation_x * response['separation'] * separation_scale,
        (cohesion_y / neighbors) * response['cohesion'] + separation_y * response['separation'] * separation_scale,
    )


def advance_particle(particle, particles, field_cell, drift_scale, material_response=None, boid_rules=None):
    responses = material_response or MATERIAL_RESPONSE
    rules = boid_rules or BOID_RULES
    response = responses.get(normalize_material(particle.get('material')), responses[UNKNOWN_MATERIAL])
    boid_x, boid_y = boid_adjustment(particle, particles, responses, rules)
    wave = field_cell.get('wave_height', 0.0) * response['wave']
    wave_sign = _stable_wave_sign(particle['id'])
    wave_jitter = rules.get('wave_jitter_scale', BOID_RULES['wave_jitter_scale'])

    ax = (
        field_cell.get('current_x', 0.0) * response['current'] +
        field_cell.get('wind_x', 0.0) * response['wind'] +
        wave * wave_sign * wave_jitter +
        boid_x
    )
    ay = (
        field_cell.get('current_y', 0.0) * response['current'] +
        field_cell.get('wind_y', 0.0) * response['wind'] -
        wave * wave_sign * wave_jitter +
        boid_y
    )

    water_damping = min(1.0, max(0.0, rules.get('water_damping', BOID_RULES['water_damping'])))
    velocity_retention = 1.0 - water_damping
    vx = particle.get('vx', 0.0) * velocity_retention + ax * drift_scale
    vy = particle.get('vy', 0.0) * velocity_retention + ay * drift_scale
    return {
        'x': round(particle['x'] + vx, 3),
        'y': round(particle['y'] + vy, 3),
        'vx': round(vx, 3),
        'vy': round(vy, 3),
        'ax': round(ax, 3),
        'ay': round(ay, 3),
    }


def public_particle(particle):
    return {
        'id': particle['id'],
        'material': normalize_material(particle.get('material')),
        'x': particle['x'],
        'y': particle['y'],
        'initial_x': particle['initial_x'],
        'initial_y': particle['initial_y'],
        'vx': particle.get('vx', 0.0),
        'vy': particle.get('vy', 0.0),
        'ax': particle.get('ax', 0.0),
        'ay': particle.get('ay', 0.0),
        'status': particle.get('status', 'active'),
    }
