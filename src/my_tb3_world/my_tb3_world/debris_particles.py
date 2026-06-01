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


def normalize_material(material):
    material = str(material or UNKNOWN_MATERIAL)
    return MATERIAL_ALIASES.get(material, material if material in MATERIAL_RESPONSE else UNKNOWN_MATERIAL)


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


def boid_adjustment(particle, particles, neighbor_radius=0.45):
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

    response = MATERIAL_RESPONSE.get(normalize_material(particle.get('material')), MATERIAL_RESPONSE[UNKNOWN_MATERIAL])
    return (
        (cohesion_x / neighbors) * response['cohesion'] + separation_x * response['separation'] * 0.01,
        (cohesion_y / neighbors) * response['cohesion'] + separation_y * response['separation'] * 0.01,
    )


def advance_particle(particle, particles, field_cell, drift_scale):
    response = MATERIAL_RESPONSE.get(normalize_material(particle.get('material')), MATERIAL_RESPONSE[UNKNOWN_MATERIAL])
    boid_x, boid_y = boid_adjustment(particle, particles)
    wave = field_cell.get('wave_height', 0.0) * response['wave']
    wave_sign = _stable_wave_sign(particle['id'])

    ax = (
        field_cell.get('current_x', 0.0) * response['current'] +
        field_cell.get('wind_x', 0.0) * response['wind'] +
        wave * wave_sign * 0.1 +
        boid_x
    )
    ay = (
        field_cell.get('current_y', 0.0) * response['current'] +
        field_cell.get('wind_y', 0.0) * response['wind'] -
        wave * wave_sign * 0.1 +
        boid_y
    )

    vx = particle.get('vx', 0.0) * 0.35 + ax * drift_scale
    vy = particle.get('vy', 0.0) * 0.35 + ay * drift_scale
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
