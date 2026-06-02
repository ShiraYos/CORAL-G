#!/usr/bin/env python3

import json
import math

try:
    from opensimplex import OpenSimplex
except ImportError:
    OpenSimplex = None


FALLBACK_MAP_MIN = -2.0
FALLBACK_MAP_MAX = 2.0
DEFAULT_CELL_SIZE = 0.2
_NOISE_SEED = 61403
_NOISE_GENERATORS = {}

DEFAULT_CONTROLS = {
    'field_time_sec': 0.0,
    'current_strength': 0.22,
    'current_heading_deg': 45.0,
    'current_spatial_variation': 0.22,
    'current_shear': 0.11,
    'current_temporal_variation': 0.08,
    'current_noise_scale_m': 1.35,
    'current_noise_strength': 0.28,
    'current_noise_time_scale': 0.025,
    'current_noise_delta_m': 0.18,
    'wind_x': 0.18,
    'wind_y': 0.09,
    'wind_spatial_variation': 0.015,
    'wave_height': 0.32,
    'wave_spatial_variation': 0.08,
    'tide_strength': 0.07,
    'tide_heading_deg': 90.0,
    'tide_phase': 0.5,
    'vortex_strength': 0.82,
    'vortex_x': 0.0,
    'vortex_y': 0.0,
    'vortex_radius': 0.82,
    'vortex2_strength': -0.66,
    'vortex2_x': 1.0,
    'vortex2_y': -1.0,
    'vortex2_radius': 0.68,
    'eddy_orbit_radius': 0.28,
    'eddy_pulse_strength': 0.42,
    'eddy_intensity': 1.35,
    'shore_influence_radius': 0.75,
    'shore_current_tangent': 0.16,
    'shore_current_damping': 0.22,
    'shore_wind_damping': 0.08,
    'wave_shore_damping': 0.55,
    'wave_fetch_strength': 0.08,
    'confidence': 0.9,
}


def parse_generation_controls(raw_controls):
    controls = {}
    warnings = []
    if not raw_controls:
        return controls, warnings

    try:
        parsed = json.loads(raw_controls)
    except json.JSONDecodeError:
        warnings.append('generation_controls_json was not valid JSON; defaults used')
        return controls, warnings

    if not isinstance(parsed, dict):
        warnings.append('generation_controls_json must be an object; defaults used')
        return controls, warnings

    for key, default_value in DEFAULT_CONTROLS.items():
        if key not in parsed:
            continue
        try:
            controls[key] = float(parsed[key])
        except (TypeError, ValueError):
            warnings.append(f'generation control {key} was not numeric; default used')
    return controls, warnings


def _normalized(value, min_value, max_value):
    span = max(max_value - min_value, 0.001)
    return ((value - min_value) / span) * 2.0 - 1.0


def _noise_generator(seed):
    if OpenSimplex is None:
        return None
    if seed not in _NOISE_GENERATORS:
        _NOISE_GENERATORS[seed] = OpenSimplex(seed=seed)
    return _NOISE_GENERATORS[seed]


def _fallback_noise(x, y, z, seed):
    return math.sin(
        x * 1.71 +
        y * 2.23 +
        z * 1.37 +
        seed * 0.00031 +
        0.37 * math.sin(x * 0.53 - y * 0.41 + z)
    )


def _open_simplex_noise(x, y, z=0.0, seed=_NOISE_SEED):
    generator = _noise_generator(seed)
    if generator is None:
        return _fallback_noise(x, y, z, seed)
    if hasattr(generator, 'noise3'):
        return generator.noise3(x, y, z)
    if hasattr(generator, 'noise3d'):
        return generator.noise3d(x, y, z)
    if hasattr(generator, 'noise2'):
        return generator.noise2(x + z * 0.37, y - z * 0.29)
    return _fallback_noise(x, y, z, seed)


def _curl_noise_vector(x, y, controls):
    scale = max(controls['current_noise_scale_m'], 0.05)
    delta = max(controls['current_noise_delta_m'], 0.02)
    phase = controls['field_time_sec'] * controls['current_noise_time_scale']

    def sample(px, py):
        return _open_simplex_noise(px / scale, py / scale, phase)

    dpsi_dy = (sample(x, y + delta) - sample(x, y - delta)) / (2.0 * delta)
    dpsi_dx = (sample(x + delta, y) - sample(x - delta, y)) / (2.0 * delta)
    current_x = dpsi_dy * controls['current_noise_strength']
    current_y = -dpsi_dx * controls['current_noise_strength']
    return _limit_vector(current_x, current_y, controls['current_noise_strength'] * 1.6)


def _limit_vector(x, y, max_magnitude):
    magnitude = math.hypot(x, y)
    if magnitude <= max_magnitude or magnitude <= 0.000001:
        return x, y
    scale = max_magnitude / magnitude
    return x * scale, y * scale


def _vortex_vector(x, y, controls, prefix='vortex'):
    strength = controls[f'{prefix}_strength']
    radius = max(controls[f'{prefix}_radius'], 0.001)
    center_x = controls[f'{prefix}_x']
    center_y = controls[f'{prefix}_y']
    dx = x - center_x
    dy = y - center_y
    dist_sq = dx * dx + dy * dy
    if dist_sq <= 0.000001:
        return 0.0, 0.0

    dist = math.sqrt(dist_sq)
    influence = strength * math.exp(-dist_sq / (2.0 * radius * radius))
    return -dy / dist * influence, dx / dist * influence


def _dynamic_vortex_controls(controls, prefix, phase, offset):
    adjusted = dict(controls)
    orbit = controls['eddy_orbit_radius']
    pulse = controls['eddy_pulse_strength']
    base_strength = controls[f'{prefix}_strength']
    adjusted[f'{prefix}_x'] = controls[f'{prefix}_x'] + orbit * math.sin(phase + offset)
    adjusted[f'{prefix}_y'] = controls[f'{prefix}_y'] + orbit * math.cos(phase * 0.7 + offset)
    adjusted[f'{prefix}_strength'] = base_strength * (1.0 + pulse * math.sin(phase * 1.3 + offset))
    return adjusted


def _land_context(map_cell, map_cells, cell_size_m):
    land_cells = [
        cell for cell in map_cells
        if cell.get('occupancy') != 'free'
    ]
    if not land_cells:
        return None

    nearest = min(
        land_cells,
        key=lambda cell: (
            (map_cell['x'] - cell['x']) ** 2 +
            (map_cell['y'] - cell['y']) ** 2
        ),
    )
    dx = map_cell['x'] - nearest['x']
    dy = map_cell['y'] - nearest['y']
    distance = math.sqrt(dx * dx + dy * dy)
    if distance <= 0.000001:
        return None

    return {
        'distance': distance,
        'normal_x': dx / distance,
        'normal_y': dy / distance,
        'cell_size_m': cell_size_m,
    }


def _apply_land_effects(vector, controls, land):
    if not land:
        return vector

    radius = max(controls['shore_influence_radius'], land.get('cell_size_m', 0.0), 0.001)
    influence = max(0.0, 1.0 - min(land['distance'] / radius, 1.0))
    if influence <= 0.0:
        return vector

    normal_x = land['normal_x']
    normal_y = land['normal_y']
    tangent_x = -normal_y
    tangent_y = normal_x

    current_x = vector['current_x']
    current_y = vector['current_y']
    normal_component = current_x * normal_x + current_y * normal_y
    if normal_component < 0.0:
        current_x -= normal_x * normal_component
        current_y -= normal_y * normal_component

    tangent_component = current_x * tangent_x + current_y * tangent_y
    tangent_sign = 1.0 if tangent_component >= 0.0 else -1.0
    damping = 1.0 - controls['shore_current_damping'] * influence
    current_x = current_x * damping + tangent_x * tangent_sign * controls['shore_current_tangent'] * influence
    current_y = current_y * damping + tangent_y * tangent_sign * controls['shore_current_tangent'] * influence

    adjusted = dict(vector)
    adjusted['current_x'] = current_x
    adjusted['current_y'] = current_y
    return adjusted


def environment_vector(controls, x=0.0, y=0.0, bounds=None, land=None):
    controls = {**DEFAULT_CONTROLS, **(controls or {})}
    bounds = bounds or fallback_bounds()
    phase = float(controls.get('field_time_sec', 0.0)) * 0.045
    heading = math.radians(controls['current_heading_deg'])
    strength = controls['current_strength']
    current_x = strength * math.cos(heading)
    current_y = strength * math.sin(heading)

    norm_x = _normalized(x, bounds['min_x'], bounds['max_x'])
    norm_y = _normalized(y, bounds['min_y'], bounds['max_y'])
    noise_x, noise_y = _curl_noise_vector(x, y, controls)
    current_x += noise_x
    current_y += noise_y

    vortex_controls = _dynamic_vortex_controls(controls, 'vortex', phase, 0.0)
    vortex2_controls = _dynamic_vortex_controls(controls, 'vortex2', phase, 2.1)
    vortex_x, vortex_y = _vortex_vector(x, y, vortex_controls, 'vortex')
    vortex2_x, vortex2_y = _vortex_vector(x, y, vortex2_controls, 'vortex2')
    current_x += (vortex_x + vortex2_x) * controls['eddy_intensity']
    current_y += (vortex_y + vortex2_y) * controls['eddy_intensity']

    wind_variation = controls['wind_spatial_variation']
    wind_x = controls['wind_x'] + wind_variation * math.sin(norm_y * math.pi * 0.5)
    wind_y = controls['wind_y'] + wind_variation * math.cos(norm_x * math.pi * 0.5)

    wave = controls['wave_height'] + controls['wave_spatial_variation'] * (
        math.sin((norm_x + 1.0) * math.pi) +
        math.cos((norm_y + 1.0) * math.pi)
    ) * 0.5
    tide = controls['tide_strength'] * (
        0.5 + 0.5 * math.sin(controls['tide_phase'] + phase * 0.35)
    )
    wave += tide
    vector = _apply_land_effects({
        'current_x': current_x,
        'current_y': current_y,
        'wind_x': wind_x,
        'wind_y': wind_y,
        'wave_height': max(0.0, wave),
    }, controls, land)

    return {
        'current_x': round(vector['current_x'], 3),
        'current_y': round(vector['current_y'], 3),
        'wind_x': round(vector['wind_x'], 3),
        'wind_y': round(vector['wind_y'], 3),
        'wave_height': round(vector['wave_height'], 3),
        'confidence': round(controls['confidence'], 3),
    }


def map_bounds(map_msg):
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    width_m = map_msg.info.width * map_msg.info.resolution
    height_m = map_msg.info.height * map_msg.info.resolution
    return {
        'min_x': origin_x,
        'min_y': origin_y,
        'max_x': origin_x + width_m,
        'max_y': origin_y + height_m,
    }


def fallback_bounds():
    return {
        'min_x': FALLBACK_MAP_MIN,
        'min_y': FALLBACK_MAP_MIN,
        'max_x': FALLBACK_MAP_MAX,
        'max_y': FALLBACK_MAP_MAX,
    }


def simulation_cell_centers(bounds, cell_size_m=DEFAULT_CELL_SIZE):
    x = bounds['min_x'] + cell_size_m / 2.0
    while x < bounds['max_x']:
        y = bounds['min_y'] + cell_size_m / 2.0
        while y < bounds['max_y']:
            yield round(x, 3), round(y, 3)
            y += cell_size_m
        x += cell_size_m


def occupancy_at(msg, x, y):
    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)

    if not (0 <= grid_x < msg.info.width and 0 <= grid_y < msg.info.height):
        return 'unknown'

    value = msg.data[grid_y * msg.info.width + grid_x]
    if value == 0:
        return 'free'
    if value == -1:
        return 'unknown'
    return 'blocked'


def build_map_cells(map_msg, cell_size_m=DEFAULT_CELL_SIZE):
    bounds = map_bounds(map_msg)
    return [
        {
            'x': x,
            'y': y,
            'occupancy': occupancy_at(map_msg, x, y),
        }
        for x, y in simulation_cell_centers(bounds, cell_size_m)
    ]


def build_environment_cells(map_msg=None, cell_size_m=DEFAULT_CELL_SIZE, controls=None):
    controls = {**DEFAULT_CONTROLS, **(controls or {})}
    bounds = map_bounds(map_msg) if map_msg is not None else fallback_bounds()
    map_cells = (
        build_map_cells(map_msg, cell_size_m)
        if map_msg is not None
        else [
            {'x': x, 'y': y, 'occupancy': 'free'}
            for x, y in simulation_cell_centers(fallback_bounds(), cell_size_m)
        ]
    )
    cells = []

    for map_cell in map_cells:
        occupancy = map_cell['occupancy']
        has_environment = occupancy == 'free'
        land = (
            _land_context(map_cell, map_cells, cell_size_m)
            if occupancy == 'free'
            else None
        )
        vector = environment_vector(controls, map_cell['x'], map_cell['y'], bounds, land)
        cell = {
            'x': map_cell['x'],
            'y': map_cell['y'],
            'occupancy': occupancy,
            'has_environment': has_environment,
            'current_x': vector['current_x'] if has_environment else 0.0,
            'current_y': vector['current_y'] if has_environment else 0.0,
            'wind_x': vector['wind_x'] if has_environment else 0.0,
            'wind_y': vector['wind_y'] if has_environment else 0.0,
            'wave_height': vector['wave_height'] if has_environment else 0.0,
            'confidence': vector['confidence'] if has_environment else 0.0,
        }
        cells.append(cell)

    return cells


def observation_cells(field_cells):
    return [
        {
            'x': cell['x'],
            'y': cell['y'],
            'current_x': cell['current_x'],
            'current_y': cell['current_y'],
            'wind_x': cell['wind_x'],
            'wind_y': cell['wind_y'],
            'wave_height': cell['wave_height'],
        }
        for cell in field_cells
        if cell.get('has_environment', True)
    ]
