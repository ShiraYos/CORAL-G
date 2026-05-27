import random
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

from .contracts import (
    MATERIALS,
    cell_key,
    distance,
    material_counts_zero,
    material_probabilities,
    pose,
    top_density_cells,
    with_envelope,
)


DEFAULT_FIELD_POLICY = {
    "density_reward": 1.0,
    "travel_cost": 0.2,
    "storage_penalty": 0.5,
    "fuel_penalty": 0.5,
    "return_reserve": 0.2,
    "min_goal_distance": 1.0,
    "collected_cell_exclusion_radius": 0.45,
}

DEFAULT_RETURN_POLICY = {
    "base": {"x": 0.0, "y": 0.0, "heading": 0.0},
    "storage": 0.5,
    "fuel": 0.25,
}

DEFAULT_DEBRIS_TARGETS = (
    {"x": 0.75, "y": -0.75, "density": 1.2, "material_hint": "plastic"},
    {"x": -0.75, "y": -0.75, "density": 1.15, "material_hint": "foam"},
    {"x": 0.75, "y": -1.75, "density": 1.45, "material_hint": "rubber"},
)


def default_environment(
    width_m: int = 10,
    height_m: int = 10,
    cell_size_m: float = 1.0,
    x_min: float = 0.0,
    y_min: float = 0.0,
) -> Dict[str, Any]:
    cells = []
    for x in range(width_m):
        for y in range(height_m):
            cell_x = round(float(x_min) + x * float(cell_size_m), 3)
            cell_y = round(float(y_min) + y * float(cell_size_m), 3)
            current_x = round(0.08 + 0.01 * y, 3)
            current_y = round(0.02 * ((x % 3) - 1), 3)
            wind_x = round(0.03 * ((y % 4) - 1), 3)
            wind_y = round(0.04 + 0.005 * x, 3)
            wave = round(0.2 + 0.02 * ((x + y) % 5), 3)
            cells.append(
                {
                    "x": cell_x,
                    "y": cell_y,
                    "current": {"x": current_x, "y": current_y},
                    "wind": {"x": wind_x, "y": wind_y},
                    "wave": wave,
                }
            )
    return with_envelope(
        "environment_field",
        "twin_input_node",
        {
            "mode": "preset_ocean_conditions",
            "frame_id": "map",
            "cleanup_zone": {
                "x_min": float(x_min),
                "y_min": float(y_min),
                "x_max": float(x_min) + float(width_m) * float(cell_size_m),
                "y_max": float(y_min) + float(height_m) * float(cell_size_m),
            },
            "cell_size_m": float(cell_size_m),
            "cells": cells,
            "status": "ready",
        },
    )


def fallback_map_update(scan_seen: bool = False, cell_size_m: float = 1.0) -> Dict[str, Any]:
    free_space = [{"x": x, "y": y} for x in range(3) for y in range(3)]
    obstacles = [] if scan_seen else [{"x": 4, "y": 4, "reason": "mock_obstacle"}]
    return with_envelope(
        "map_sync_update",
        "twin_input_node",
        {
            "frame_id": "map",
            "robot_pose": pose(0.0, 0.0, 0.0),
            "cell_size_m": float(cell_size_m),
            "observed_obstacles": obstacles,
            "free_space_cells": free_space,
            "map_confidence": 0.65 if scan_seen else 0.35,
            "status": "degraded" if scan_seen else "waiting_for_scan",
        },
    )


def map_update_from_scan(ranges: Iterable[float], angle_min: float, angle_increment: float, range_max: float) -> Dict[str, Any]:
    free_space = []
    obstacles = []
    for index, reading in enumerate(ranges):
        if index % 12 != 0:
            continue
        if reading <= 0:
            continue
        angle = angle_min + index * angle_increment
        distance_m = min(float(reading), float(range_max), 5.0)
        x = round(distance_m)
        y = round(angle * 2.0)
        if reading < range_max * 0.95:
            obstacles.append({"x": x, "y": y, "range_m": round(float(reading), 3)})
        else:
            free_space.append({"x": x, "y": y})
    if not free_space:
        free_space = [{"x": x, "y": 0} for x in range(1, 4)]
    return with_envelope(
        "map_sync_update",
        "twin_input_node",
        {
            "frame_id": "map",
            "robot_pose": pose(0.0, 0.0, 0.0),
            "observed_obstacles": obstacles[:20],
            "free_space_cells": free_space[:30],
            "map_confidence": 0.8,
            "status": "ready",
        },
    )


class MaterialBelief:
    def __init__(self) -> None:
        self.counts = material_counts_zero()
        self.warnings: List[str] = []

    def update_from_report(self, report: Mapping[str, Any]) -> Dict[str, Any]:
        for item in report.get("materials", []):
            label = str(item.get("material", item.get("label", "unknown"))).lower()
            count = int(item.get("count", 1))
            if label not in MATERIALS:
                self.warnings.append(f"mapped unsupported material '{label}' to unknown")
                label = "unknown"
            self.counts[label] += max(0, count)
        return self.message()

    def message(self) -> Dict[str, Any]:
        total = sum(self.counts.values())
        return with_envelope(
            "material_distribution",
            "material_belief_node",
            {
                "counts": dict(self.counts),
                "probabilities": material_probabilities(self.counts),
                "status": "ready" if total else "no_observations",
                "warnings": list(self.warnings),
            },
        )


class DigitalTwinFusion:
    def __init__(self) -> None:
        self.environment: Optional[Dict[str, Any]] = None
        self.map_update: Optional[Dict[str, Any]] = None
        self.material_distribution: Optional[Dict[str, Any]] = None
        self.collection_updates: List[Dict[str, Any]] = []

    def set_environment(self, message: Mapping[str, Any]) -> Dict[str, Any]:
        self.environment = dict(message)
        return self.message()

    def set_map_update(self, message: Mapping[str, Any]) -> Dict[str, Any]:
        self.map_update = dict(message)
        return self.message()

    def set_material_distribution(self, message: Mapping[str, Any]) -> Dict[str, Any]:
        self.material_distribution = dict(message)
        return self.message()

    def add_collection_report(self, message: Mapping[str, Any]) -> Dict[str, Any]:
        self.collection_updates.append(dict(message))
        self.collection_updates = self.collection_updates[-20:]
        return self.message()

    def _map_cells(self) -> Dict[str, Any]:
        water_cells = []
        blocked_cells = []
        unknown_cells = []
        if self.environment:
            water_cells = [{"x": c["x"], "y": c["y"]} for c in self.environment.get("cells", [])]
        if self.map_update:
            blocked_cells = [dict(c) for c in self.map_update.get("observed_obstacles", [])]
            free_keys = {cell_key(c) for c in self.map_update.get("free_space_cells", [])}
            blocked_keys = {cell_key(c) for c in blocked_cells}
            known = free_keys | blocked_keys
            unknown_cells = [c for c in water_cells if cell_key(c) not in known]
        else:
            unknown_cells = list(water_cells)
        return {"water_cells": water_cells, "blocked_cells": blocked_cells, "unknown_cells": unknown_cells}

    def message(self) -> Dict[str, Any]:
        missing = []
        if not self.environment:
            missing.append("environment_field")
        if not self.map_update:
            missing.append("map_sync_update")
        if not self.material_distribution:
            missing.append("material_distribution")
        status = "ready" if not missing else "waiting_for_inputs"
        if self.map_update and self.map_update.get("status") in ("degraded", "waiting_for_scan"):
            status = "degraded"
        return with_envelope(
            "twin_state",
            "digital_twin_state",
            {
                "environment_status": self.environment.get("status", "missing") if self.environment else "missing",
                "map": self._map_cells(),
                "material_belief": self.material_distribution or MaterialBelief().message(),
                "collection_updates": list(self.collection_updates),
                "sync_status": {"status": status, "missing_inputs": missing},
            },
        )


def simulate_density(twin_state: Mapping[str, Any], debris_count: int = 100, horizon_sec: float = 60.0, seed: int = 23) -> Dict[str, Any]:
    twin_map = twin_state.get("map", {})
    water_cells = [dict(c) for c in twin_map.get("water_cells", [])]
    blocked = {cell_key(c) for c in twin_map.get("blocked_cells", [])}
    valid_cells = [c for c in water_cells if cell_key(c) not in blocked]
    rng = random.Random(seed)
    density_cells = []
    if valid_cells:
        collected_locations = [
            update.get("location", {})
            for update in twin_state.get("collection_updates", [])
            if update.get("status") == "ready"
        ]
        used_keys = set()
        for target in DEFAULT_DEBRIS_TARGETS[: max(0, min(len(DEFAULT_DEBRIS_TARGETS), int(debris_count)))]:
            if any(distance(target, location) <= 0.45 for location in collected_locations):
                continue
            nearest = min(valid_cells, key=lambda cell: distance(cell, target))
            nearest_key = cell_key(nearest)
            if nearest_key in used_keys:
                continue
            used_keys.add(nearest_key)
            jitter = rng.random() * 0.05
            density_cells.append(
                {
                    "x": nearest["x"],
                    "y": nearest["y"],
                    "density": round(float(target["density"]) + jitter, 4),
                    "material_hint": target["material_hint"],
                }
            )
    return with_envelope(
        "debris_density_map",
        "debris_simulation_node",
        {
            "frame_id": "map",
            "cell_size_m": 1.0,
            "simulation_horizon_sec": float(horizon_sec),
            "status": "ready" if density_cells else "waiting_for_twin_state",
            "density_cells": density_cells,
        },
    )


def plan_next_cell(
    twin_state: Optional[Mapping[str, Any]],
    density_map: Optional[Mapping[str, Any]],
    robot_state: Optional[Mapping[str, Any]],
    field_policy: Mapping[str, float] = DEFAULT_FIELD_POLICY,
    return_policy: Mapping[str, Any] = DEFAULT_RETURN_POLICY,
) -> Dict[str, Any]:
    base = return_policy.get("base", DEFAULT_RETURN_POLICY["base"])
    if not twin_state or not density_map or not robot_state:
        return _planner_message("waiting", "idle", base, 0.0, 0.0, 0.0, 0.0, 0.0, "unknown", "waiting_for_inputs")

    robot_pose = robot_state.get("pose", pose())
    fuel = float(robot_state.get("fuel_level", 1.0))
    storage = float(robot_state.get("storage_fill", 0.0))
    if not density_map.get("density_cells"):
        return_cost = distance(robot_pose, base)
        if return_cost > 0.5:
            return _planner_message(
                "ready",
                "return_to_base",
                base,
                0.0,
                0.0,
                0.0,
                return_cost,
                fuel,
                "normal",
                "all_debris_collected_return_to_base",
            )
        return _planner_message("ready", "idle", base, 0.0, 0.0, 0.0, 0.0, fuel, "normal", "mission_complete")
    if fuel <= float(return_policy.get("fuel", 0.25)):
        return _planner_message("ready", "return_to_base", base, 0.0, 0.0, 0.0, distance(robot_pose, base), fuel, "critical", "fuel_below_return_threshold")
    if storage >= float(return_policy.get("storage", 0.8)):
        return _planner_message("ready", "return_to_base", base, 0.0, 0.0, 0.0, distance(robot_pose, base), fuel, "full", "storage_above_return_threshold")

    blocked = {cell_key(c) for c in twin_state.get("map", {}).get("blocked_cells", [])}
    collected_locations = [
        update.get("location", {})
        for update in twin_state.get("collection_updates", [])
        if update.get("status") == "ready"
    ]
    collected_radius = float(field_policy.get("collected_cell_exclusion_radius", 0.45))
    min_goal_distance = float(field_policy.get("min_goal_distance", 0.0))

    def already_collected(cell: Mapping[str, Any]) -> bool:
        return any(distance(cell, location) <= collected_radius for location in collected_locations)

    def best_candidate(require_min_distance: bool) -> Optional[Tuple[float, Mapping[str, Any], float, float, float]]:
        best = None
        for cell in density_map.get("density_cells", []):
            if cell_key(cell) in blocked or already_collected(cell):
                continue
            if require_min_distance and distance(robot_pose, cell) < min_goal_distance:
                continue
            density_reward = float(cell.get("density", 0.0)) * float(field_policy.get("density_reward", 1.0))
            travel_cost = distance(robot_pose, cell) * float(field_policy.get("travel_cost", 0.2))
            return_cost = distance(cell, base) * float(field_policy.get("return_reserve", 0.2))
            storage_penalty = storage * float(field_policy.get("storage_penalty", 0.5))
            fuel_penalty = max(0.0, 1.0 - fuel) * float(field_policy.get("fuel_penalty", 0.5))
            utility = density_reward - travel_cost - return_cost - storage_penalty - fuel_penalty
            candidate = (utility, cell, density_reward, travel_cost, return_cost)
            if best is None or candidate[0] > best[0]:
                best = candidate
        return best

    best = best_candidate(require_min_distance=True) or best_candidate(require_min_distance=False)
    if not best:
        return _planner_message("degraded", "idle", base, 0.0, 0.0, 0.0, 0.0, fuel, "normal", "no_reachable_density_cells")
    utility, cell, density_reward, travel_cost, return_cost = best
    return _planner_message("ready", "cleanup_cell", cell, utility, density_reward, travel_cost, return_cost, fuel, "normal", "highest_utility_cell")


def _planner_message(
    status: str,
    mode: str,
    cell: Mapping[str, Any],
    utility: float,
    density_reward: float,
    travel_cost: float,
    return_cost: float,
    fuel_margin: float,
    storage_state: str,
    reason: str,
) -> Dict[str, Any]:
    return with_envelope(
        "next_cell_goal",
        "field_planner_node",
        {
            "status": status,
            "mode": mode,
            "cell": {"x": float(cell.get("x", 0.0)), "y": float(cell.get("y", 0.0)), "heading": float(cell.get("heading", 0.0))},
            "utility": round(float(utility), 4),
            "density_reward": round(float(density_reward), 4),
            "travel_cost": round(float(travel_cost), 4),
            "return_cost": round(float(return_cost), 4),
            "fuel_margin": round(float(fuel_margin), 4),
            "storage_state": storage_state,
            "reason": reason,
        },
    )


def make_robot_state(
    x: float = 0.0,
    y: float = 0.0,
    heading: float = 0.0,
    storage_fill: float = 0.0,
    fuel_level: float = 1.0,
    navigation_status: str = "waiting_for_nav2",
    base: Mapping[str, Any] = DEFAULT_RETURN_POLICY["base"],
) -> Dict[str, Any]:
    robot_pose = pose(x, y, heading)
    return with_envelope(
        "robot_state",
        "robot_state_node",
        {
            "pose": robot_pose,
            "storage_fill": max(0.0, min(1.0, float(storage_fill))),
            "fuel_level": max(0.0, min(1.0, float(fuel_level))),
            "at_base": distance(robot_pose, base) <= 0.5,
            "navigation_status": navigation_status,
        },
    )


def make_collection_report(location: Mapping[str, Any], material: str = "unknown", count: int = 1, radius_m: float = 0.75) -> Dict[str, Any]:
    label = material if material in MATERIALS else "unknown"
    return with_envelope(
        "collection_report",
        "collection_report_node",
        {
            "location": {"x": float(location.get("x", 0.0)), "y": float(location.get("y", 0.0)), "heading": float(location.get("heading", 0.0))},
            "materials": [{"material": label, "count": max(0, int(count))}],
            "count": max(0, int(count)),
            "collection_radius_m": float(radius_m),
            "status": "ready",
        },
    )


def make_demo_report(
    run_id: str,
    next_cell_goal: Optional[Mapping[str, Any]],
    robot_state: Optional[Mapping[str, Any]],
    material_distribution: Optional[Mapping[str, Any]],
    density_map: Optional[Mapping[str, Any]],
    field_policy: Mapping[str, Any] = DEFAULT_FIELD_POLICY,
) -> Dict[str, Any]:
    counts = material_distribution.get("counts", material_counts_zero()) if material_distribution else material_counts_zero()
    return with_envelope(
        "report",
        "collection_report_node",
        {
            "run_id": run_id,
            "next_cell_goal": dict(next_cell_goal or {}),
            "collection_stats": {"counts": counts, "total": sum(int(v) for v in counts.values())},
            "distribution_shift": material_distribution.get("probabilities", {}) if material_distribution else {},
            "robot_summary": dict(robot_state or {}),
            "field_policy": dict(field_policy),
            "heatmap_cells": top_density_cells((density_map or {}).get("density_cells", []), limit=20),
        },
    )
