import json
import math
from datetime import datetime, timezone
from typing import Any, Dict, Iterable, List, Mapping, Tuple


MATERIALS = ("plastic", "foam", "rubber", "metal", "unknown")

TOPICS = {
    "environment_field": "/coral_g/environment_field",
    "material_distribution": "/coral_g/material_distribution",
    "map_sync_update": "/coral_g/map_sync_update",
    "twin_state": "/coral_g/twin_state",
    "debris_density_map": "/coral_g/debris_density_map",
    "next_cell_goal": "/coral_g/next_cell_goal",
    "goal_pose": "/coral_g/goal_pose",
    "navigation_status": "/coral_g/navigation_status",
    "scan": "/scan",
    "cmd_vel": "/cmd_vel",
    "robot_state": "/coral_g/robot_state",
    "collection_report": "/coral_g/collection_report",
    "report": "/report",
}

SCHEMAS = {
    "environment_field": "coral_g.environment_field.v1",
    "material_distribution": "coral_g.material_distribution.v1",
    "map_sync_update": "coral_g.map_sync_update.v1",
    "twin_state": "coral_g.twin_state.v1",
    "debris_density_map": "coral_g.debris_density_map.v1",
    "next_cell_goal": "coral_g.next_cell_goal.v1",
    "robot_state": "coral_g.robot_state.v1",
    "collection_report": "coral_g.collection_report.v1",
    "report": "coral_g.report.v1",
}

REQUIRED_FIELDS = {
    "environment_field": ("schema", "stamp", "source", "mode", "frame_id", "cleanup_zone", "cell_size_m", "cells"),
    "material_distribution": ("schema", "stamp", "source", "counts", "probabilities", "status"),
    "map_sync_update": (
        "schema",
        "stamp",
        "source",
        "frame_id",
        "robot_pose",
        "observed_obstacles",
        "free_space_cells",
        "map_confidence",
        "status",
    ),
    "twin_state": ("schema", "stamp", "source", "environment_status", "map", "material_belief", "collection_updates", "sync_status"),
    "debris_density_map": ("schema", "stamp", "source", "frame_id", "cell_size_m", "simulation_horizon_sec", "status", "density_cells"),
    "next_cell_goal": (
        "schema",
        "stamp",
        "source",
        "status",
        "mode",
        "cell",
        "utility",
        "density_reward",
        "travel_cost",
        "return_cost",
        "fuel_margin",
        "storage_state",
        "reason",
    ),
    "robot_state": ("schema", "stamp", "source", "pose", "storage_fill", "fuel_level", "at_base", "navigation_status"),
    "collection_report": ("schema", "stamp", "source", "location", "materials", "count", "collection_radius_m", "status"),
    "report": (
        "schema",
        "stamp",
        "source",
        "run_id",
        "next_cell_goal",
        "collection_stats",
        "distribution_shift",
        "robot_summary",
        "field_policy",
        "heatmap_cells",
    ),
}


class ContractError(ValueError):
    """Raised when a CORAL-G JSON contract is invalid."""


def utc_stamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def with_envelope(kind: str, source: str, payload: Mapping[str, Any]) -> Dict[str, Any]:
    message = {"schema": SCHEMAS[kind], "stamp": utc_stamp(), "source": source}
    message.update(payload)
    return message


def dumps(message: Mapping[str, Any]) -> str:
    return json.dumps(message, separators=(",", ":"), sort_keys=True)


def loads(data: str) -> Dict[str, Any]:
    parsed = json.loads(data)
    if not isinstance(parsed, dict):
        raise ContractError("message payload must be a JSON object")
    return parsed


def validate(kind: str, message: Mapping[str, Any]) -> None:
    if kind not in REQUIRED_FIELDS:
        raise ContractError(f"unknown contract kind: {kind}")
    missing = [field for field in REQUIRED_FIELDS[kind] if field not in message]
    if missing:
        raise ContractError(f"{kind} missing required fields: {', '.join(missing)}")
    expected_schema = SCHEMAS.get(kind)
    if expected_schema and message.get("schema") != expected_schema:
        raise ContractError(f"{kind} schema must be {expected_schema}, got {message.get('schema')}")
    if kind == "material_distribution":
        for bucket in ("counts", "probabilities"):
            labels = set(message[bucket])
            missing_materials = [m for m in MATERIALS if m not in labels]
            if missing_materials:
                raise ContractError(f"{kind}.{bucket} missing material labels: {missing_materials}")


def validate_json(kind: str, data: str) -> Dict[str, Any]:
    message = loads(data)
    validate(kind, message)
    return message


def pose(x: float = 0.0, y: float = 0.0, heading: float = 0.0) -> Dict[str, float]:
    return {"x": float(x), "y": float(y), "heading": float(heading)}


def cell_key(cell: Mapping[str, Any]) -> Tuple[int, int]:
    return (int(round(float(cell.get("x", 0)))), int(round(float(cell.get("y", 0)))))


def distance(a: Mapping[str, Any], b: Mapping[str, Any]) -> float:
    return math.hypot(float(a.get("x", 0.0)) - float(b.get("x", 0.0)), float(a.get("y", 0.0)) - float(b.get("y", 0.0)))


def quaternion_from_yaw(yaw: float) -> Dict[str, float]:
    half = yaw * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def yaw_from_quaternion(q: Mapping[str, Any]) -> float:
    z = float(q.get("z", 0.0))
    w = float(q.get("w", 1.0))
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def material_counts_zero() -> Dict[str, int]:
    return {material: 0 for material in MATERIALS}


def material_probabilities(counts: Mapping[str, int]) -> Dict[str, float]:
    total = float(sum(int(counts.get(material, 0)) for material in MATERIALS))
    if total <= 0:
        return {material: 0.0 for material in MATERIALS}
    return {material: round(float(counts.get(material, 0)) / total, 6) for material in MATERIALS}


def top_density_cells(cells: Iterable[Mapping[str, Any]], limit: int = 20) -> List[Dict[str, Any]]:
    ranked = sorted(cells, key=lambda c: float(c.get("density", 0.0)), reverse=True)
    return [dict(cell) for cell in ranked[:limit]]
