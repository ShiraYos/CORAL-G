from .contracts import REQUIRED_FIELDS, validate
from .core import (
    DEFAULT_FIELD_POLICY,
    DEFAULT_RETURN_POLICY,
    DigitalTwinFusion,
    MaterialBelief,
    default_environment,
    fallback_map_update,
    make_collection_report,
    make_demo_report,
    make_robot_state,
    plan_next_cell,
    simulate_density,
)


def build_contract_fixtures():
    material = MaterialBelief().message()
    environment = default_environment()
    map_update = fallback_map_update(scan_seen=True)
    fusion = DigitalTwinFusion()
    fusion.set_environment(environment)
    fusion.set_map_update(map_update)
    fusion.set_material_distribution(material)
    twin = fusion.message()
    density = simulate_density(twin)
    robot = make_robot_state()
    next_goal = plan_next_cell(twin, density, robot, DEFAULT_FIELD_POLICY, DEFAULT_RETURN_POLICY)
    collection = make_collection_report(next_goal["cell"], "plastic", 1)
    report = make_demo_report("contract_check", next_goal, robot, material, density, DEFAULT_FIELD_POLICY)
    return {
        "environment_field": environment,
        "map_sync_update": map_update,
        "material_distribution": material,
        "twin_state": twin,
        "debris_density_map": density,
        "robot_state": robot,
        "next_cell_goal": next_goal,
        "collection_report": collection,
        "report": report,
    }


def main() -> None:
    fixtures = build_contract_fixtures()
    failures = []
    for kind in REQUIRED_FIELDS:
        try:
            validate(kind, fixtures[kind])
        except Exception as exc:
            failures.append(f"{kind}: {exc}")
    if failures:
        raise SystemExit("CORAL-G contract check failed:\n" + "\n".join(failures))
    print(f"CORAL-G contract check passed for {len(fixtures)} JSON contracts.")


if __name__ == "__main__":
    main()
