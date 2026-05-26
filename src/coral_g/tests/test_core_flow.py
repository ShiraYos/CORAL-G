import unittest

from coral_g.core import (
    DEFAULT_FIELD_POLICY,
    DEFAULT_RETURN_POLICY,
    DigitalTwinFusion,
    MaterialBelief,
    default_environment,
    fallback_map_update,
    make_collection_report,
    make_robot_state,
    plan_next_cell,
    simulate_density,
)


def _ready_twin():
    fusion = DigitalTwinFusion()
    fusion.set_environment(default_environment())
    fusion.set_map_update(fallback_map_update(scan_seen=True))
    fusion.set_material_distribution(MaterialBelief().message())
    return fusion.message()


class CoreFlowTests(unittest.TestCase):
    def test_density_is_deterministic_and_excludes_blocked_cells(self):
        twin = _ready_twin()
        density_a = simulate_density(twin, seed=23)
        density_b = simulate_density(twin, seed=23)
        self.assertEqual(density_a["density_cells"], density_b["density_cells"])
        blocked = {(cell["x"], cell["y"]) for cell in twin["map"]["blocked_cells"]}
        self.assertTrue(blocked.isdisjoint({(cell["x"], cell["y"]) for cell in density_a["density_cells"]}))

    def test_collected_cells_are_removed_from_density_targets(self):
        fusion = DigitalTwinFusion()
        fusion.set_environment(default_environment(width_m=8, height_m=10, cell_size_m=0.5, x_min=-1.75, y_min=-3.75))
        fusion.set_map_update(fallback_map_update(scan_seen=True))
        fusion.set_material_distribution(MaterialBelief().message())
        initial_density = simulate_density(fusion.message(), seed=23)
        first_cell = initial_density["density_cells"][0]

        fusion.add_collection_report(make_collection_report(first_cell, "plastic", 1))
        updated_density = simulate_density(fusion.message(), seed=23)

        remaining = {(cell["x"], cell["y"]) for cell in updated_density["density_cells"]}
        self.assertNotIn((first_cell["x"], first_cell["y"]), remaining)

    def test_planner_selects_cleanup_cell_when_resources_are_ok(self):
        twin = _ready_twin()
        density = simulate_density(twin, seed=23)
        robot = make_robot_state(fuel_level=1.0, storage_fill=0.0)
        decision = plan_next_cell(twin, density, robot, DEFAULT_FIELD_POLICY, DEFAULT_RETURN_POLICY)
        self.assertEqual(decision["status"], "ready")
        self.assertEqual(decision["mode"], "cleanup_cell")
        self.assertEqual(decision["reason"], "highest_utility_cell")

    def test_planner_forces_return_for_low_fuel_and_high_storage(self):
        twin = _ready_twin()
        density = simulate_density(twin, seed=23)
        low_fuel = plan_next_cell(twin, density, make_robot_state(fuel_level=0.1), DEFAULT_FIELD_POLICY, DEFAULT_RETURN_POLICY)
        full_storage = plan_next_cell(twin, density, make_robot_state(storage_fill=0.95), DEFAULT_FIELD_POLICY, DEFAULT_RETURN_POLICY)
        self.assertEqual(low_fuel["mode"], "return_to_base")
        self.assertEqual(low_fuel["reason"], "fuel_below_return_threshold")
        self.assertEqual(full_storage["mode"], "return_to_base")
        self.assertEqual(full_storage["reason"], "storage_above_return_threshold")

    def test_material_belief_maps_unknown_labels(self):
        belief = MaterialBelief()
        report = make_collection_report({"x": 1, "y": 2}, "unknown", 1)
        report["materials"].append({"material": "glass", "count": 2})
        message = belief.update_from_report(report)
        self.assertEqual(message["counts"]["unknown"], 3)
        self.assertEqual(message["status"], "ready")
        self.assertTrue(message["warnings"])


if __name__ == "__main__":
    unittest.main()
