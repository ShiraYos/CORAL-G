# CORAL-G Digital Twin PoC

This workspace implements the 2-week CORAL-G build plan as a ROS 2 Python package plus a small interface package.

## What Is Included

- `src/coral_g`: ROS 2 nodes and pure-Python digital-twin logic.
- `src/coral_g_interfaces`: `ResetReseed` service with `{ reason, random_seed } -> { accepted, message }`.
- `contract_check`: a fast smoke check for every JSON topic contract.
- Tests for contract validity, deterministic debris density, utility planning, return behavior, and material belief updates.

## Build And Run

From this workspace root in a ROS 2 environment:

```powershell
colcon build
.\install\setup.ps1
ros2 launch coral_g coral_g_demo.launch.py
```

Run the contract smoke check:

```powershell
ros2 run coral_g contract_check
```

Run the pure-Python tests without launching ROS:

```powershell
$env:PYTHONPATH="src/coral_g"
python -m unittest discover src/coral_g/tests
```

## Implemented Nodes

- `twin_input_node`: publishes `/coral_g/environment_field` and `/coral_g/map_sync_update`.
- `material_belief_node`: publishes `/coral_g/material_distribution` from `/coral_g/collection_report`.
- `digital_twin_state`: fuses environment, map, material, and collection inputs into `/coral_g/twin_state`.
- `debris_simulation_node`: publishes deterministic `/coral_g/debris_density_map` and serves `reset_reseed`.
- `robot_state_node`: publishes `/coral_g/robot_state`.
- `field_planner_node`: publishes `/coral_g/next_cell_goal`.
- `mission_planner_node`: converts next-cell/return decisions into `/coral_g/goal_pose`.
- `goal_navigation_node`: bridges `/coral_g/goal_pose` into Nav2 `NavigateToPose`.
- `collection_report_node`: publishes `/coral_g/collection_report` and `/report`.
- `demo_control_surface`: subscribes to `/report` and owns the manual reset client boundary.

## Development Gates

Use `contract_check` after each layer is built. It validates the message envelope, schema names, required fields, and material labels for every JSON contract so schema drift is caught before full Gazebo/Nav2 integration.
