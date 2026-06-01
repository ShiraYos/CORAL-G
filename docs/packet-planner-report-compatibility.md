# Packet: Planner Report Compatibility

Goal:
Move the planner off cluster-shaped debris assumptions while preserving current
mission behavior and ROS topic names.

Product spine:
The mission loop now plans from the digital twin's density abstraction, not from
physical particles or obsolete cluster coordinates.

Outcome:
- `field_planner_node` reads normalized `density_cells` first.
- `cells` remains accepted as a temporary compatibility alias.
- Remaining debris mass comes from `prediction_counts.active` when available.
- Legacy `clusters_remaining` is only a fallback for older density payloads.
- Cleanup goals target the selected density cell center.
- Planner logging now talks about predicted debris mass and density cells, not
  clusters.
- `field_planner_node` now uses the documented utility-field components:
  density reward, travel cost, storage penalty, fuel penalty, map risk, return
  cost, and fuel margin.
- Base-return goals use the base pose embedded in `/twin_state`, not a planner
  hard-coded origin.

Constraints:
- No topic rename.
- No planner subscription changes.
- No physical particle IDs in planner input.
- No dashboard/debug topics consumed by planner.
- No mission planner behavior change beyond density-cell targeting.

Verification:
- `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py`
- `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v`
- Unit tests confirm density-cell targeting without cluster fields and idle
  behavior from `prediction_counts.active`.
- Unit tests confirm `/next_cell_goal` includes utility components and return
  goals use the twin base pose.
