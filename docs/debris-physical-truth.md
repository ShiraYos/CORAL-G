# Debris Physical Truth

`environment_node` is the current runtime owner of physical debris truth. This
checkout keeps the existing node name to avoid unnecessary conflicts, but the
architecture role maps to the visual/docs `physical_debris_simulation_node`.

Important correction: old fixed `CLUSTERS` were a transitional stand-in for
debris truth. That is implementation debt, not the target model. The current
model is a hidden deterministic particle cloud over free environment cells.
Particles carry material, position, velocity, acceleration, boid-style movement,
collection state, and washed-out state. See `docs/debris-simulation-model.md`.

Only `/environment_observation` and `/collection_event` are truth-to-twin
contract topics. `digital_twin_state_node`, `debris_prediction_node`, and
planners should use those contract topics rather than hidden physical truth.

`environment_generator_node` provides the typed `generate_environment_field`
service. It takes `/map`, deterministic generation parameters, and request
controls, then tiles the loaded map bounds into simulation cells using
`cell_size_m` (currently `0.20m` by default). `/map.info.resolution` remains the
occupancy-map pixel resolution; `cell_size_m` is the coarser simulation-grid
resolution. Free cells carry environment vectors; blocked and unknown cells are
marked without environmental vectors. `environment_node` calls this service when
available and keeps the existing preset field as a fallback so the demo can
still run without a live map.

The force field is an input to physical truth. The current runtime applies it to
particles. Particles that touch land/non-water should become `washed_out` and
leave active simulation. Washed-out counts are separate from collection counts.
Collected and washed-out particles are logged into cumulative dashboard counts,
then removed from the active set. Each tick respawns enough hidden particles over
free environment cells to maintain 100 active physical particles. The collection
radius is a lowered code constant, currently `0.2m`.

The digital side should receive environment evidence through
`/environment_observation` and `/twin_state`, and collection evidence through
`/collection_event`, not by reading generator or physical-truth internals
directly.

`debris_prediction_node` owns separate digital prediction state. It should own
independent prediction particles, derive normalized density cells from those
particles, and publish density as the planner-facing abstraction. It must not
consume `/dashboard` or import physical truth state from `environment_node`.
Current predicted cluster-point drift is also transitional debt.

`/dashboard` is a debug-only `std_msgs/msg/String` JSON topic with schema
`dtas.dashboard.v1`. It may expose hidden physical debris truth, including true
drifted debris positions, for dashboards, Python renderers, force-field
inspection, and tests. It must not be consumed by planners, prediction nodes, or
mission logic.

The browser preview is also debug-only. It listens to `/dashboard` and
`/debris_density_map` so teammates can visually compare physical truth against
digital prediction, but it is not part of the mission loop. The desired preview
surface is side-by-side truth particles versus predicted density, with toggles
for force layers, density, particles, and washed-out counters.

Gazebo remains the robot/world visual surface. Dashboard and Python inspection
surfaces should consume ROS output such as `/dashboard` instead of importing
simulation internals directly.

For a minimal terminal preview, run `environment_node` and then run:

```bash
python3 tools/debris_dashboard_preview.py --demo-odom
```

For a browser preview, run the ROS nodes and then run:

```bash
python3 tools/debris_dashboard_web.py --host 0.0.0.0 --port 8765 --demo-odom
```

The preview tool is a developer inspection aid. It is not part of the active
DTAS mission loop.
