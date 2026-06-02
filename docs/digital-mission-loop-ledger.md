# Digital Mission Loop Ledger

Status: superseded for future work by
[full-integration-ledger.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-ledger.md).
This ledger remains the verified history for the completed minimal digital
mission loop through Chunk 8. Do not continue new full-integration work from
this ledger unless explicitly asked to resume the old minimal-loop packet.

Product spine: complete the minimal digital mission loop around the debris work:

`/environment_observation + /collection_event + robot/base/map inputs -> /twin_state -> /debris_density_map -> /next_cell_goal -> Nav2 goal contract`

Contract decisions:
- `/twin_state` does not own prediction density.
- `debris_prediction_node` owns `/debris_density_map`.
- `field_planner_node` is the v1 utility-field planner.
- `mission_planner_node` owns the Nav2 `NavigateToPose` boundary.
- First integration test stops before live Nav2 and validates `/next_cell_goal`.

Plan fit:
- `/twin_state` is the fused world context: map cells, robot/base state,
  environment observations, material evidence, and collection history.
- `/debris_density_map` is the separate prediction output: normalized
  location/density cells for planning reward.
- `field_planner_node` joins those two digital inputs at decision time:
  `/twin_state` supplies constraints and `/debris_density_map` supplies reward.
- `/next_cell_goal` is the mission intent contract. It is not Nav2 yet.
- `mission_planner_node` converts `/next_cell_goal` into the Nav2
  `NavigateToPose` action and keeps action status internal.

Surfaced decisions:
- Keep the `/debris_density_map.cells` alias for compatibility.
- Do not use `cell_id` in `/next_cell_goal` v1. Document it only as a future
  option if stable cell identity becomes necessary.
- `field_planner_node` should use `/twin_state` for map/context and
  `/debris_density_map` for reward.
- `mission_planner_node` should validate schema/mode, map frame, and numeric
  goal shape before sending Nav2 goals; full map-bound checks can wait.
- Add planner intent to the dashboard later as a debug-only toggle layer like
  force arrows, after `/next_cell_goal` is contract-tested.

| Chunk | Status | Still on spine | Scope | Technical verification | Product-readiness verification | Direction warning |
| --- | --- | --- | --- | --- | --- | --- |
| 1. Runtime plan + contract ledger | verified | yes | Record repo-local chunk plan, accepted contracts, and handoff path. | passed: ledger/handoff created | passed: decisions reflect docs and user comments | none |
| 2. Digital twin contract hardening | verified | yes | Test `/twin_state` required sections, degraded no-map cells, robot/base/material evidence, collection history, and absence of prediction/debug truth. | passed: py_compile and 41 unit tests | passed: density remains outside `/twin_state`; debug physical particles ignored | none |
| 3. Density contract hardening | verified | yes | Test `/debris_density_map` normalization and planner-facing cleanliness. | passed: py_compile and 42 unit tests | passed: density cells are `x/y/density`, normalized, material-free, robot-free, map-free, and particle-free; `cells` alias retained | none |
| 4. Field planner contract completion | verified | yes | Test utility cleanup, idle, return-to-base, duplicate suppression, and degraded density handling. | passed: py_compile and 44 unit tests | passed: planner uses `/twin_state` for map eligibility, `/debris_density_map` for reward, suppresses duplicate goals, and keeps `cell_id` out of v1 goals | none |
| 5. Mission planner Nav2 boundary | verified | yes | Test `/next_cell_goal` to map-frame `NavigateToPose.Goal` conversion with mocked Nav2. | passed: py_compile and 48 unit tests | passed: mission planner validates idle/malformed/map-frame numeric goals, converts cleanup and return-to-base intent into `NavigateToPose.Goal`, and keeps Nav2 lifecycle internal | none |
| 6. Digital-side integration test | verified | yes | Fake inputs through twin, prediction, density, and field planner; assert `/next_cell_goal`. | passed: py_compile and 49 unit tests | passed: fake map/observation/robot/base/collection inputs produce clean `/twin_state`, planner-facing `/debris_density_map`, and map-frame `/next_cell_goal` without dashboard/debug consumption or live Gazebo/Nav2 | none |
| 7. Dashboard planner intent surface | verified | yes | Mirror `/next_cell_goal` into the debug dashboard and render planner target/status without changing mission behavior. | passed: py_compile, 50 unit tests, Docker build, Docker direct node suite | passed: dashboard shows debug-only planner panel, enabled intent toggle, utility breakdown, freshness, and target marker/robot-to-target line on canvases | none |
| 8. Dynamic current and stronger eddies | verified | yes | Make environment-field current time-varying and intensify eddy/vortex defaults while preserving map masking and dashboard surface. | passed: py_compile, 52 unit tests, Docker build, Docker direct node suite | passed: fallback/dashboard current vectors now change over time and current arrows show stronger eddy variation without changing particle seeding or planner behavior | none |

Current operational state:
- Branch: `debris-minimal-sim`.
- Preview: `http://127.0.0.1:8765/`.
- Existing uncommitted debris work is intentionally preserved.
- Docker build: `colcon build --symlink-install` passed in
  `coral-g-dashboard-preview`.
- Docker functional tests: direct node unit suite passed, 50 tests.
- Docker package tests: `colcon test --packages-select my_tb3_world` was last
  checked before Chunk 7 and failed repo-wide flake8/pep257 lint gates with
  existing style/docstring issues across legacy launch/example files and a few
  touched debris files.
- Browser preview: Planner panel and target layer verified at
  `http://127.0.0.1:8765/`. Preview-only `field_planner_node` was started with
  `min_density_reward:=0.001` so normalized density produces a visible target;
  repo planner behavior was not changed.
- Dynamic current preview: fallback field remains `preset_fallback` when no
  `/map` is available, but current vectors now refresh with `field_time_sec`;
  sample dashboard current vectors changed across a 2.5 second interval.

Human gates:
- Stop before renaming `environment_node` to `physical_debris_simulation_node`.
- Stop before changing `/next_cell_goal` schema.
- Stop before requiring live Gazebo/Nav2 tests in CI.
- Stop before adding dashboard planner-intent UI beyond a debug-only toggle
  surface.
