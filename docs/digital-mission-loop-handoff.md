# Digital Mission Loop Handoff

Active plan: [digital-mission-loop-ledger.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/digital-mission-loop-ledger.md)

Product spine: `/environment_observation + /collection_event + robot/base/map inputs -> /twin_state -> /debris_density_map -> /next_cell_goal -> Nav2 goal contract`.

Current chunk: Chunk 8, dynamic current and stronger eddies, verified.

Completed:
- Chunk 1 ledger/handoff created.
- Contract decisions recorded: `/twin_state` does not own density; planner reads `/debris_density_map`; `field_planner_node` is the v1 utility planner; `mission_planner_node` is the Nav2 boundary.
- Chunk 2 verified: `/twin_state` carries world context but excludes density and debug physical truth.
- Chunk 3 verified: `/debris_density_map` stays planner-facing with normalized `x/y/density` cells, no material/debug/robot/map fields, and the `cells` alias retained.
- Chunk 4 verified: `field_planner_node` uses `/twin_state` for map eligibility and `/debris_density_map` for reward, rejects blocked/unknown twin-map cells, suppresses duplicate goals, and keeps `cell_id` out of `/next_cell_goal`.
- Chunk 5 verified: `mission_planner_node` validates `/next_cell_goal`, ignores idle/malformed goals, accepts only map-frame numeric cleanup goals, resolves return-to-base from `/twin_state`, and converts accepted intent into `NavigateToPose.Goal`.
- Chunk 6 verified: fake map/observation/robot/base/collection inputs flow through `digital_twin_state_node`, `debris_prediction_node`, and `field_planner_node` to produce a map-frame `/next_cell_goal` without consuming dashboard/debug topics or requiring live Gazebo/Nav2.
- Chunk 7 verified: dashboard web preview mirrors `/next_cell_goal` into debug-only `planner_intent`, shows Planner status/utility/freshness, and renders a target marker plus robot-to-target line on both canvases without changing mission topics or planner behavior.
- Chunk 8 verified: `environment_field` current generation now changes with `field_time_sec`, eddy/vortex defaults are stronger, and `environment_node` passes clock time to the field service plus refreshes fallback current when the service exists but no map is available.

Next exact task:
- Decide whether to close this packet with a commit/push or address the planner threshold mismatch as a separate packet.
- The observed mismatch: normalized `/debris_density_map` cells can all sit below the current `field_planner_node.min_density_reward=0.1`, causing idle despite active debris. Preview verification used `min_density_reward:=0.001` only for the live dashboard target.
- If continuing simulation tuning, the next likely packet is parameter surfacing/documentation for field controls so eddy strength, temporal variation, and material responses can be tuned from launch/YAML instead of code defaults.

Surfaced decisions to preserve:
- Keep `/debris_density_map.cells` alias.
- Do not use `cell_id` in `/next_cell_goal` v1; document it only as a future option.
- Planner should use `/twin_state` for map/context and `/debris_density_map` for reward.
- Planner intent is now exposed only as a debug dashboard layer; it remains forbidden as mission input.

Verification commands:
- `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py`
- `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v`
- Docker closeout when useful: `docker run --rm -v "$PWD":/ws -w /ws ros:humble bash -lc 'apt-get update >/tmp/apt.log && apt-get install -y python3-colcon-common-extensions >/tmp/apt-install.log && colcon build --symlink-install'`

Latest verification:
- Docker `colcon build --symlink-install`: passed in running
  `coral-g-dashboard-preview`.
- Docker `ros2 launch my_tb3_world coral_g_nodes.launch.py --show-args`: passed.
- Docker direct node suite: passed, 52 tests.
- Docker `colcon test --packages-select my_tb3_world`: functional node tests
  passed before Chunk 7, but package test failed flake8/pep257 lint gates on
  repo-wide style/docstring issues. This lint gate was not rerun after Chunk 7.
- Browser preview `http://127.0.0.1:8765/`: responds, title is
  `CORAL-G Debris Dashboard`, 4 canvases present, debug reset visible, Planner
  panel visible, Planner intent toggle on, and target marker/robot-to-target
  line verified. Preview-only planner was started with
  `min_density_reward:=0.001` so a normalized density target is visible.
- Dynamic current check: dashboard API sampled twice over 2.5 seconds showed
  changed current vectors in fallback mode, confirming the preview field is no
  longer static.

Hard stops:
- Do not rename `environment_node`.
- Do not consume `/dashboard` or `/prediction_dashboard` from mission nodes.
- Do not make live Nav2/Gazebo required for the first digital-side integration test.
