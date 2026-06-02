# CORAL-G Full Demo Runbook

Purpose: run the full CORAL-G demo path from Gazebo/Nav2 through the digital
mission loop, without making Gazebo/Nav2 a local code gate.

## Local Code Gate

Run these before handing the repo to a Gazebo/RViz machine:

```bash
python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py
PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v
```

Expected result: compile passes and the focused node suite passes.

## Get The Branch

Pull the latest main branch:

```bash
git fetch origin
git checkout main
git pull
```

## Build

In a ROS 2 Jazzy environment (Docker image: osrf/ros:jazzy-desktop-full):

```bash
colcon build --symlink-install
source install/setup.bash
```

On PowerShell:

```powershell
colcon build --symlink-install
.\install\setup.ps1
```

## Launch Order

Preferred single launcher:

```bash
ros2 launch my_tb3_world coral_g_full_demo.launch.py \
  use_rviz:=true \
  initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0 \
  min_density_reward:=0.001
```

This starts Gazebo, TurtleBot3 spawn, SLAM/Nav2, CORAL-G nodes, and RViz.

Start the dashboard in a second terminal from the source checkout:

```bash
source install/setup.bash
python3 tools/debris_dashboard_web.py --host 0.0.0.0 --port 8765
```

Dashboard URL on the same machine:

```text
http://127.0.0.1:8765/
```

If the full launcher is too hard to debug, use the manual launch order below.

Terminal 1, Gazebo world:

```bash
ros2 launch my_tb3_world new_world.launch.py
```

Terminal 2, SLAM/Nav2 and mission planner:

```bash
ros2 launch my_tb3_world nav2_slam_navigation.launch.py \
  initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

Terminal 3, CORAL-G digital mission nodes:

```bash
ros2 launch my_tb3_world coral_g_nodes.launch.py \
  min_density_reward:=0.001
```

Terminal 4, dashboard:

```bash
python3 tools/debris_dashboard_web.py --host 0.0.0.0 --port 8765
```

Dashboard URL on the same machine:

```text
http://127.0.0.1:8765/
```

## Tiered Expectations

Tier 0, local code proof:

- Build or Python tests pass.
- Mocked Nav2 success emits `/collection_event`.
- Prediction records `observed_removed`.
- Planner can produce a next `/next_cell_goal`.

Tier 1, partner startup proof:

- `coral_g_full_demo.launch.py --show-args` works.
- Gazebo opens and TurtleBot3 spawns.
- RViz opens.
- Nav2/SLAM starts without immediate launch failure.
- CORAL-G nodes start.

Tier 2, partner topic proof:

- `/twin_state` publishes robot/base/map context.
- `/debris_density_map` publishes normalized density cells.
- `/next_cell_goal` publishes a map-frame cleanup, return, or idle intent.
- Dashboard opens and shows planner intent.

Tier 3, partner Nav2 handoff proof:

- `mission_planner_node` receives `/next_cell_goal`.
- A `NavigateToPose` goal is sent to Nav2.
- Active Nav2 goal is not canceled/replaced by planner churn.

Tier 4, partner motion proof:

- TurtleBot starts moving toward one selected cleanup target.
- RViz pose and map alignment look plausible.
- Any failure is categorized as Nav2 activation, map/pose alignment,
  controller/planner behavior, or CORAL-G topic flow.

Tier 5, full loop stretch:

- TurtleBot reaches one cleanup target.
- Navigation succeeds.
- `/collection_event` is published.
- Prediction records `observed_removed`.
- Planner advances, returns, or idles based on state.

Tier 6, lab stretch:

- Complete all configured targets or demonstrate the current continuous-belief
  behavior clearly.
- Return to base or finish in a clean idle state.
- Document any recovery behavior or local planner failures.

## Sanity Checks

Use these checks before judging robot motion:

```bash
ros2 topic echo /twin_state --once
ros2 topic echo /debris_density_map --once
ros2 topic echo /next_cell_goal --once
ros2 topic echo /collection_event --once
```

Expected observations:

- `/twin_state` has robot/base/map context.
- `/debris_density_map` has normalized `density_cells`.
- `/next_cell_goal` eventually reports `mode: cleanup` or a return/idle state.
- `/collection_event` appears after a successful cleanup navigation result or
  physical particle pickup.

## Partner Pre-Lab Acceptance

Minimum partner-ready acceptance:

- Gazebo world opens.
- RViz/Nav2 reaches active state.
- CORAL-G nodes launch with the demo planner threshold.
- Dashboard opens and shows planner intent.
- A real `/next_cell_goal` reaches the mission planner.

Stretch acceptance:

- TurtleBot starts moving toward one cleanup target.
- Navigation succeeds.
- `/collection_event` is published.
- Prediction records observed removal and planner advances, returns, or idles.

## Triage

If no planner target appears:

- Check `/twin_state` has map cells and robot state.
- Check `/debris_density_map` has nonzero normalized density.
- Confirm `coral_g_nodes.launch.py` was started with
  `min_density_reward:=0.001` or the demo params file.

If Nav2 does not move:

- Confirm Nav2 lifecycle nodes are active in RViz.
- Confirm `/next_cell_goal` is in `frame_id: map`.
- Check initial pose alignment in RViz.
- Check controller/planner error logs before changing CORAL-G contracts.

If collection does not feed back:

- Confirm `mission_planner_node` logs goal success.
- Check `/collection_event`.
- Check `debris_prediction_node` dashboard lifetime counts for
  `observed_removed`.
