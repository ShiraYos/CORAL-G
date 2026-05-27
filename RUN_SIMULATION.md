# CORAL-G Simulation Runbook

This runbook explains how to run the CORAL-G digital twin proof of concept in WSL + Docker + Gazebo + Nav2.

It is written for future agents or teammates who need to understand the terminal setup, what should happen, and how to inspect the system while it runs.

## What This Simulation Runs

The branch contains the existing TurtleBot3/Gazebo workspace plus the CORAL-G digital twin packages:

```text
src/my_tb3_world/          Existing Gazebo world and TurtleBot launch package
src/coral_g/               CORAL-G digital twin and mission-planning nodes
src/coral_g_interfaces/    CORAL-G custom ResetReseed service
```

The main runtime loop is:

```text
Gazebo TurtleBot3
  -> /scan and /odom
  -> CORAL-G robot/environment/twin/debris/planner nodes
  -> /coral_g/goal_pose
  -> Nav2 /navigate_to_pose
  -> /cmd_vel
  -> Gazebo robot moves
```

CORAL-G decides where the robot should go. Nav2 decides how to drive there.

## Expected PoC Behavior

The current PoC uses a finite set of logical trash cells inside the small map:

```text
( 0.75, -0.75)
(-0.75, -0.75)
( 0.75, -1.75)
(-0.75, -1.75)
( 0.75, -2.75)
(-0.75, -2.75)
```

Expected mission behavior:

1. Robot starts near base `(0, 0)`.
2. CORAL-G selects the highest-utility remaining trash cell.
3. Nav2 drives the robot to that cell.
4. On successful navigation, CORAL-G marks that logical cell as collected.
5. The collected cell disappears from `/coral_g/debris_density_map`.
6. Storage increases by `0.25`.
7. Fuel decreases by `0.08`.
8. After two collected cells, storage reaches `0.5`.
9. CORAL-G switches to `return_to_base`.
10. Robot returns near `(0, 0)`.
11. On return success near base, fuel resets to `1.0` and storage resets to `0.0`.
12. The robot continues with the remaining trash cells.

This is a digital-twin PoC. Trash cells are logical targets, not physical Gazebo trash objects.

## Prerequisites

- Windows with WSL Ubuntu 24.04.
- Docker Desktop running with WSL integration enabled.
- Repo available inside WSL, normally:

```bash
/home/c2irr10/turtlebot3_ws
```

- Docker image available:

```bash
docker images | grep turtlebot3_ws
```

If the image does not exist, build it from the workspace root:

```bash
cd /home/c2irr10/turtlebot3_ws
docker build -t turtlebot3_ws .
```

## Terminal Layout

Use four WSL terminals:

```text
Terminal 1: Docker container + Gazebo
Terminal 2: Nav2
Terminal 3: Initial pose + monitoring
Terminal 4: CORAL-G nodes
```

Do not use the Gazebo reset button during the demo. It can remove or desync the spawned TurtleBot. Restart the Gazebo launch instead.

## Terminal 1: Start Docker And Gazebo

Open a WSL terminal:

```bash
cd /home/c2irr10/turtlebot3_ws
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY \
  -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  -v /home/c2irr10/turtlebot3_ws:/ws \
  --user $(id -u):$(id -g) turtlebot3_ws bash
```

Inside Docker:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --packages-select my_tb3_world coral_g_interfaces coral_g
source install/setup.bash
ros2 launch my_tb3_world new_world.launch.py
```

Expected:

- Gazebo opens.
- TurtleBot3 appears in the world.
- Terminal shows entity creation success.

Keep this terminal running.

## Terminal 2: Launch Nav2

Open a second WSL terminal:

```bash
docker exec -it turtlebot3_container bash
```

Inside Docker:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="/ws/Simulation Files/map.yaml"
```

Expected:

- RViz opens.
- Nav2 logs may say AMCL needs an initial pose.

Keep this terminal running.

## Terminal 3: Set Initial Pose

Open a third WSL terminal:

```bash
docker exec -it turtlebot3_container bash
```

Inside Docker:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
```

Publish initial pose:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
```

Verify Nav2 action server:

```bash
ros2 action info /navigate_to_pose
```

Expected:

```text
Action servers: 1
    /bt_navigator
```

Why this is needed:

Nav2 uses the saved map frame. Gazebo knows the robot pose in simulation, but AMCL/Nav2 needs an initial map-frame pose to align `map -> odom`.

## Terminal 4: Launch CORAL-G

Open a fourth WSL terminal:

```bash
docker exec -it turtlebot3_container bash
```

Inside Docker:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
ros2 launch coral_g coral_g_demo.launch.py
```

Expected:

- CORAL-G nodes start.
- The robot begins moving through trash cells.
- Demo report logs appear:

```text
demo report received: run_id=coral_g_demo total_collected=...
```

Keep this terminal running.

## Monitor Commands

Run these in Terminal 3.

### Navigation Status

```bash
ros2 topic echo /coral_g/navigation_status
```

Expected normal pattern:

```text
received
active
succeeded
```

Occasional `failed` can happen if Nav2 rejects a path, but repeated rapid failures indicate a Nav2/costmap/localization issue.

### Robot State, Fuel, Storage, Pose

```bash
ros2 topic echo /coral_g/robot_state --once --full-length
```

Important fields:

```text
fuel_level
storage_fill
at_base
pose.x
pose.y
navigation_status
```

Expected resource behavior:

```text
cleanup success: fuel -= 0.08, storage += 0.25
return success near base: fuel = 1.0, storage = 0.0
```

### Current Planner Decision

```bash
ros2 topic echo /coral_g/next_cell_goal --once --full-length
```

Look for:

```text
mode: cleanup_cell
reason: highest_utility_cell
```

or:

```text
mode: return_to_base
reason: storage_above_return_threshold
```

or, after all targets are cleared:

```text
mode: idle
reason: all_debris_collected
```

### Remaining Trash Cells

```bash
ros2 topic echo /coral_g/debris_density_map --once --full-length
```

At the start there should be around six `density_cells`. As cells are collected, the list should shrink.

### Demo Report

```bash
ros2 topic echo /report --once --full-length
```

Look for:

```text
collection_stats.total
heatmap_cells
robot_summary
next_cell_goal
```

### One Combined Watch Command

```bash
watch -n 2 "ros2 topic echo /coral_g/robot_state --once --full-length && ros2 topic echo /coral_g/next_cell_goal --once --full-length"
```

Stop with `Ctrl+C`.

## Important Implementation Notes

### CORAL-G Goal Stamps

`mission_planner_node` publishes `/coral_g/goal_pose` with stamp `0`.

This is intentional. Nav2 is running with `use_sim_time:=True`, and stamp `0` means "use the latest transform." Using wall-clock timestamps caused Nav2 transform failures.

Verify:

```bash
ros2 topic echo /coral_g/goal_pose --once
```

Expected:

```text
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: map
```

### Duplicate Goal Suppression

`goal_navigation_node` remembers pending, active, and just-completed goal poses. It ignores duplicate goals so Nav2 is not preempted every second by the same target.

`mission_planner_node` also locks onto the current target while navigation is `received` or `active`. This prevents the utility planner from switching to a different trash cell mid-drive just because the robot moved and travel costs changed.

If Nav2 logs repeated:

```text
Received goal preemption request
```

then duplicate suppression or mission goal locking is not active, or old CORAL-G code is running.

### Finite Trash Targets

The debris map is finite, not every cell. It is generated from `DEFAULT_DEBRIS_TARGETS` in:

```text
src/coral_g/coral_g/core.py
```

Collected cells are removed from future density maps using collection reports stored in the twin state.

## Troubleshooting

### Robot Disappears After Gazebo Reset

Do not use the Gazebo reset button for this PoC. If the TurtleBot disappears or `/odom` stops:

1. Stop CORAL-G with `Ctrl+C`.
2. Stop Nav2 with `Ctrl+C`.
3. Stop Gazebo with `Ctrl+C`.
4. Restart from Terminal 1.
5. Publish `/initialpose` again before launching or testing CORAL-G.

### Nav2 Has No Action Server

Check:

```bash
ros2 action info /navigate_to_pose
```

If it says:

```text
Action servers: 0
```

Nav2 is not ready or not launched correctly.

### AMCL Says Initial Pose Is Missing

Publish `/initialpose` from Terminal 3 or use RViz's "2D Pose Estimate" tool.

### CORAL-G Publishes Goals But Robot Does Not Move

Check:

```bash
ros2 topic echo /coral_g/navigation_status
ros2 action info /navigate_to_pose
ros2 topic echo /coral_g/goal_pose --once
```

Also inspect the Nav2 terminal for:

```text
Failed to create plan
Goal is occupied
Start outside bounds
Timed out waiting for transform
```

### Manual Nav2 Goal Test

Stop CORAL-G, then test Nav2 directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.75, y: -0.75, z: 0.0}, orientation: {w: 1.0}}}}"
```

If this works but CORAL-G goals fail, inspect `/coral_g/goal_pose` stamp and duplicate-goal behavior.

### Verify CORAL-G Package Contracts

```bash
ros2 run coral_g contract_check
```

Expected:

```text
CORAL-G contract check passed for 9 JSON contracts.
```

## Shutdown

Recommended shutdown order:

```text
Terminal 4: Ctrl+C CORAL-G
Terminal 2: Ctrl+C Nav2
Terminal 1: Ctrl+C Gazebo
Terminal 3: exit monitor shell
```

The Docker container was started with `--rm`, so it is removed automatically when Terminal 1 exits.
