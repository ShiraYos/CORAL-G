# Week 8 Lab Demo Runbook

Goal: run the lab demo and collect evidence for the Week 8 DT checklist.

Use Option A when the physical TurtleBot3 and the Gazebo twin are both shown.

## Per-Terminal Setup

Run this in every laptop terminal:

```bash
cd /home/team23/CORAL-G
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=33
export TURTLEBOT3_MODEL=burger
```

On the TurtleBot3 SSH terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=33
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
ros2 launch turtlebot3_bringup robot.launch.py
```

## Launch Order

Terminal 1, Gazebo twin:

```bash
ros2 launch my_tb3_world gazebo_twin.launch.py
```

Terminal 2, Nav2 and AMCL with the lab map:

```bash
ros2 launch my_tb3_world nav2_localization.launch.py use_sim_time:=false
```

Check that `/map` exists before starting the digital twin layer:

```bash
ros2 topic info /map
```

Terminal 3, CORAL-G digital twin and environment loop:

```bash
ros2 launch my_tb3_world coral_g_twin_demo.launch.py
```

Terminal 4, field planner:

```bash
ros2 run my_tb3_world field_planner_node --ros-args \
  -p use_sim_time:=false \
  -p min_density_reward:=0.001 \
  -p goal_wall_clearance_cells:=1 \
  -p map_cell_size_m:=0.5 \
  -p plan_rate_hz:=2.0 \
  -p republish_interval_sec:=2.0
```

Terminal 5, RViz:

```bash
rviz2 -d src/my_tb3_world/rviz/coral_g.rviz
```

Do not pass old `/ws/...` or `/root/...` paths. `nav2_localization.launch.py`
defaults to the lab map and lab Nav2 params.

## Success Checks

```bash
ros2 topic echo /dashboard --once
ros2 topic echo /twin_state --once
ros2 topic echo /next_cell_goal --once
```

Expected dashboard signs:

```text
environment.source: environment_generator_node
environment.service_available: true
environment.cell_count: 108
environment.environment_cell_count: about 42
environment.blocked_cell_count: about 56
environment.unknown_cell_count: about 10
```

If AMCL is active but the robot is not localized, set the initial pose in RViz
with "2D Pose Estimate" at the robot's real position in the saved map.

## Week 8 Checklist Evidence

Group: _____ TA: _____ Date: ______ Option: A

### 1. DT Requirements

| Requirement | What was shown | Evidence command or video moment |
| --- | --- | --- |
| Bidirectional pub/sub, physical to twin | Physical robot pose/state enters the digital twin. `robot_state_node` publishes `/robot_state`; `digital_twin_state_node` subscribes and republishes synchronized `/twin_state`. | `ros2 topic info /robot_state -v` and `ros2 topic echo /robot_state --once`. Look for pose, fuel, storage, and `source: robot_state_node`. |
| Bidirectional pub/sub, twin to physical | Digital planner publishes `/next_cell_goal`; `mission_planner_node` subscribes and sends the Nav2 goal. The command path then reaches `/cmd_vel_raw`, `twin_safety_node`, and `/cmd_vel`. | `ros2 topic info /next_cell_goal -v`, `ros2 topic echo /next_cell_goal --once`, and `ros2 node info /mission_planner_node`. |
| State synchronization, non-motion | Fuel, storage, at-base state, collection events, map status, and material evidence sync into `/twin_state`. | `ros2 topic echo /twin_state --once`. Good signs: `sync_status: ready`, `environment_source: environment_generator_node`, robot `fuel_level`, `storage_fill`, `at_base`, and map `obstacle_count`. |
| Environmental interaction | Obstacle avoidance/dynamic safety response uses the physical lidar. `twin_safety_node` subscribes to `/scan`, publishes safe velocity to both `/cmd_vel` and `/sim/cmd_vel`, and the lab launch disables Gazebo scan as a stop source with `use_sim_scan_for_stop: false`. | `ros2 node info /twin_safety_node`. During demo, place an obstacle in front of the physical robot and show `STOP: obstacle detected in real front sector` in the safety node terminal or log. |

Environmental interaction propagation:

```text
Yes. twin_safety_node consumes the physical /scan and gates both /cmd_vel and
/sim/cmd_vel, so a real obstacle stops both the physical robot and the Gazebo
twin command stream.  In the lab launch, Gazebo's /sim/scan remains available
for visualization/debugging but does not stop the physical robot.
```

### 2. Option A Validation

| Check item | Notes |
| --- | --- |
| Option A: video/demo shows both physical robot and Gazebo twin in the same scenario. | Show the real TurtleBot3, RViz, and Gazebo twin while the same `/next_cell_goal`/safety loop is active. |
| TA final note | ______________________________________________ |
| Status | Ready / Needs small fixes / Not demo-ready |

## Useful Evidence Commands

Run these from any sourced laptop terminal:

```bash
ros2 node list | sort
ros2 topic list | sort
ros2 node info /twin_safety_node
ros2 node info /digital_twin_state_node
ros2 node info /robot_state_node
ros2 node info /field_planner_node
ros2 topic info /robot_state -v
ros2 topic info /twin_state -v
ros2 topic info /next_cell_goal -v
ros2 topic info /scan -v
ros2 topic info /sim/scan -v
ros2 topic echo /robot_state --once
ros2 topic echo /twin_state --once
ros2 topic echo /dashboard --once
ros2 topic echo /debris_density_map --once
ros2 topic echo /next_cell_goal --once
```

Useful log files during a lab run:

```bash
ls -lt ~/.ros/log | head
ps -eo pid,tty,cmd | grep twin_safety_node
tail -f ~/.ros/log/python3_<PID>_*.log
```

The node-specific name is often not in the filename; match the PID from `ps` to
the `python3_<PID>_...log` file.
