# Resetting And Re-running The CORAL-G Simulation

This guide explains how to cleanly stop and restart the CORAL-G Gazebo/Nav2 simulation when you want to run another instance of the PoC.

## Short Version

Do not use the Gazebo reset button for this PoC. It can remove or desync the spawned TurtleBot.

Use this shutdown order:

```text
1. Stop CORAL-G
2. Stop Nav2
3. Stop Gazebo / Docker container
4. Start again from Gazebo -> Nav2 -> CORAL-G
```

The CORAL-G launch now publishes the Nav2 initial pose automatically, so you should not need to manually run the long `/initialpose` command anymore.

## Why Gazebo Reset Is Not Recommended

The TurtleBot is spawned by the ROS launch process. Pressing reset inside Gazebo may reset the world physics but leave ROS, Nav2, AMCL, transforms, or the spawned robot entity in an inconsistent state.

Symptoms after using Gazebo reset can include:

- TurtleBot disappears from the Gazebo entity tree.
- `/odom` stops publishing.
- Nav2 still thinks the robot is at the old pose.
- CORAL-G keeps publishing goals, but the robot does not move correctly.
- AMCL asks for a new initial pose.

The reliable fix is a clean restart.

## Clean Shutdown

### Terminal 4: CORAL-G

In the terminal running:

```bash
ros2 launch coral_g coral_g_demo.launch.py
```

press:

```text
Ctrl+C
```

Wait until all CORAL-G node processes stop.

Optional check from another Docker terminal:

```bash
ros2 node list | grep -E 'twin_input|material_belief|digital_twin|debris_simulation|robot_state|field_planner|mission_planner|goal_navigation|collection_report|demo_control|initial_pose'
```

Expected: no output.

### Terminal 2: Nav2

In the terminal running:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="/ws/Simulation Files/map.yaml"
```

press:

```text
Ctrl+C
```

Wait until Nav2 and RViz close.

### Terminal 1: Gazebo

In the terminal running:

```bash
ros2 launch my_tb3_world new_world.launch.py
```

press:

```text
Ctrl+C
```

This also stops the Docker container if this was the original `docker run` terminal.

If the shell stays inside Docker, type:

```bash
exit
```

## Clean Restart

Use three or four WSL terminals.

### Terminal 1: Start Docker And Gazebo

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

Wait until Gazebo opens and the TurtleBot appears.

### Terminal 2: Start Nav2

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

Wait until RViz opens and Nav2 is running.

### Terminal 3: Start CORAL-G

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

`coral_g_demo.launch.py` now starts `initial_pose_publisher` first. It publishes `/initialpose` several times at startup:

```text
x = 0.0
y = 0.0
yaw = 0.0
frame_id = map
```

This replaces the manual long `/initialpose` command for the normal start position.

## Verifying Initial Pose Was Published

In a monitor terminal:

```bash
docker exec -it turtlebot3_container bash
```

Inside Docker:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
ros2 action info /navigate_to_pose
```

Expected:

```text
Action servers: 1
    /bt_navigator
```

Check CORAL-G navigation:

```bash
ros2 topic echo /coral_g/navigation_status
```

Expected pattern:

```text
received
active
succeeded
```

## If Automatic Initial Pose Does Not Take

If Nav2 still logs:

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose
```

then CORAL-G may have been launched before Nav2/AMCL subscribed to `/initialpose`.

Fix:

1. Stop CORAL-G with `Ctrl+C`.
2. Confirm Nav2 is running.
3. Launch CORAL-G again.

If needed, run the manual fallback once:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}"
```

## Resetting Only CORAL-G Logic

If Gazebo and Nav2 are healthy and you only want to restart the digital twin mission:

1. Stop CORAL-G with `Ctrl+C`.
2. Relaunch:

```bash
ros2 launch coral_g coral_g_demo.launch.py
```

This resets the CORAL-G in-memory state:

- collection history,
- material belief,
- simulated fuel/storage,
- current mission progress.

Gazebo robot pose does not reset in this case. If the robot is not near base, CORAL-G will start from the robot's current `/odom` pose.

For a truly fresh run from the start position, restart Gazebo and Nav2 too.

## Useful Post-reset Checks

Check robot odometry:

```bash
ros2 topic echo /odom --once
```

Check CORAL-G robot state:

```bash
ros2 topic echo /coral_g/robot_state --once --full-length
```

Check remaining trash cells:

```bash
ros2 topic echo /coral_g/debris_density_map --once --full-length
```

Check planner decision:

```bash
ros2 topic echo /coral_g/next_cell_goal --once --full-length
```

Check Nav2 goal status:

```bash
ros2 topic echo /coral_g/navigation_status
```

