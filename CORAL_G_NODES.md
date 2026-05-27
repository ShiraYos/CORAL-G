# CORAL-G Implemented Nodes And Behaviors

This document describes the ROS 2 nodes implemented in the `coral_g` package, what each node publishes/subscribes to, and the logic it uses when new data arrives.

The current system is a functional proof-of-concept digital twin loop. It connects Gazebo/TurtleBot3 sensor and odometry data to CORAL-G planning, sends goals to Nav2, and reports mission progress. Debris, collection, fuel, storage, and material classification are currently logical/simulated layers rather than physical Gazebo object pickup.

## Topic And Data Model Overview

Most CORAL-G topics use `std_msgs/msg/String` containing JSON with a schema field. The main exceptions are ROS-native navigation and sensor messages.

| Topic | Type | Main Role |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Live Gazebo/TurtleBot LiDAR input. |
| `/odom` | `nav_msgs/msg/Odometry` | Live Gazebo/TurtleBot robot pose input. |
| `/coral_g/environment_field` | JSON string | Synthetic cleanup-zone/environment grid. |
| `/coral_g/map_sync_update` | JSON string | Simplified free/blocked map evidence from LiDAR. |
| `/coral_g/material_distribution` | JSON string | Belief over collected material labels. |
| `/coral_g/twin_state` | JSON string | Fused digital-twin state. |
| `/coral_g/debris_density_map` | JSON string | Synthetic debris-density prediction. |
| `/coral_g/robot_state` | JSON string | Live robot pose plus simulated fuel/storage. |
| `/coral_g/next_cell_goal` | JSON string | Planner decision: cleanup cell, return-to-base, or idle. |
| `/coral_g/goal_pose` | `geometry_msgs/msg/PoseStamped` | ROS navigation goal derived from planner decision. |
| `/coral_g/navigation_status` | `std_msgs/msg/String` | Simplified Nav2 status: received, active, succeeded, failed, timeout, waiting_for_nav2. |
| `/coral_g/collection_report` | JSON string | Logical collection event after successful navigation. |
| `/report` | JSON string | Demo summary report for the control surface. |

## `twin_input_node`

**Purpose:** Provides the digital twin with environmental field data and map evidence.

**Publishes:**

- `/coral_g/environment_field`
- `/coral_g/map_sync_update`

**Subscribes:**

- `/scan`

**Behavior:**

- Every tick, it publishes a synthetic environment grid.
- In the current Gazebo-map-aware setup, the default ROS grid is constrained to the small Nav2 map:
  - `zone_width_m = 8`
  - `zone_height_m = 10`
  - `zone_x_min = -1.75`
  - `zone_y_min = -3.75`
  - `cell_size_m = 0.5`
- This produces cells from approximately:
  - `x = -1.75` to `1.75`
  - `y = -3.75` to `0.75`
- For each cell, it creates synthetic environmental values:
  - current vector,
  - wind vector,
  - wave value.

**Reaction to `/scan`:**

- When a `LaserScan` arrives, the node samples scan readings.
- Every 12th reading is converted into a rough cell observation.
- If a sampled reading is below about `95%` of max range, it becomes an observed obstacle.
- Otherwise, it becomes free-space evidence.
- If no free cells are detected, it publishes a deterministic fallback set of free cells.

**Important limitation:**

The LiDAR-to-map conversion is intentionally simple. It is not a full SLAM/map-building algorithm. It only gives the digital twin basic free/blocked evidence.

## `material_belief_node`

**Purpose:** Maintains a material distribution belief from collection reports.

**Publishes:**

- `/coral_g/material_distribution`

**Subscribes:**

- `/coral_g/collection_report`

**Behavior:**

- Starts with zero counts for:
  - `plastic`
  - `foam`
  - `rubber`
  - `metal`
  - `unknown`
- Every second, it publishes the current belief.
- When a collection report arrives, it reads each material item and increments the matching material count.
- Unsupported material labels are mapped to `unknown`.
- It recalculates probabilities as:

```text
probability(material) = count(material) / total_collected_count
```

**Status logic:**

- If total count is `0`, status is `no_observations`.
- If at least one item has been reported, status is `ready`.

## `digital_twin_state`

**Purpose:** Fuses the environment, map, material belief, and collection updates into one digital twin state.

**Publishes:**

- `/coral_g/twin_state`

**Subscribes:**

- `/coral_g/environment_field`
- `/coral_g/map_sync_update`
- `/coral_g/material_distribution`
- `/coral_g/collection_report`

**Behavior:**

- Stores the latest environment message.
- Stores the latest map update.
- Stores the latest material distribution.
- Stores the latest collection reports, keeping only the most recent 20.

**Map fusion logic:**

- `water_cells` come from the environment grid.
- `blocked_cells` come from observed obstacles in the map update.
- `free_space_cells` come from the map update.
- `unknown_cells` are environment cells that are not known free and not blocked.

**Status logic:**

- If any required input is missing, sync status is `waiting_for_inputs`.
- If all required inputs exist, sync status is `ready`.
- If the map update status is `degraded` or `waiting_for_scan`, sync status becomes `degraded`.

## `debris_simulation_node`

**Purpose:** Produces a synthetic debris-density map from the fused twin state.

**Publishes:**

- `/coral_g/debris_density_map`

**Subscribes:**

- `/coral_g/twin_state`

**Services:**

- `reset_reseed`

**Behavior:**

- Waits until a twin state exists.
- Takes all water cells from the twin state.
- Removes blocked cells.
- For each active PoC trash target, finds the nearest valid environment cell and publishes a synthetic density there.

**Debris target model:**

The PoC uses a finite list of three separated debris targets inside the small Nav2 map instead of making every grid cell look like possible trash:

```text
(0.75, -0.75)
(-0.75, -0.75)
(0.75, -1.75)
```

For each target, the node finds the nearest valid environment cell and publishes it as a debris-density cell. A small deterministic random jitter is added to the configured density, so the output stays deterministic for the same random seed but still has varied scores.

**Clearing logic:**

- Collection reports are read from the twin state.
- If a target is within `0.45 m` of a collected location, it is removed from the future density map.
- This means successful cleanup clears that target from the remaining mission.

**Parameters:**

- `debris_count`, default `100`; for the finite PoC target model this acts as a cap on how many predefined targets are active.
- `simulation_horizon_sec`, default `60.0`
- `random_seed`, default `23`
- `tick_rate_hz`, default `1.0`

**Reset/reseed behavior:**

- Calling `reset_reseed` changes the `random_seed`.
- The node immediately recalculates the density map after accepting the reset.

**Important limitation:**

This is a logical debris predictor. It does not simulate physical floating debris objects in Gazebo.

## `robot_state_node`

**Purpose:** Publishes the robot state used by CORAL-G planning.

**Publishes:**

- `/coral_g/robot_state`

**Subscribes:**

- `/odom`
- `/coral_g/navigation_status`

**Behavior:**

- Reads live Gazebo/TurtleBot odometry from `/odom`.
- Converts odometry pose into:
  - `x`
  - `y`
  - `heading`
- Publishes this pose into `/coral_g/robot_state`.

**Heading calculation:**

- Reads quaternion `z` and `w` from odometry orientation.
- Converts yaw using:

```text
heading = atan2(2 * w * z, 1 - 2 * z * z)
```

**Simulated resource logic:**

- Starts with:
  - `fuel_level = 1.0`
  - `storage_fill = 0.0`
- When navigation status becomes `succeeded` for a cleanup goal:
  - fuel decreases by `0.08`
  - storage increases by `0.25`
- When navigation status becomes `succeeded` for a return-to-base goal near base:
  - fuel resets to `1.0`
  - storage resets to `0.0`
- Values are clamped:
  - fuel stays between `0.0` and `1.0`
  - storage stays between `0.0` and `1.0`

**Base logic:**

- Base is currently `(0.0, 0.0, 0.0)`.
- `at_base` is true if the robot is within `0.5 m` of base.

## `field_planner_node`

**Purpose:** Chooses the next cleanup cell or decides to return to base.

**Publishes:**

- `/coral_g/next_cell_goal`

**Subscribes:**

- `/coral_g/twin_state`
- `/coral_g/debris_density_map`
- `/coral_g/robot_state`

**Behavior:**

- Waits until twin state, density map, and robot state are all available.
- If any are missing, publishes an idle/waiting decision.
- If robot fuel is below the return threshold, chooses return-to-base.
- If robot storage is above the return threshold, chooses return-to-base.
- Otherwise, evaluates every candidate density cell and chooses the one with highest utility.

**Return thresholds:**

```text
fuel <= 0.25 -> return_to_base
storage >= 0.5 -> return_to_base
```

**Utility calculation:**

For each candidate cell:

```text
density_reward = density * density_reward_weight
travel_cost = distance(robot_pose, cell) * travel_cost_weight
return_cost = distance(cell, base) * return_reserve_weight
storage_penalty = storage_fill * storage_penalty_weight
fuel_penalty = (1 - fuel_level) * fuel_penalty_weight

utility =
  density_reward
  - travel_cost
  - return_cost
  - storage_penalty
  - fuel_penalty
```

Default weights:

```text
density_reward = 1.0
travel_cost = 0.2
storage_penalty = 0.5
fuel_penalty = 0.5
return_reserve = 0.2
```

**Output modes:**

- `cleanup_cell`: go to the highest-utility remaining cleanup cell.
- `return_to_base`: go back to base because fuel/storage thresholds require it.
- `idle`: no valid goal is available yet, or all debris targets have been collected and the robot is already back at base.

When all debris targets have been collected and the robot is not at base yet, the planner publishes a final `return_to_base` decision with reason `all_debris_collected_return_to_base`. Once that return succeeds and the robot is near base, the planner idles with reason `mission_complete`.

## `mission_planner_node`

**Purpose:** Converts the JSON planner decision into a ROS navigation pose.

**Publishes:**

- `/coral_g/goal_pose`

**Subscribes:**

- `/coral_g/next_cell_goal`
- `/coral_g/navigation_status`
- `/coral_g/robot_state`

**Behavior:**

- Ignores planner messages unless `status` is `ready`.
- Ignores planner messages with mode `idle`.
- If mode is `cleanup_cell`, it uses the selected planner cell.
- If mode is `return_to_base`, it uses the configured base pose.
- Publishes a `PoseStamped` in the `map` frame.

**Orientation logic:**

- Converts target heading to quaternion using yaw:

```text
z = sin(heading / 2)
w = cos(heading / 2)
```

## `goal_navigation_node`

**Purpose:** Bridges CORAL-G goals into Nav2.

**Publishes:**

- `/coral_g/navigation_status`

**Subscribes:**

- `/coral_g/goal_pose`

**Action client:**

- `/navigate_to_pose`

**Behavior:**

- Receives a `PoseStamped` goal from `/coral_g/goal_pose`.
- Rejects it if the frame is not `map`.
- Publishes `received` when a valid goal message arrives.
- Checks whether the Nav2 `NavigateToPose` action server is available.
- If Nav2 is unavailable, publishes `waiting_for_nav2`.
- If Nav2 accepts the goal, publishes `active`.
- If Nav2 returns success, publishes `succeeded`.
- If Nav2 rejects or fails the goal, publishes `failed`.

**Timeout behavior:**

- Tracks active goal age.
- If a goal exceeds `goal_timeout_sec`, default `120.0`, it cancels the goal and publishes `timeout`.

## `collection_report_node`

**Purpose:** Creates logical collection reports and demo summaries.

**Publishes:**

- `/coral_g/collection_report`
- `/report`

**Subscribes:**

- `/coral_g/next_cell_goal`
- `/coral_g/robot_state`
- `/coral_g/material_distribution`
- `/coral_g/debris_density_map`
- `/coral_g/navigation_status`

**Behavior:**

- Stores the latest planner goal, robot state, material belief, and density map.
- Every 5 seconds, publishes a demo report on `/report`.
- When navigation status becomes `succeeded`, it creates a logical collection report only if the latest goal was a `cleanup_cell`.
- Return-to-base success does not count as collection.

**Collection logic:**

- Uses the latest `next_cell_goal` location.
- Uses `default_material`, currently `unknown`.
- Uses count `1`.
- Uses `collection_radius_m`, default `0.75`.

**Important limitation:**

This node does not check whether a physical debris object exists in Gazebo. Collection is triggered by navigation success, so it is a logical mission-progress event.

## `demo_control_surface`

**Purpose:** Provides a simple demo-facing monitor and reset client boundary.

**Subscribes:**

- `/report`

**Service client:**

- `reset_reseed`

**Behavior:**

- Receives demo reports.
- Logs the run id and total collected count.
- Owns a reset client for the debris simulation boundary, though the current demo mainly uses it as a monitoring surface.

## Contract And Validation Helpers

The package also includes `contract_check`, which validates JSON topic contracts.

Run:

```bash
ros2 run coral_g contract_check
```

It checks that all expected JSON messages include:

- correct schema name,
- required fields,
- valid material labels for material distribution.

This is useful before running Gazebo/Nav2 because it catches schema drift between nodes.

## Verified Behaviors In Gazebo/Nav2

The following behaviors were verified in the current setup:

- Gazebo launches the custom world.
- TurtleBot3 spawns.
- `/scan` is bridged from Gazebo into ROS.
- `/odom` is bridged from Gazebo into ROS.
- CORAL-G reads live `/odom` and publishes live `/coral_g/robot_state`.
- CORAL-G computes a finite debris-density map from separated PoC target cells.
- CORAL-G chooses cleanup goals from utility calculations, avoids already collected cells, and returns to base once storage is full.
- CORAL-G publishes `/coral_g/goal_pose`.
- Nav2 accepts goals through `/navigate_to_pose`.
- Nav2 publishes velocity commands to move the robot in Gazebo.
- CORAL-G receives `succeeded` navigation status.
- Logical collection reports increase total collected count.
- When fuel becomes low or storage becomes full, planner switches to `return_to_base`.

## Current Functional Status

Fully working proof-of-concept:

- Digital twin message pipeline.
- Live robot pose integration.
- Finite synthetic debris target calculation with cleared-cell removal.
- Utility-based field planning.
- Nav2 goal handoff.
- Gazebo robot movement.
- Logical mission reporting.
- Return-to-base decision logic.

Not yet physically realistic:

- No physical debris objects are spawned or removed in Gazebo.
- No camera/object detector is used.
- Material classification is simulated.
- Collection is based on navigation success, not object contact or pickup.
- Fuel and storage are simulated internal state, not real robot telemetry.
- Environmental current/wind/wave values are preset synthetic values.
