# CORAL-G Full System Context

## What This Is

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The visual graph describes the implementation contract: components communicate through explicit ROS topics, actions, services, and parameter nodes, and each visible node has a stable build/review packet.

Use this artifact when another assistant, teammate, or review session needs the whole system context instead of a single selected node. The top sections explain the graph and system as one architecture; the per-node packets below remain the detailed implementation briefs.

## Current System In One Pass

1. TurtleBot/Gazebo and Nav2 provide the implemented robot/navigation base: LiDAR arrives on `/scan`, Nav2 executes `NavigateToPose`, and velocity commands go to `/cmd_vel`.
2. `twin_input_node` is the input adapter. It publishes static/mock ocean conditions to `/coral_g/environment_field` and robot-observed map evidence to `/coral_g/map_sync_update`.
3. `digital_twin_state` fuses environment, map evidence, material belief, and collection updates into `/coral_g/twin_state`. It owns world belief only.
4. `debris_simulation_node` predicts debris reward density from the fused twin state and publishes `/coral_g/debris_density_map`. It does not own live robot telemetry or navigation decisions.
5. `robot_state_node` publishes live telemetry on `/coral_g/robot_state`: pose, fuel, storage, base-state, and navigation status.
6. `field_planner_node` joins twin state, debris density, and robot state into a utility field. It chooses one next cell or a return mode and publishes `/coral_g/next_cell_goal`.
7. `mission_planner_node` converts the field planner's JSON decision into the `/coral_g/goal_pose` message that Nav2 can execute.
8. `goal_navigation_node` bridges CORAL-G goals into Nav2's `NavigateToPose` action and publishes simplified navigation status.
9. `collection_report_node` reports collection events and demo summaries. `material_belief_node` updates observed material probabilities from collection reports, starting from no observations and preserving `unknown`.
10. `demo_control_surface` is a manual/debug surface for optional reset/reseed calls and report viewing, not an autonomous planning component.

## Contract Principles

- Preserve existing topic names, schema names, message types, action/service boundaries, parameter ownership, and graph links unless the team explicitly approves a redesign.
- Components should connect through the topic/action/service/parameter nodes shown in the graph, not hidden direct dependencies.
- Use topic streams for continuous state and events. Keep services limited to synchronous commands such as the optional reset/reseed control.
- The utility-field planner replaces separate queue, target-selection, and hotspot concepts. Density is a reward field; travel, return reserve, fuel, storage, and map confidence shape final utility.
- Nav2 remains the path executor. CORAL-G decides the next goal cell or return behavior; Nav2 computes and follows the actual path.
- `return_policy` owns the base pose/reference used for return behavior. `state_defaults` owns storage, fuel, and telemetry tick defaults only.
- `/coral_g/twin_state`, `/coral_g/debris_density_map`, and `/coral_g/robot_state` must stay distinct: world belief, predicted debris reward, and live robot telemetry are separate inputs joined by the planner.
- Material belief starts empty/no-observations. Do not add a default material prior in the simplified v1 contract.

## Plain-Language Terms

- Node: a running software component or adapter.
- Topic: a named ROS channel where publishers write messages and subscribers read them.
- Schema: the agreed JSON shape for one topic.
- Pose: position plus orientation. In 2D examples this usually means `x`, `y`, and heading/yaw.
- `frame_id: "map"`: coordinates are in the shared world map, not relative to the robot body.
- Density: predicted debris reward in a cell, not a guarantee that debris is physically present.
- Utility: the planner's final score for a cell after adding rewards and subtracting costs/risks.
- Cost or penalty: a reason a cell becomes less attractive, such as travel distance, low fuel, full storage, low map confidence, or blocked space.
- Status: whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- Unknown: an intentional category for missing, unobserved, or unrecognized evidence. Preserve it instead of guessing.
- Goal: the next place the robot should go. `/coral_g/next_cell_goal` is the planner's JSON decision; `/coral_g/goal_pose` is the ROS/Nav2 pose message.

## Graph Node Inventory

- `twin_input_node` (Component, CORAL-G adapter): twin_input_node - Simplified PoC input adapter that feeds preset ocean conditions and robot-observed map evidence into the digital twin.
- `material_belief_node` (Component, CORAL-G custom DTAS): material_belief_node - Maintains observed material counts and probabilities after collection reports; starts with unknown/no observations.
- `debris_simulation_node` (Component, CORAL-G custom DTAS): debris_simulation_node - Predicts a debris density field from fused twin state, keeping debris reward out of blocked or non-water cells.
- `field_planner_node` (Component, CORAL-G custom DTAS): field_planner_node - Scores each map grid cell with a utility value from debris density, robot telemetry, twin map confidence, and return constraints, then emits one next-cell goal.
- `mission_planner_node` (Component, CORAL-G custom DTAS): mission_planner_node - Converts field-planner next-cell or return-to-base decisions into map-frame robot goals that Nav2 can execute.
- `goal_navigation_node` (Component, CORAL-G adapter): goal_navigation_node - Existing bridge from CORAL-G map-frame goals into Nav2 execution; Nav2 calculates and follows the actual path.
- `nav2` (Component, ROS-native / prebuilt): Nav2 - Navigation stack that plans and executes robot movement from map-frame goals.
- `turtlebot_gazebo` (Component, ROS-native / prebuilt): TurtleBot / Gazebo - Physical or simulated robot that executes velocity commands and exposes LiDAR data.
- `digital_twin_state` (Component, CORAL-G custom DTAS): digital_twin_state - Fused world-state node for environment, observed map, material belief, and collection feedback; it does not copy live robot telemetry or prediction output.
- `robot_state_node` (Component, CORAL-G adapter): robot_state_node - Publishes live robot telemetry: position/heading, storage, fuel, base-state, and navigation status.
- `collection_report_node` (Component, CORAL-G custom DTAS): collection_report_node - Reports raw collection observations and publishes the requested demo mission summary.
- `demo_control_surface` (Component, CORAL-G custom DTAS): demo_control_surface - Manual demo or developer control surface for optional reset/reseed calls.
- `topic_environment_field` (Topic, CORAL-G custom DTAS): /coral_g/environment_field - External/static environment forces: mode, frame_id, cleanup zone, cell_size_m, and grid cells with x/y/current/wind/wave fields. Robot-observed obstacles do not belong here.
- `topic_material_distribution` (Topic, CORAL-G custom DTAS): /coral_g/material_distribution - Observed material belief: counts and normalized probabilities for plastic, foam, rubber, metal, and unknown plus status/warnings. Starts as no_observations.
- `topic_map_sync_update` (Topic, CORAL-G adapter): /coral_g/map_sync_update - Robot-observed map update: frame_id, observer pose (position + heading), observed_obstacles, free_space_cells, map_confidence, and status.
- `topic_twin_state` (Topic, CORAL-G custom DTAS): /coral_g/twin_state - Fused world belief: environment_status, map water/blocked/unknown cells, material_belief, collection_updates, and sync_status. Live robot telemetry and prediction density stay in their own topics.
- `topic_debris_density_map` (Topic, CORAL-G custom DTAS): /coral_g/debris_density_map - Prediction output: frame_id, cell_size_m, simulation_horizon_sec, status, and density_cells with x/y/density/material_hint only for valid water/free cells. Density is a planning reward signal, not guaranteed physical debris.
- `topic_next_cell_goal` (Topic, CORAL-G custom DTAS): /coral_g/next_cell_goal - Single field-planner decision: status, mode, map-frame cell pose (position + heading), utility score, density reward, travel cost, return cost, fuel margin, storage state, and reason.
- `topic_goal_pose` (Topic, CORAL-G adapter): /coral_g/goal_pose - Map-frame navigation target pose: header.frame_id must be map; pose means position plus orientation, stored as x/y/z and a quaternion.
- `topic_navigation_status` (Topic, CORAL-G adapter): /coral_g/navigation_status - Navigation state string: waiting_for_nav2, ready, received, active, succeeded, canceled, timeout, or failed.
- `topic_scan` (Topic, ROS-native / prebuilt): /scan - LiDAR scan ranges, angles, and range limits from TurtleBot/Gazebo; consumed by Nav2 and summarized by twin_input_node.
- `topic_cmd_vel` (Topic, ROS-native / prebuilt): /cmd_vel - Velocity command for the robot; Nav2 is the only intended publisher during navigation demos.
- `topic_robot_state` (Topic, CORAL-G adapter): /coral_g/robot_state - Robot state summary: map-frame pose (position + heading), storage_fill, fuel_level, at_base, and navigation_status.
- `topic_collection_report` (Topic, CORAL-G custom DTAS): /coral_g/collection_report - Collection event: map-frame location, materials with counts, total count, collection_radius_m, and status.
- `topic_report` (Topic, CORAL-G custom DTAS): /report - Human/demo summary: run_id, next_cell_goal, collection_stats, distribution_shift, robot_summary, field_policy, and heatmap_cells.
- `action_navigate_to_pose` (Action, ROS-native / prebuilt): NavigateToPose - Goal: map-frame target pose (position + orientation); feedback: navigation progress and current pose (position + orientation); result: succeeded, canceled, or failed.
- `service_reset_reseed` (Service, CORAL-G custom DTAS): reset_reseed - Request: { reason, random_seed }; response: { accepted, message }.
- `param_preset_ocean_conditions` (Parameter, CORAL-G custom DTAS): preset_ocean_conditions - Define the deterministic ocean-current, wind, wave, cleanup-zone, and grid defaults used when no external API is connected. Default/value: 10m x 10m mock current/wind/wave grid, 1m cells.
- `param_simulation_defaults` (Parameter, CORAL-G custom DTAS): debris_count=100 - Seed enough virtual debris particles to make the density field visible and repeatable. Default/value: count=100, horizon=60s, seed=23.
- `param_field_policy` (Parameter, CORAL-G custom DTAS): field_policy - Score each reachable grid cell as a utility field while enforcing enough fuel margin to return to base. Default/value: density_reward=1.0, travel_cost=0.2, storage_penalty=0.5, fuel_penalty=0.5, return_reserve=0.2.
- `param_return_policy` (Parameter, CORAL-G custom DTAS): return_policy - Defines the Gazebo/map base pose and switches from next-cell cleanup goals to base goals before storage or fuel becomes critical. Default/value: base=(0,0,0), storage=0.8, fuel=0.25.
- `param_goal_timeout` (Parameter, CORAL-G custom DTAS): goal_timeout_sec=120 - Cancel a navigation goal that exceeds the demo timeout window. Default/value: 120.0.
- `param_robot_state_defaults` (Parameter, CORAL-G custom DTAS): state_defaults - Track storage, fuel, and telemetry tick behavior; base pose comes from return_policy. Default/value: storage_capacity_items, fuel_capacity_norm, state_tick_rate_hz.
- `param_collection_defaults` (Parameter, CORAL-G custom DTAS): collection_defaults - Constrain manual/demo collection reports to known v1 material labels. Default/value: allowed_materials, default_material, collection_radius_m.

## Graph Link Inventory

- `twin-input-publishes-environment-field`: twin_input_node -> /coral_g/environment_field (Publish; publishes)
- `environment-field-subscribed-by-twin`: /coral_g/environment_field -> digital_twin_state (Subscribe; updates twin)
- `material-publishes-distribution`: material_belief_node -> /coral_g/material_distribution (Publish; publishes)
- `material-distribution-subscribed-by-twin`: /coral_g/material_distribution -> digital_twin_state (Subscribe; updates twin)
- `material-distribution-subscribed-by-collection-summary`: /coral_g/material_distribution -> collection_report_node (Subscribe; summary input)
- `twin-input-publishes-map-sync-update`: twin_input_node -> /coral_g/map_sync_update (Publish; publishes)
- `map-sync-update-subscribed-by-twin`: /coral_g/map_sync_update -> digital_twin_state (Subscribe; updates twin map)
- `twin-publishes-state`: digital_twin_state -> /coral_g/twin_state (Publish; publishes)
- `twin-state-subscribed-by-debris`: /coral_g/twin_state -> debris_simulation_node (Subscribe; constrains prediction)
- `twin-state-subscribed-by-field-planner`: /coral_g/twin_state -> field_planner_node (Subscribe; utility constraints)
- `debris-publishes-density`: debris_simulation_node -> /coral_g/debris_density_map (Publish; publishes)
- `density-subscribed-by-field-planner`: /coral_g/debris_density_map -> field_planner_node (Subscribe; reward field input)
- `density-subscribed-by-collection-summary`: /coral_g/debris_density_map -> collection_report_node (Subscribe; heatmap input)
- `field-planner-publishes-next-cell`: field_planner_node -> /coral_g/next_cell_goal (Publish; publishes)
- `next-cell-subscribed-by-mission`: /coral_g/next_cell_goal -> mission_planner_node (Subscribe; next goal input)
- `next-cell-subscribed-by-collection-summary`: /coral_g/next_cell_goal -> collection_report_node (Subscribe; summary input)
- `mission-publishes-goal`: mission_planner_node -> /coral_g/goal_pose (Publish; publishes)
- `goal-subscribed-by-navigation`: /coral_g/goal_pose -> goal_navigation_node (Subscribe; subscribed by)
- `navigation-publishes-status`: goal_navigation_node -> /coral_g/navigation_status (Publish; publishes)
- `status-subscribed-by-mission`: /coral_g/navigation_status -> mission_planner_node (Subscribe; subscribed by)
- `status-subscribed-by-state`: /coral_g/navigation_status -> robot_state_node (Subscribe; subscribed by)
- `status-subscribed-by-collection-summary`: /coral_g/navigation_status -> collection_report_node (Subscribe; summary input)
- `navigation-calls-action`: goal_navigation_node -> NavigateToPose (Action client; action client)
- `action-served-by-nav2`: NavigateToPose -> Nav2 (Action server; action server)
- `nav2-publishes-cmd-vel`: Nav2 -> /cmd_vel (Publish; publishes)
- `cmd-vel-subscribed-by-robot`: /cmd_vel -> TurtleBot / Gazebo (Subscribe; subscribed by)
- `robot-publishes-scan`: TurtleBot / Gazebo -> /scan (Publish; publishes)
- `scan-subscribed-by-nav2`: /scan -> Nav2 (Subscribe; subscribed by)
- `scan-subscribed-by-twin-input`: /scan -> twin_input_node (Subscribe; subscribed by)
- `state-publishes-robot-state`: robot_state_node -> /coral_g/robot_state (Publish; publishes)
- `robot-state-subscribed-by-field-planner`: /coral_g/robot_state -> field_planner_node (Subscribe; resource state input)
- `robot-state-subscribed-by-mission`: /coral_g/robot_state -> mission_planner_node (Subscribe; subscribed by)
- `robot-state-subscribed-by-collection-summary`: /coral_g/robot_state -> collection_report_node (Subscribe; summary input)
- `collection-publishes-report`: collection_report_node -> /coral_g/collection_report (Publish; publishes)
- `collection-report-subscribed-by-material`: /coral_g/collection_report -> material_belief_node (Subscribe; updates belief)
- `collection-report-subscribed-by-twin`: /coral_g/collection_report -> digital_twin_state (Subscribe; updates twin)
- `collection-publishes-demo-report`: collection_report_node -> /report (Publish; publishes)
- `demo-report-subscribed-by-demo-control`: /report -> demo_control_surface (Subscribe; demo output)
- `demo-control-calls-reset`: demo_control_surface -> reset_reseed (Service client; service client)
- `reset-provided-by-debris`: reset_reseed -> debris_simulation_node (Service provider; service provider)
- `preset-ocean-conditions-owned-by-twin-input`: preset_ocean_conditions -> twin_input_node (Parameter owner; configures)
- `simulation-defaults-owned-by-debris`: debris_count=100 -> debris_simulation_node (Parameter owner; configures)
- `field-policy-owned-by-field-planner`: field_policy -> field_planner_node (Parameter owner; configures)
- `return-policy-owned-by-mission`: return_policy -> mission_planner_node (Parameter owner; configures)
- `return-policy-owned-by-state`: return_policy -> robot_state_node (Parameter owner; base reference)
- `goal-timeout-owned-by-navigation`: goal_timeout_sec=120 -> goal_navigation_node (Parameter owner; configures)
- `robot-state-defaults-owned-by-state`: state_defaults -> robot_state_node (Parameter owner; configures)
- `collection-defaults-owned-by-collection`: collection_defaults -> collection_report_node (Parameter owner; configures)

## Per-Node Context Packets

---

# twin_input_node

## Assignment

- Graph ID: `twin_input_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Simplified PoC input adapter that feeds preset ocean conditions and robot-observed map evidence into the digital twin.
- Workflow area: Twin inputs
- Workflow role: Feeds preset ocean conditions and robot-observed map evidence into the digital twin.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Owns: publishes `/coral_g/environment_field`, `/coral_g/map_sync_update`; subscribes to `/scan`; depends on parameters `preset_ocean_conditions`, `cell_size_m=1.0`, `tick_rate_hz=1.0`
- Does not own: digital twin fusion, debris simulation, utility-field planning, Nav2 execution, or collection reporting.

## Contract Surface

### Upstream Relationships

- `topic_scan` (/scan): Subscribe - subscribed by
- `param_preset_ocean_conditions` (preset_ocean_conditions): Parameter owner - configures

### Downstream Relationships

- `topic_environment_field` (/coral_g/environment_field): Publish - publishes
- `topic_map_sync_update` (/coral_g/map_sync_update): Publish - publishes

### Structured Contract Details

- Role: Simplified PoC input adapter that feeds preset ocean conditions and robot-observed map evidence into the digital twin.
- Publishes: `/coral_g/environment_field`, `/coral_g/map_sync_update`
- Subscribes: `/scan`
- Actions: none
- Services: none
- Parameters: `preset_ocean_conditions`, `cell_size_m=1.0`, `tick_rate_hz=1.0`

### Source-Backed Contract Notes

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Purpose: Provide the simplified PoC input boundary for the digital twin by combining preset ocean conditions with robot-observed map evidence.
- Source requirement: FR1, FR5, NFR1, and teacher feedback about showing where public/environment data enters the system.
- Inputs: `preset_ocean_conditions`, cleanup-zone bounds, `/scan`, and optional planned pose/map sources such as `/tf`, `/odom`, or `/map`.
- Outputs: `/coral_g/environment_field` and `/coral_g/map_sync_update` as `std_msgs/String` JSON.
- Parameters: `preset_ocean_conditions`, `zone_width_m`, `zone_height_m`, `cell_size_m`, `tick_rate_hz`; future API URL or map-confidence thresholds may be added later.
- Default behavior: Publish deterministic mock/static ocean-current, wind, and wave fields, plus a thin map-sync update from available scan evidence or a degraded/mock observation.
- Failure behavior: Publish deterministic fallback environment data and `status: "degraded"` or `status: "waiting_for_scan"` for map-sync evidence when robot observations are missing.
- Demo evidence: Logs show preset ocean conditions and robot-observed or mock map evidence both updating the digital twin state.

## Implementation Brief

Implement or review `twin_input_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Inputs to handle: `preset_ocean_conditions`, cleanup-zone bounds, `/scan`, and optional planned pose/map sources such as `/tf`, `/odom`, or `/map`.
- Outputs to produce or verify: `/coral_g/environment_field` and `/coral_g/map_sync_update` as `std_msgs/String` JSON.
- Default behavior: Publish deterministic mock/static ocean-current, wind, and wave fields, plus a thin map-sync update from available scan evidence or a degraded/mock observation.
- Failure/degraded behavior: Publish deterministic fallback environment data and `status: "degraded"` or `status: "waiting_for_scan"` for map-sync evidence when robot observations are missing.
- Demo evidence to produce: Logs show preset ocean conditions and robot-observed or mock map evidence both updating the digital twin state.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `twin_input_node` (`twin_input_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Simplified PoC input adapter that feeds preset ocean conditions and robot-observed map evidence into the digital twin. Neighbor relationships: twin_input_node -> topic_environment_field (publish: publishes); twin_input_node -> topic_map_sync_update (publish: publishes); topic_scan -> twin_input_node (subscribe: subscribed by); param_preset_ocean_conditions -> twin_input_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `twin_input_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `twin_input_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# material_belief_node

## Assignment

- Graph ID: `material_belief_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Maintains observed material counts and probabilities after collection reports; starts with unknown/no observations.
- Workflow area: Prediction and feedback
- Workflow role: Maintains observed material counts and probabilities after collection.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: publishes `/coral_g/material_distribution`; subscribes to `/coral_g/collection_report`
- Does not own: collection detection, density-map simulation, or navigation planning.

## Contract Surface

### Upstream Relationships

- `topic_collection_report` (/coral_g/collection_report): Subscribe - updates belief

### Downstream Relationships

- `topic_material_distribution` (/coral_g/material_distribution): Publish - publishes

### Structured Contract Details

- Role: Maintains observed material counts and probabilities after collection reports; starts with unknown/no observations.
- Publishes: `/coral_g/material_distribution`
- Subscribes: `/coral_g/collection_report`
- Actions: none
- Services: none
- Parameters: none

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Purpose: Maintain the observed material distribution used by the debris simulation and reporting loop.
- Source requirement: FR4 and FR12.
- Inputs: `/coral_g/collection_report`; no default material prior.
- Outputs: `/coral_g/material_distribution` as `std_msgs/String` JSON.
- Parameters: none for the simplified v1 contract.
- Default behavior: Start with empty counts and `status: "no_observations"`; probabilities are zero until a collection report arrives.
- Failure behavior: Map unrecognized material labels to `unknown` and publish a warning/status entry instead of discarding evidence silently.
- Demo evidence: Collected materials change the predicted distribution before the next simulation step.

## Implementation Brief

Implement or review `material_belief_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Inputs to handle: `/coral_g/collection_report`; no default material prior.
- Outputs to produce or verify: `/coral_g/material_distribution` as `std_msgs/String` JSON.
- Default behavior: Start with empty counts and `status: "no_observations"`; probabilities are zero until a collection report arrives.
- Failure/degraded behavior: Map unrecognized material labels to `unknown` and publish a warning/status entry instead of discarding evidence silently.
- Demo evidence to produce: Collected materials change the predicted distribution before the next simulation step.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `material_belief_node` (`material_belief_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Maintains observed material counts and probabilities after collection reports; starts with unknown/no observations. Neighbor relationships: material_belief_node -> topic_material_distribution (publish: publishes); topic_collection_report -> material_belief_node (subscribe: updates belief). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `material_belief_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `material_belief_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# debris_simulation_node

## Assignment

- Graph ID: `debris_simulation_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Predicts a debris density field from fused twin state, keeping debris reward out of blocked or non-water cells.
- Workflow area: Prediction
- Workflow role: Uses fused twin state to predict debris density only in valid water/free cells.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: publishes `/coral_g/debris_density_map`; subscribes to `/coral_g/twin_state`; uses services `reset_reseed`; depends on parameters `debris_count=100`, `simulation_horizon_sec=60.0`, `random_seed=23`
- Does not own: map synchronization, material posterior updates, utility-field policy, or robot motion.

## Contract Surface

### Upstream Relationships

- `topic_twin_state` (/coral_g/twin_state): Subscribe - constrains prediction
- `service_reset_reseed` (reset_reseed): Service provider - service provider
- `param_simulation_defaults` (debris_count=100): Parameter owner - configures

### Downstream Relationships

- `topic_debris_density_map` (/coral_g/debris_density_map): Publish - publishes

### Structured Contract Details

- Role: Predicts a debris density field from fused twin state, keeping debris reward out of blocked or non-water cells.
- Publishes: `/coral_g/debris_density_map`
- Subscribes: `/coral_g/twin_state`
- Actions: none
- Services: `reset_reseed`
- Parameters: `debris_count=100`, `simulation_horizon_sec=60.0`, `random_seed=23`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Purpose: Predict virtual debris movement through the fused twin field while keeping virtual debris out of blocked or non-water cells.
- Source requirement: FR1, FR2, FR3, FR4.
- Inputs: `/coral_g/twin_state`, simulation parameters.
- Outputs: `/coral_g/debris_density_map` as `std_msgs/String` JSON.
- Parameters: `debris_count`, `simulation_horizon_sec`, `tick_rate_hz`, `random_seed`, `density_cell_size_m`.
- Default behavior: Seed virtual debris deterministically and publish density only for valid water/free cells from the twin map.
- Failure behavior: Publish a degraded or waiting status until usable twin state is available.
- Demo evidence: Visible density and accumulation zones change when synchronized obstacles or blocked cells change the twin map.

## Implementation Brief

Implement or review `debris_simulation_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Inputs to handle: `/coral_g/twin_state`, simulation parameters.
- Outputs to produce or verify: `/coral_g/debris_density_map` as `std_msgs/String` JSON.
- Default behavior: Seed virtual debris deterministically and publish density only for valid water/free cells from the twin map.
- Failure/degraded behavior: Publish a degraded or waiting status until usable twin state is available.
- Demo evidence to produce: Visible density and accumulation zones change when synchronized obstacles or blocked cells change the twin map.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `debris_simulation_node` (`debris_simulation_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Predicts a debris density field from fused twin state, keeping debris reward out of blocked or non-water cells. Neighbor relationships: topic_twin_state -> debris_simulation_node (subscribe: constrains prediction); debris_simulation_node -> topic_debris_density_map (publish: publishes); service_reset_reseed -> debris_simulation_node (service-provider: service provider); param_simulation_defaults -> debris_simulation_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `debris_simulation_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `debris_simulation_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# field_planner_node

## Assignment

- Graph ID: `field_planner_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Scores each map grid cell with a utility value from debris density, robot telemetry, twin map confidence, and return constraints, then emits one next-cell goal.
- Workflow area: Decision
- Workflow role: Builds a utility field and emits one next grid-cell or base-return goal.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: publishes `/coral_g/next_cell_goal`; subscribes to `/coral_g/debris_density_map`, `/coral_g/robot_state`, `/coral_g/twin_state`; depends on parameters `field_policy`
- Does not own: map-frame goal publication, Nav2 execution, collection reporting, or raw density simulation.

## Contract Surface

### Upstream Relationships

- `topic_twin_state` (/coral_g/twin_state): Subscribe - utility constraints
- `topic_debris_density_map` (/coral_g/debris_density_map): Subscribe - reward field input
- `topic_robot_state` (/coral_g/robot_state): Subscribe - resource state input
- `param_field_policy` (field_policy): Parameter owner - configures

### Downstream Relationships

- `topic_next_cell_goal` (/coral_g/next_cell_goal): Publish - publishes

### Structured Contract Details

- Role: Scores each map grid cell with a utility value from debris density, robot telemetry, twin map confidence, and return constraints, then emits one next-cell goal.
- Publishes: `/coral_g/next_cell_goal`
- Subscribes: `/coral_g/debris_density_map`, `/coral_g/robot_state`, `/coral_g/twin_state`
- Actions: none
- Services: none
- Parameters: `field_policy`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Purpose: Convert the debris density map into a utility field and choose one next grid-cell or return-to-base goal.
- Source requirement: FR6, FR7, FR8, FR11, and NFR2, simplified from candidate-list planning into field-based next-cell planning.
- Inputs: `/coral_g/debris_density_map`, `/coral_g/robot_state`, `/coral_g/twin_state`, and `return_policy`.
- Outputs: `/coral_g/next_cell_goal` as `std_msgs/String` JSON.
- Parameters: `field_policy` reward/penalty weights plus `return_policy` base pose and reserve thresholds.
- Default behavior: Score each valid grid cell as `utility = density_reward - travel_cost - storage_penalty - fuel_penalty - map_risk`, reject cells that cannot still return to base with reserve, and publish the best reachable next cell. Utility means the final planner score; higher is better.
- Failure behavior: Publish `status: "idle"` when no cleanup cell is eligible, or `mode: "return_to_base"` when fuel/storage policy requires base return.
- Demo evidence: The next-cell goal shows density reward, travel cost, return cost, fuel margin, storage state, map confidence, and the reason the cell or base was selected.

## Implementation Brief

Implement or review `field_planner_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Inputs to handle: `/coral_g/debris_density_map`, `/coral_g/robot_state`, `/coral_g/twin_state`, and `return_policy`.
- Outputs to produce or verify: `/coral_g/next_cell_goal` as `std_msgs/String` JSON.
- Default behavior: Score each valid grid cell as `utility = density_reward - travel_cost - storage_penalty - fuel_penalty - map_risk`, reject cells that cannot still return to base with reserve, and publish the best reachable next cell. Utility means the final planner score; higher is better.
- Failure/degraded behavior: Publish `status: "idle"` when no cleanup cell is eligible, or `mode: "return_to_base"` when fuel/storage policy requires base return.
- Demo evidence to produce: The next-cell goal shows density reward, travel cost, return cost, fuel margin, storage state, map confidence, and the reason the cell or base was selected.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `field_planner_node` (`field_planner_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Scores each map grid cell with a utility value from debris density, robot telemetry, twin map confidence, and return constraints, then emits one next-cell goal. Neighbor relationships: topic_twin_state -> field_planner_node (subscribe: utility constraints); topic_debris_density_map -> field_planner_node (subscribe: reward field input); field_planner_node -> topic_next_cell_goal (publish: publishes); topic_robot_state -> field_planner_node (subscribe: resource state input); param_field_policy -> field_planner_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `field_planner_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `field_planner_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# mission_planner_node

## Assignment

- Graph ID: `mission_planner_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Converts field-planner next-cell or return-to-base decisions into map-frame robot goals that Nav2 can execute.
- Workflow area: Robot execution
- Workflow role: Converts next-cell or return-to-base state into map-frame goals.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: publishes `/coral_g/goal_pose`; subscribes to `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`; depends on parameters `return_policy`
- Does not own: utility-field scoring, Nav2 action execution, or low-level `/cmd_vel` commands.

## Contract Surface

### Upstream Relationships

- `topic_next_cell_goal` (/coral_g/next_cell_goal): Subscribe - next goal input
- `topic_navigation_status` (/coral_g/navigation_status): Subscribe - subscribed by
- `topic_robot_state` (/coral_g/robot_state): Subscribe - subscribed by
- `param_return_policy` (return_policy): Parameter owner - configures

### Downstream Relationships

- `topic_goal_pose` (/coral_g/goal_pose): Publish - publishes

### Structured Contract Details

- Role: Converts field-planner next-cell or return-to-base decisions into map-frame robot goals that Nav2 can execute.
- Publishes: `/coral_g/goal_pose`
- Subscribes: `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`
- Actions: none
- Services: none
- Parameters: `return_policy`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Purpose: Convert field-planner decisions into map-frame robot goals.
- Source requirement: FR7, FR8, FR10, FR11.
- Inputs: `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`.
- Outputs: `/coral_g/goal_pose` as `geometry_msgs/PoseStamped`.
- Parameters: `return_policy` for base pose and return thresholds, plus `goal_frame_id`.
- Default behavior: Send next-cell goals while storage and fuel are acceptable; send base goal when the field planner or return policy requires return.
- Failure behavior: Do not publish a goal if the next-cell payload is missing, idle, malformed, or outside the map frame.
- Demo evidence: The robot receives a valid map-frame goal from the next-cell or return-to-base decision.

## Implementation Brief

Implement or review `mission_planner_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Inputs to handle: `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`.
- Outputs to produce or verify: `/coral_g/goal_pose` as `geometry_msgs/PoseStamped`.
- Default behavior: Send next-cell goals while storage and fuel are acceptable; send base goal when the field planner or return policy requires return.
- Failure/degraded behavior: Do not publish a goal if the next-cell payload is missing, idle, malformed, or outside the map frame.
- Demo evidence to produce: The robot receives a valid map-frame goal from the next-cell or return-to-base decision.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `mission_planner_node` (`mission_planner_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Converts field-planner next-cell or return-to-base decisions into map-frame robot goals that Nav2 can execute. Neighbor relationships: topic_next_cell_goal -> mission_planner_node (subscribe: next goal input); mission_planner_node -> topic_goal_pose (publish: publishes); topic_navigation_status -> mission_planner_node (subscribe: subscribed by); topic_robot_state -> mission_planner_node (subscribe: subscribed by); param_return_policy -> mission_planner_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `mission_planner_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `mission_planner_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# goal_navigation_node

## Assignment

- Graph ID: `goal_navigation_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Existing bridge from CORAL-G map-frame goals into Nav2 execution; Nav2 calculates and follows the actual path.
- Workflow area: Robot execution
- Workflow role: Bridges CORAL-G goals into Nav2 NavigateToPose execution.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Owns: publishes `/coral_g/navigation_status`; subscribes to `/coral_g/goal_pose`; uses actions `NavigateToPose client`; depends on parameters `goal_timeout_sec=120.0`
- Does not own: choosing mission targets or publishing `/cmd_vel` directly.

## Contract Surface

### Upstream Relationships

- `topic_goal_pose` (/coral_g/goal_pose): Subscribe - subscribed by
- `param_goal_timeout` (goal_timeout_sec=120): Parameter owner - configures

### Downstream Relationships

- `topic_navigation_status` (/coral_g/navigation_status): Publish - publishes
- `action_navigate_to_pose` (NavigateToPose): Action client - action client

### Structured Contract Details

- Role: Existing bridge from CORAL-G map-frame goals into Nav2 execution; Nav2 calculates and follows the actual path.
- Publishes: `/coral_g/navigation_status`
- Subscribes: `/coral_g/goal_pose`
- Actions: `NavigateToPose client`
- Services: none
- Parameters: `goal_timeout_sec=120.0`

### Source-Backed Contract Notes

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Purpose: Existing bridge between CORAL-G goals and Nav2 execution.
- Source requirement: FR10.
- Inputs: `/coral_g/goal_pose` as `geometry_msgs/PoseStamped`.
- Outputs: `/coral_g/navigation_status` as `std_msgs/String`; Nav2 actions through `BasicNavigator`.
- Parameters: `goal_timeout_sec`, `initial_x`, `initial_y`, `initial_yaw`.
- Default behavior: Incoming goals must use `header.frame_id: "map"`; valid goals are sent to Nav2 through `BasicNavigator`.
- Failure behavior: Reject non-map goals, cancel timed-out goals, publish failed/timeout/canceled status.
- Demo evidence: Status sequence `ready -> received -> active -> succeeded` for a reachable goal.

## Implementation Brief

Implement or review `goal_navigation_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Inputs to handle: `/coral_g/goal_pose` as `geometry_msgs/PoseStamped`.
- Outputs to produce or verify: `/coral_g/navigation_status` as `std_msgs/String`; Nav2 actions through `BasicNavigator`.
- Default behavior: Incoming goals must use `header.frame_id: "map"`; valid goals are sent to Nav2 through `BasicNavigator`.
- Failure/degraded behavior: Reject non-map goals, cancel timed-out goals, publish failed/timeout/canceled status.
- Demo evidence to produce: Status sequence `ready -> received -> active -> succeeded` for a reachable goal.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `goal_navigation_node` (`goal_navigation_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Existing bridge from CORAL-G map-frame goals into Nav2 execution; Nav2 calculates and follows the actual path. Neighbor relationships: topic_goal_pose -> goal_navigation_node (subscribe: subscribed by); goal_navigation_node -> topic_navigation_status (publish: publishes); goal_navigation_node -> action_navigate_to_pose (action-client: action client); param_goal_timeout -> goal_navigation_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `goal_navigation_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `goal_navigation_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# Nav2

## Assignment

- Graph ID: `nav2`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Navigation stack that plans and executes robot movement from map-frame goals.
- Workflow area: Robot execution
- Workflow role: Plans and executes robot movement and remains the intended `/cmd_vel` owner during navigation demos.
- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Owns: publishes `/cmd_vel`; subscribes to `/scan`; uses actions `NavigateToPose server`
- Does not own: CORAL-G decision logic, material belief, or demo reporting.

## Contract Surface

### Upstream Relationships

- `action_navigate_to_pose` (NavigateToPose): Action server - action server
- `topic_scan` (/scan): Subscribe - subscribed by

### Downstream Relationships

- `topic_cmd_vel` (/cmd_vel): Publish - publishes

### Structured Contract Details

- Role: Navigation stack that plans and executes robot movement from map-frame goals.
- Publishes: `/cmd_vel`
- Subscribes: `/scan`
- Actions: `NavigateToPose server`
- Services: none
- Parameters: none

### Source-Backed Contract Notes

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- No separate detailed contract table is recorded for this node; use `graph-data.ts`, neighboring contract notes, and the v1 ROS boundaries as source truth.

## Implementation Brief

Review this node against its graph relationships and neighboring source-backed contracts.

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- If a behavior is not specified in the source docs, record it as a gap instead of inventing a new contract.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `nav2` (`Nav2`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Navigation stack that plans and executes robot movement from map-frame goals. Neighbor relationships: action_navigate_to_pose -> nav2 (action-server: action server); nav2 -> topic_cmd_vel (publish: publishes); topic_scan -> nav2 (subscribe: subscribed by). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `nav2`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `nav2` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# TurtleBot / Gazebo

## Assignment

- Graph ID: `turtlebot_gazebo`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Physical or simulated robot that executes velocity commands and exposes LiDAR data.
- Workflow area: Robot execution
- Workflow role: Represents the physical or simulated robot that executes velocity commands and exposes LiDAR.
- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Owns: publishes `/scan`; subscribes to `/cmd_vel`
- Does not own: digital twin predictions, utility-field planning, or mission planning.

## Contract Surface

### Upstream Relationships

- `topic_cmd_vel` (/cmd_vel): Subscribe - subscribed by

### Downstream Relationships

- `topic_scan` (/scan): Publish - publishes

### Structured Contract Details

- Role: Physical or simulated robot that executes velocity commands and exposes LiDAR data.
- Publishes: `/scan`
- Subscribes: `/cmd_vel`
- Actions: none
- Services: none
- Parameters: none

### Source-Backed Contract Notes

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- No separate detailed contract table is recorded for this node; use `graph-data.ts`, neighboring contract notes, and the v1 ROS boundaries as source truth.

## Implementation Brief

Review this node against its graph relationships and neighboring source-backed contracts.

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- If a behavior is not specified in the source docs, record it as a gap instead of inventing a new contract.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `turtlebot_gazebo` (`TurtleBot / Gazebo`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Physical or simulated robot that executes velocity commands and exposes LiDAR data. Neighbor relationships: topic_cmd_vel -> turtlebot_gazebo (subscribe: subscribed by); turtlebot_gazebo -> topic_scan (publish: publishes). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `turtlebot_gazebo`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `turtlebot_gazebo` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# digital_twin_state

## Assignment

- Graph ID: `digital_twin_state`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Fused world-state node for environment, observed map, material belief, and collection feedback; it does not copy live robot telemetry or prediction output.
- Workflow area: Twin synchronization
- Workflow role: Fused world-state publisher for environment, observed map, material belief, and collection feedback.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: publishes `/coral_g/twin_state`; subscribes to `/coral_g/environment_field`, `/coral_g/map_sync_update`, `/coral_g/material_distribution`, `/coral_g/collection_report`
- Does not own: raw LiDAR processing, live robot telemetry, prediction density output, Nav2 execution, external API ownership, or robot control.

## Contract Surface

### Upstream Relationships

- `topic_environment_field` (/coral_g/environment_field): Subscribe - updates twin
- `topic_material_distribution` (/coral_g/material_distribution): Subscribe - updates twin
- `topic_map_sync_update` (/coral_g/map_sync_update): Subscribe - updates twin map
- `topic_collection_report` (/coral_g/collection_report): Subscribe - updates twin

### Downstream Relationships

- `topic_twin_state` (/coral_g/twin_state): Publish - publishes

### Structured Contract Details

- Role: Fused world-state node for environment, observed map, material belief, and collection feedback; it does not copy live robot telemetry or prediction output.
- Publishes: `/coral_g/twin_state`
- Subscribes: `/coral_g/environment_field`, `/coral_g/map_sync_update`, `/coral_g/material_distribution`, `/coral_g/collection_report`
- Actions: none
- Services: none
- Parameters: none

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Purpose: Fuse external environment, synchronized map evidence, material belief, and collection feedback into the current world-state belief.
- Source requirement: NFR1, FR5, FR8, FR9, FR12.
- Inputs: `/coral_g/environment_field`, `/coral_g/map_sync_update`, `/coral_g/material_distribution`, `/coral_g/collection_report`.
- Outputs: `/coral_g/twin_state` as `std_msgs/String` JSON.
- Parameters: none required for v1 planning.
- Default behavior: Publish fused world state with water, blocked, and unknown cells plus material belief and freshness/confidence status.
- Failure behavior: Publish `sync_status: "partial"` until all major inputs are fresh enough for a full mission loop.
- Demo evidence: Twin state shows robot-observed map and material/collection changes constraining debris prediction and utility-field planning.

## Implementation Brief

Implement or review `digital_twin_state` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Inputs to handle: `/coral_g/environment_field`, `/coral_g/map_sync_update`, `/coral_g/material_distribution`, `/coral_g/collection_report`.
- Outputs to produce or verify: `/coral_g/twin_state` as `std_msgs/String` JSON.
- Default behavior: Publish fused world state with water, blocked, and unknown cells plus material belief and freshness/confidence status.
- Failure/degraded behavior: Publish `sync_status: "partial"` until all major inputs are fresh enough for a full mission loop.
- Demo evidence to produce: Twin state shows robot-observed map and material/collection changes constraining debris prediction and utility-field planning.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `digital_twin_state` (`digital_twin_state`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Fused world-state node for environment, observed map, material belief, and collection feedback; it does not copy live robot telemetry or prediction output. Neighbor relationships: topic_environment_field -> digital_twin_state (subscribe: updates twin); topic_material_distribution -> digital_twin_state (subscribe: updates twin); topic_map_sync_update -> digital_twin_state (subscribe: updates twin map); digital_twin_state -> topic_twin_state (publish: publishes); topic_collection_report -> digital_twin_state (subscribe: updates twin). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `digital_twin_state`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `digital_twin_state` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# robot_state_node

## Assignment

- Graph ID: `robot_state_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Publishes live robot telemetry: position/heading, storage, fuel, base-state, and navigation status.
- Workflow area: Feedback and planning
- Workflow role: Publishes pose, storage, fuel, and base-state information for utility-field planning.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Owns: publishes `/coral_g/robot_state`; subscribes to `/coral_g/navigation_status`; depends on parameters `storage_capacity_items`, `fuel_capacity_norm`, `state_tick_rate_hz`
- Does not own: utility scoring, goal publication, or collection material classification.

## Contract Surface

### Upstream Relationships

- `topic_navigation_status` (/coral_g/navigation_status): Subscribe - subscribed by
- `param_return_policy` (return_policy): Parameter owner - base reference
- `param_robot_state_defaults` (state_defaults): Parameter owner - configures

### Downstream Relationships

- `topic_robot_state` (/coral_g/robot_state): Publish - publishes

### Structured Contract Details

- Role: Publishes live robot telemetry: position/heading, storage, fuel, base-state, and navigation status.
- Publishes: `/coral_g/robot_state`
- Subscribes: `/coral_g/navigation_status`
- Actions: none
- Services: none
- Parameters: `storage_capacity_items`, `fuel_capacity_norm`, `state_tick_rate_hz`

### Source-Backed Contract Notes

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Purpose: Publish live robot telemetry needed by the utility-field planner, mission planner, and demo report.
- Source requirement: NFR1, FR9, FR11.
- Inputs: Robot pose source (position plus orientation), navigation status, optional collection/base events.
- Outputs: `/coral_g/robot_state` as `std_msgs/String` JSON.
- Parameters: `storage_capacity_items`, `fuel_capacity_norm`, `state_tick_rate_hz`; base pose is read from `return_policy`.
- Default behavior: Track pose, storage fill, fuel level, and whether the robot is at base.
- Refuel behavior: When the robot returns to base and offloads, reset storage to `0.0` and fuel to `1.0`.
- Demo evidence: State log shows fuel/storage decreasing during a mission and resetting at base.

## Implementation Brief

Implement or review `robot_state_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Inputs to handle: Robot pose source (position plus orientation), navigation status, optional collection/base events.
- Outputs to produce or verify: `/coral_g/robot_state` as `std_msgs/String` JSON.
- Default behavior: Track pose, storage fill, fuel level, and whether the robot is at base.
- Special behavior: When the robot returns to base and offloads, reset storage to `0.0` and fuel to `1.0`.
- Demo evidence to produce: State log shows fuel/storage decreasing during a mission and resetting at base.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `robot_state_node` (`robot_state_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Publishes live robot telemetry: position/heading, storage, fuel, base-state, and navigation status. Neighbor relationships: topic_navigation_status -> robot_state_node (subscribe: subscribed by); robot_state_node -> topic_robot_state (publish: publishes); param_return_policy -> robot_state_node (parameter-owner: base reference); param_robot_state_defaults -> robot_state_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `robot_state_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `robot_state_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# collection_report_node

## Assignment

- Graph ID: `collection_report_node`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Reports raw collection observations and publishes the requested demo mission summary.
- Workflow area: Feedback loop and reporting
- Workflow role: Publishes collection feedback and the requested human-readable demo run summary.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: publishes `/coral_g/collection_report`, `/report`; subscribes to `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`, `/coral_g/material_distribution`, `/coral_g/debris_density_map`; depends on parameters `allowed_materials`, `default_material`, `collection_radius_m`
- Does not own: navigation execution, density simulation ownership, or material posterior computation.

## Contract Surface

### Upstream Relationships

- `topic_material_distribution` (/coral_g/material_distribution): Subscribe - summary input
- `topic_debris_density_map` (/coral_g/debris_density_map): Subscribe - heatmap input
- `topic_next_cell_goal` (/coral_g/next_cell_goal): Subscribe - summary input
- `topic_navigation_status` (/coral_g/navigation_status): Subscribe - summary input
- `topic_robot_state` (/coral_g/robot_state): Subscribe - summary input
- `param_collection_defaults` (collection_defaults): Parameter owner - configures

### Downstream Relationships

- `topic_collection_report` (/coral_g/collection_report): Publish - publishes
- `topic_report` (/report): Publish - publishes

### Structured Contract Details

- Role: Reports raw collection observations and publishes the requested demo mission summary.
- Publishes: `/coral_g/collection_report`, `/report`
- Subscribes: `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`, `/coral_g/material_distribution`, `/coral_g/debris_density_map`
- Actions: none
- Services: none
- Parameters: `allowed_materials`, `default_material`, `collection_radius_m`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Purpose: Report collected trash observations back into the digital twin and publish the requested demo mission summary.
- Source requirement: FR9 and FR12.
- Inputs: Manual/demo collection trigger or future physical collection detector; `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`, `/coral_g/material_distribution`, and `/coral_g/debris_density_map` for demo summary output.
- Outputs: `/coral_g/collection_report` and `/report` as `std_msgs/String` JSON.
- Parameters: `allowed_materials`, `default_material`, `collection_radius_m`.
- Default behavior: Publish material, collection location, count, and timestamp; when the demo report is requested, publish a mission summary with the latest next-cell goal, collection stats, distribution shift, robot state, field policy, and JSON heatmap cells.
- Failure behavior: Reject material labels outside the allowed set unless explicitly mapped to `unknown`.
- Demo evidence: Collection report updates material belief, reduces predicted density near the collection location, and `/report` gives the human reviewer one run summary.

## Implementation Brief

Implement or review `collection_report_node` so it satisfies its source-backed purpose without changing adjacent contracts.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Inputs to handle: Manual/demo collection trigger or future physical collection detector; `/coral_g/next_cell_goal`, `/coral_g/robot_state`, `/coral_g/navigation_status`, `/coral_g/material_distribution`, and `/coral_g/debris_density_map` for demo summary output.
- Outputs to produce or verify: `/coral_g/collection_report` and `/report` as `std_msgs/String` JSON.
- Default behavior: Publish material, collection location, count, and timestamp; when the demo report is requested, publish a mission summary with the latest next-cell goal, collection stats, distribution shift, robot state, field policy, and JSON heatmap cells.
- Failure/degraded behavior: Reject material labels outside the allowed set unless explicitly mapped to `unknown`.
- Demo evidence to produce: Collection report updates material belief, reduces predicted density near the collection location, and `/report` gives the human reviewer one run summary.
- Verify against upstream/downstream relationships before marking the node complete.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `collection_report_node` (`collection_report_node`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Reports raw collection observations and publishes the requested demo mission summary. Neighbor relationships: topic_material_distribution -> collection_report_node (subscribe: summary input); topic_debris_density_map -> collection_report_node (subscribe: heatmap input); topic_next_cell_goal -> collection_report_node (subscribe: summary input); topic_navigation_status -> collection_report_node (subscribe: summary input); topic_robot_state -> collection_report_node (subscribe: summary input); collection_report_node -> topic_collection_report (publish: publishes); collection_report_node -> topic_report (publish: publishes); param_collection_defaults -> collection_report_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `collection_report_node`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `collection_report_node` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# demo_control_surface

## Assignment

- Graph ID: `demo_control_surface`
- Kind: `component`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Assign implementation or review ownership and validate connected contracts.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Manual demo or developer control surface for optional reset/reseed calls.
- Workflow area: Demo control and review
- Workflow role: Reviews report output and may trigger optional reset/reseed behavior.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: subscribes to `/report`; uses services `reset_reseed client`
- Does not own: autonomous decision logic or hidden writes to graph/context contracts.

## Contract Surface

### Upstream Relationships

- `topic_report` (/report): Subscribe - demo output

### Downstream Relationships

- `service_reset_reseed` (reset_reseed): Service client - service client

### Structured Contract Details

- Role: Manual demo or developer control surface for optional reset/reseed calls.
- Publishes: none
- Subscribes: `/report`
- Actions: none
- Services: `reset_reseed client`
- Parameters: none

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- No separate detailed contract table is recorded for this node; use `graph-data.ts`, neighboring contract notes, and the v1 ROS boundaries as source truth.

## Implementation Brief

Review this node against its graph relationships and neighboring source-backed contracts.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- If a behavior is not specified in the source docs, record it as a gap instead of inventing a new contract.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `demo_control_surface` (`demo_control_surface`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Manual demo or developer control surface for optional reset/reseed calls. Neighbor relationships: topic_report -> demo_control_surface (subscribe: demo output); demo_control_surface -> service_reset_reseed (service-client: service client). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `demo_control_surface`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `demo_control_surface` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Owned publishers, subscribers, actions, services, and parameters are checked against implementation or lab plan.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/environment_field

## Assignment

- Graph ID: `topic_environment_field`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/environment_field carries std_msgs/String JSON using coral_g.environment_field.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.environment_field.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `twin_input_node` (twin_input_node): Publish - publishes

### Downstream Relationships

- `digital_twin_state` (digital_twin_state): Subscribe - updates twin

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.environment_field.v1`
- Contains: External/static environment forces: mode, frame_id, cleanup zone, cell_size_m, and grid cells with x/y/current/wind/wave fields. Robot-observed obstacles do not belong here.
- Publishers: `twin_input_node`
- Subscribers: `digital_twin_state`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Required fields: `stamp`, `source`, `mode`, `cell_size_m`, `cells[]`; external/static forces only, not robot-observed obstacle updates.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_environment_field` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.environment_field.v1`
- Contains: External/static environment forces: mode, frame_id, cleanup zone, cell_size_m, and grid cells with x/y/current/wind/wave fields. Robot-observed obstacles do not belong here.
- Required fields: `stamp`, `source`, `mode`, `cell_size_m`, `cells[]`; external/static forces only, not robot-observed obstacle updates.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_environment_field` (`/coral_g/environment_field`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/environment_field carries std_msgs/String JSON using coral_g.environment_field.v1. Neighbor relationships: twin_input_node -> topic_environment_field (publish: publishes); topic_environment_field -> digital_twin_state (subscribe: updates twin). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_environment_field`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_environment_field` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/material_distribution

## Assignment

- Graph ID: `topic_material_distribution`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/material_distribution carries std_msgs/String JSON using coral_g.material_distribution.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.material_distribution.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `material_belief_node` (material_belief_node): Publish - publishes

### Downstream Relationships

- `digital_twin_state` (digital_twin_state): Subscribe - updates twin
- `collection_report_node` (collection_report_node): Subscribe - summary input

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.material_distribution.v1`
- Contains: Observed material belief: counts and normalized probabilities for plastic, foam, rubber, metal, and unknown plus status/warnings. Starts as no_observations.
- Publishers: `material_belief_node`
- Subscribers: `digital_twin_state`, `collection_report_node`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Required fields: `stamp`, `source`, `counts`, `probabilities`, `status`; `counts` and `probabilities` include `plastic`, `foam`, `rubber`, `metal`, and `unknown`.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_material_distribution` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.material_distribution.v1`
- Contains: Observed material belief: counts and normalized probabilities for plastic, foam, rubber, metal, and unknown plus status/warnings. Starts as no_observations.
- Required fields: `stamp`, `source`, `counts`, `probabilities`, `status`; `counts` and `probabilities` include `plastic`, `foam`, `rubber`, `metal`, and `unknown`.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_material_distribution` (`/coral_g/material_distribution`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/material_distribution carries std_msgs/String JSON using coral_g.material_distribution.v1. Neighbor relationships: material_belief_node -> topic_material_distribution (publish: publishes); topic_material_distribution -> digital_twin_state (subscribe: updates twin); topic_material_distribution -> collection_report_node (subscribe: summary input). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_material_distribution`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_material_distribution` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/map_sync_update

## Assignment

- Graph ID: `topic_map_sync_update`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/map_sync_update carries std_msgs/String JSON using coral_g.map_sync_update.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.map_sync_update.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `twin_input_node` (twin_input_node): Publish - publishes

### Downstream Relationships

- `digital_twin_state` (digital_twin_state): Subscribe - updates twin map

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.map_sync_update.v1`
- Contains: Robot-observed map update: frame_id, observer pose (position + heading), observed_obstacles, free_space_cells, map_confidence, and status.
- Publishers: `twin_input_node`
- Subscribers: `digital_twin_state`

### Source-Backed Contract Notes

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Required fields: `schema`, `stamp`, `source`, `frame_id`, `robot_pose`, `observed_obstacles`, `free_space_cells`, `map_confidence`, `status`; `robot_pose` means the observer position plus heading used to place map evidence.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_map_sync_update` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.map_sync_update.v1`
- Contains: Robot-observed map update: frame_id, observer pose (position + heading), observed_obstacles, free_space_cells, map_confidence, and status.
- Required fields: `schema`, `stamp`, `source`, `frame_id`, `robot_pose`, `observed_obstacles`, `free_space_cells`, `map_confidence`, `status`; `robot_pose` means the observer position plus heading used to place map evidence.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_map_sync_update` (`/coral_g/map_sync_update`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/map_sync_update carries std_msgs/String JSON using coral_g.map_sync_update.v1. Neighbor relationships: twin_input_node -> topic_map_sync_update (publish: publishes); topic_map_sync_update -> digital_twin_state (subscribe: updates twin map). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_map_sync_update`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_map_sync_update` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/twin_state

## Assignment

- Graph ID: `topic_twin_state`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/twin_state carries std_msgs/String JSON using coral_g.twin_state.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.twin_state.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `digital_twin_state` (digital_twin_state): Publish - publishes

### Downstream Relationships

- `debris_simulation_node` (debris_simulation_node): Subscribe - constrains prediction
- `field_planner_node` (field_planner_node): Subscribe - utility constraints

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.twin_state.v1`
- Contains: Fused world belief: environment_status, map water/blocked/unknown cells, material_belief, collection_updates, and sync_status. Live robot telemetry and prediction density stay in their own topics.
- Publishers: `digital_twin_state`
- Subscribers: `debris_simulation_node`, `field_planner_node`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Required fields: `schema`, `stamp`, `source`, `environment_status`, `map`, `material_belief`, `collection_updates`, `sync_status`; `map` includes `water_cells`, `blocked_cells`, `unknown_cells`, `known_area_ratio`, and `obstacle_count`. Live robot telemetry and prediction density stay in `/coral_g/robot_state` and `/coral_g/debris_density_map`.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_twin_state` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.twin_state.v1`
- Contains: Fused world belief: environment_status, map water/blocked/unknown cells, material_belief, collection_updates, and sync_status. Live robot telemetry and prediction density stay in their own topics.
- Required fields: `schema`, `stamp`, `source`, `environment_status`, `map`, `material_belief`, `collection_updates`, `sync_status`; `map` includes `water_cells`, `blocked_cells`, `unknown_cells`, `known_area_ratio`, and `obstacle_count`. Live robot telemetry and prediction density stay in `/coral_g/robot_state` and `/coral_g/debris_density_map`.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_twin_state` (`/coral_g/twin_state`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/twin_state carries std_msgs/String JSON using coral_g.twin_state.v1. Neighbor relationships: digital_twin_state -> topic_twin_state (publish: publishes); topic_twin_state -> debris_simulation_node (subscribe: constrains prediction); topic_twin_state -> field_planner_node (subscribe: utility constraints). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_twin_state`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_twin_state` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/debris_density_map

## Assignment

- Graph ID: `topic_debris_density_map`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/debris_density_map carries std_msgs/String JSON using coral_g.debris_density_map.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.debris_density_map.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `debris_simulation_node` (debris_simulation_node): Publish - publishes

### Downstream Relationships

- `field_planner_node` (field_planner_node): Subscribe - reward field input
- `collection_report_node` (collection_report_node): Subscribe - heatmap input

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.debris_density_map.v1`
- Contains: Prediction output: frame_id, cell_size_m, simulation_horizon_sec, status, and density_cells with x/y/density/material_hint only for valid water/free cells. Density is a planning reward signal, not guaranteed physical debris.
- Publishers: `debris_simulation_node`
- Subscribers: `field_planner_node`, `collection_report_node`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Required fields: `stamp`, `source`, `frame_id`, `cell_size_m`, `density_cells[]`, `status`; density cells contain prediction reward data such as `x`, `y`, `density`, and optional `material_hint`, but do not repeat twin map or robot state. Density is a planning signal, not guaranteed physical debris.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_debris_density_map` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.debris_density_map.v1`
- Contains: Prediction output: frame_id, cell_size_m, simulation_horizon_sec, status, and density_cells with x/y/density/material_hint only for valid water/free cells. Density is a planning reward signal, not guaranteed physical debris.
- Required fields: `stamp`, `source`, `frame_id`, `cell_size_m`, `density_cells[]`, `status`; density cells contain prediction reward data such as `x`, `y`, `density`, and optional `material_hint`, but do not repeat twin map or robot state. Density is a planning signal, not guaranteed physical debris.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_debris_density_map` (`/coral_g/debris_density_map`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/debris_density_map carries std_msgs/String JSON using coral_g.debris_density_map.v1. Neighbor relationships: debris_simulation_node -> topic_debris_density_map (publish: publishes); topic_debris_density_map -> field_planner_node (subscribe: reward field input); topic_debris_density_map -> collection_report_node (subscribe: heatmap input). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_debris_density_map`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_debris_density_map` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/next_cell_goal

## Assignment

- Graph ID: `topic_next_cell_goal`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/next_cell_goal carries std_msgs/String JSON using coral_g.next_cell_goal.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.next_cell_goal.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `field_planner_node` (field_planner_node): Publish - publishes

### Downstream Relationships

- `mission_planner_node` (mission_planner_node): Subscribe - next goal input
- `collection_report_node` (collection_report_node): Subscribe - summary input

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.next_cell_goal.v1`
- Contains: Single field-planner decision: status, mode, map-frame cell pose (position + heading), utility score, density reward, travel cost, return cost, fuel margin, storage state, and reason.
- Publishers: `field_planner_node`
- Subscribers: `mission_planner_node`, `collection_report_node`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Required fields: `schema`, `stamp`, `source`, `status`, `mode`, `goal`, `utility`, `components`, `return_feasible`, and `reason`; `goal` contains map-frame `x`, `y`, and `yaw` for the next grid cell or base. Here `yaw` means heading direction.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_next_cell_goal` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.next_cell_goal.v1`
- Contains: Single field-planner decision: status, mode, map-frame cell pose (position + heading), utility score, density reward, travel cost, return cost, fuel margin, storage state, and reason.
- Required fields: `schema`, `stamp`, `source`, `status`, `mode`, `goal`, `utility`, `components`, `return_feasible`, and `reason`; `goal` contains map-frame `x`, `y`, and `yaw` for the next grid cell or base. Here `yaw` means heading direction.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_next_cell_goal` (`/coral_g/next_cell_goal`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/next_cell_goal carries std_msgs/String JSON using coral_g.next_cell_goal.v1. Neighbor relationships: field_planner_node -> topic_next_cell_goal (publish: publishes); topic_next_cell_goal -> mission_planner_node (subscribe: next goal input); topic_next_cell_goal -> collection_report_node (subscribe: summary input). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_next_cell_goal`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_next_cell_goal` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/goal_pose

## Assignment

- Graph ID: `topic_goal_pose`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/goal_pose carries geometry_msgs/PoseStamped.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Owns: message contract `geometry_msgs/PoseStamped`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `mission_planner_node` (mission_planner_node): Publish - publishes

### Downstream Relationships

- `goal_navigation_node` (goal_navigation_node): Subscribe - subscribed by

### Structured Contract Details

- Message type: `geometry_msgs/PoseStamped`
- Schema: none recorded; use the ROS message type as the contract.
- Contains: Map-frame navigation target pose: header.frame_id must be map; pose means position plus orientation, stored as x/y/z and a quaternion.
- Publishers: `mission_planner_node`
- Subscribers: `goal_navigation_node`

### Source-Backed Contract Notes

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Implemented interface role: Map-frame goal consumed by `goal_navigation_node`.
- Owner: `mission_planner_node` or the existing `send_goal` helper publishes; `goal_navigation_node` subscribes.
- Stability note: Keep `/coral_g/goal_pose` as `geometry_msgs/PoseStamped` and require map-frame goals.

## Implementation Brief

Review `topic_goal_pose` as an implemented ROS topic interface.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Message type: `geometry_msgs/PoseStamped`
- Role: Map-frame goal consumed by `goal_navigation_node`.
- Owner: `mission_planner_node` or the existing `send_goal` helper publishes; `goal_navigation_node` subscribes.
- Stability note: Keep `/coral_g/goal_pose` as `geometry_msgs/PoseStamped` and require map-frame goals.
- Contains: Map-frame navigation target pose: header.frame_id must be map; pose means position plus orientation, stored as x/y/z and a quaternion.
- Confirm publisher/subscriber ownership without changing message type or introducing a parallel topic.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_goal_pose` (`/coral_g/goal_pose`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/goal_pose carries geometry_msgs/PoseStamped. Neighbor relationships: mission_planner_node -> topic_goal_pose (publish: publishes); topic_goal_pose -> goal_navigation_node (subscribe: subscribed by). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_goal_pose`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_goal_pose` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, and ROS interface ownership are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/navigation_status

## Assignment

- Graph ID: `topic_navigation_status`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/navigation_status carries std_msgs/String.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Owns: message contract `std_msgs/String`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `goal_navigation_node` (goal_navigation_node): Publish - publishes

### Downstream Relationships

- `mission_planner_node` (mission_planner_node): Subscribe - subscribed by
- `robot_state_node` (robot_state_node): Subscribe - subscribed by
- `collection_report_node` (collection_report_node): Subscribe - summary input

### Structured Contract Details

- Message type: `std_msgs/String`
- Schema: none recorded; use the ROS message type as the contract.
- Contains: Navigation state string: waiting_for_nav2, ready, received, active, succeeded, canceled, timeout, or failed.
- Publishers: `goal_navigation_node`
- Subscribers: `mission_planner_node`, `robot_state_node`, `collection_report_node`

### Source-Backed Contract Notes

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Implemented interface role: Simplified navigation state for mission planning, robot state, and demo reporting.
- Owner: `goal_navigation_node` publishes; `mission_planner_node`, `robot_state_node`, and `collection_report_node` subscribe.
- Stability note: Keep `/coral_g/navigation_status` as `std_msgs/String` with states such as `waiting_for_nav2`, `ready`, `received`, `active`, `succeeded`, `canceled`, `timeout`, or `failed`.

## Implementation Brief

Review `topic_navigation_status` as an implemented ROS topic interface.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Message type: `std_msgs/String`
- Role: Simplified navigation state for mission planning, robot state, and demo reporting.
- Owner: `goal_navigation_node` publishes; `mission_planner_node`, `robot_state_node`, and `collection_report_node` subscribe.
- Stability note: Keep `/coral_g/navigation_status` as `std_msgs/String` with states such as `waiting_for_nav2`, `ready`, `received`, `active`, `succeeded`, `canceled`, `timeout`, or `failed`.
- Contains: Navigation state string: waiting_for_nav2, ready, received, active, succeeded, canceled, timeout, or failed.
- Confirm publisher/subscriber ownership without changing message type or introducing a parallel topic.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_navigation_status` (`/coral_g/navigation_status`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/navigation_status carries std_msgs/String. Neighbor relationships: goal_navigation_node -> topic_navigation_status (publish: publishes); topic_navigation_status -> mission_planner_node (subscribe: subscribed by); topic_navigation_status -> robot_state_node (subscribe: subscribed by); topic_navigation_status -> collection_report_node (subscribe: summary input). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_navigation_status`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_navigation_status` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, and ROS interface ownership are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /scan

## Assignment

- Graph ID: `topic_scan`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /scan carries sensor_msgs/LaserScan.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Owns: message contract `sensor_msgs/LaserScan`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `turtlebot_gazebo` (TurtleBot / Gazebo): Publish - publishes

### Downstream Relationships

- `nav2` (Nav2): Subscribe - subscribed by
- `twin_input_node` (twin_input_node): Subscribe - subscribed by

### Structured Contract Details

- Message type: `sensor_msgs/LaserScan`
- Schema: none recorded; use the ROS message type as the contract.
- Contains: LiDAR scan ranges, angles, and range limits from TurtleBot/Gazebo; consumed by Nav2 and summarized by twin_input_node.
- Publishers: `TurtleBot / Gazebo`
- Subscribers: `Nav2`, `twin_input_node`

### Source-Backed Contract Notes

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Implemented interface role: Physical or simulated LiDAR sensor data for obstacle detection, SLAM, and Nav2.
- Owner: Gazebo/TurtleBot LiDAR publishes; Nav2 and `twin_input_node` subscribe.
- Stability note: Keep `/scan` as `sensor_msgs/LaserScan` and do not replace it with a CORAL-G JSON topic.

## Implementation Brief

Review `topic_scan` as an implemented ROS topic interface.
- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Message type: `sensor_msgs/LaserScan`
- Role: Physical or simulated LiDAR sensor data for obstacle detection, SLAM, and Nav2.
- Owner: Gazebo/TurtleBot LiDAR publishes; Nav2 and `twin_input_node` subscribe.
- Stability note: Keep `/scan` as `sensor_msgs/LaserScan` and do not replace it with a CORAL-G JSON topic.
- Contains: LiDAR scan ranges, angles, and range limits from TurtleBot/Gazebo; consumed by Nav2 and summarized by twin_input_node.
- Confirm publisher/subscriber ownership without changing message type or introducing a parallel topic.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_scan` (`/scan`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /scan carries sensor_msgs/LaserScan. Neighbor relationships: turtlebot_gazebo -> topic_scan (publish: publishes); topic_scan -> nav2 (subscribe: subscribed by); topic_scan -> twin_input_node (subscribe: subscribed by). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_scan`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_scan` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, and ROS interface ownership are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /cmd_vel

## Assignment

- Graph ID: `topic_cmd_vel`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /cmd_vel carries geometry_msgs/TwistStamped.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Owns: message contract `geometry_msgs/TwistStamped`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `nav2` (Nav2): Publish - publishes

### Downstream Relationships

- `turtlebot_gazebo` (TurtleBot / Gazebo): Subscribe - subscribed by

### Structured Contract Details

- Message type: `geometry_msgs/TwistStamped`
- Schema: none recorded; use the ROS message type as the contract.
- Contains: Velocity command for the robot; Nav2 is the only intended publisher during navigation demos.
- Publishers: `Nav2 during navigation demos`
- Subscribers: `TurtleBot / Gazebo`

### Source-Backed Contract Notes

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Implemented interface role: Velocity command sent to the robot during navigation.
- Owner: Nav2 is the only intended `/cmd_vel` publisher during navigation demos; TurtleBot/Gazebo subscribes.
- Stability note: Avoid running competing `/cmd_vel` publishers during Nav2 tests.

## Implementation Brief

Review `topic_cmd_vel` as an implemented ROS topic interface.
- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Message type: `geometry_msgs/TwistStamped`
- Role: Velocity command sent to the robot during navigation.
- Owner: Nav2 is the only intended `/cmd_vel` publisher during navigation demos; TurtleBot/Gazebo subscribes.
- Stability note: Avoid running competing `/cmd_vel` publishers during Nav2 tests.
- Contains: Velocity command for the robot; Nav2 is the only intended publisher during navigation demos.
- Confirm publisher/subscriber ownership without changing message type or introducing a parallel topic.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_cmd_vel` (`/cmd_vel`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /cmd_vel carries geometry_msgs/TwistStamped. Neighbor relationships: nav2 -> topic_cmd_vel (publish: publishes); topic_cmd_vel -> turtlebot_gazebo (subscribe: subscribed by). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_cmd_vel`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_cmd_vel` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, and ROS interface ownership are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/robot_state

## Assignment

- Graph ID: `topic_robot_state`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/robot_state carries std_msgs/String JSON using coral_g.robot_state.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.robot_state.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `robot_state_node` (robot_state_node): Publish - publishes

### Downstream Relationships

- `field_planner_node` (field_planner_node): Subscribe - resource state input
- `mission_planner_node` (mission_planner_node): Subscribe - subscribed by
- `collection_report_node` (collection_report_node): Subscribe - summary input

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.robot_state.v1`
- Contains: Robot state summary: map-frame pose (position + heading), storage_fill, fuel_level, at_base, and navigation_status.
- Publishers: `robot_state_node`
- Subscribers: `field_planner_node`, `mission_planner_node`, `collection_report_node`

### Source-Backed Contract Notes

- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Required fields: `stamp`, `source`, `pose`, `storage_fill`, `fuel_level`, `at_base`; `pose` means position plus orientation/heading.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_robot_state` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G adapter - Thin project-owned bridge that summarizes or translates ROS-native data into CORAL-G contracts.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.robot_state.v1`
- Contains: Robot state summary: map-frame pose (position + heading), storage_fill, fuel_level, at_base, and navigation_status.
- Required fields: `stamp`, `source`, `pose`, `storage_fill`, `fuel_level`, `at_base`; `pose` means position plus orientation/heading.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_robot_state` (`/coral_g/robot_state`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/robot_state carries std_msgs/String JSON using coral_g.robot_state.v1. Neighbor relationships: robot_state_node -> topic_robot_state (publish: publishes); topic_robot_state -> field_planner_node (subscribe: resource state input); topic_robot_state -> mission_planner_node (subscribe: subscribed by); topic_robot_state -> collection_report_node (subscribe: summary input). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_robot_state`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_robot_state` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /coral_g/collection_report

## Assignment

- Graph ID: `topic_collection_report`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /coral_g/collection_report carries std_msgs/String JSON using coral_g.collection_report.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.collection_report.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `collection_report_node` (collection_report_node): Publish - publishes

### Downstream Relationships

- `material_belief_node` (material_belief_node): Subscribe - updates belief
- `digital_twin_state` (digital_twin_state): Subscribe - updates twin

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.collection_report.v1`
- Contains: Collection event: map-frame location, materials with counts, total count, collection_radius_m, and status.
- Publishers: `collection_report_node`
- Subscribers: `material_belief_node`, `digital_twin_state`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Required fields: `stamp`, `source`, `location`, `materials[]`, `count`.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_collection_report` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.collection_report.v1`
- Contains: Collection event: map-frame location, materials with counts, total count, collection_radius_m, and status.
- Required fields: `stamp`, `source`, `location`, `materials[]`, `count`.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_collection_report` (`/coral_g/collection_report`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /coral_g/collection_report carries std_msgs/String JSON using coral_g.collection_report.v1. Neighbor relationships: collection_report_node -> topic_collection_report (publish: publishes); topic_collection_report -> material_belief_node (subscribe: updates belief); topic_collection_report -> digital_twin_state (subscribe: updates twin). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_collection_report`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_collection_report` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# /report

## Assignment

- Graph ID: `topic_report`
- Kind: `topic`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm producers, consumers, message type, and expected payload semantics.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: /report carries std_msgs/String JSON using coral_g.report.v1.
- Workflow area: ROS interface
- Workflow role: First-class topic contract between publisher and subscriber components.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: message contract `std_msgs/String JSON` with schema `coral_g.report.v1`.
- Does not own: publisher or subscriber implementation internals outside this message contract.

## Contract Surface

### Upstream Relationships

- `collection_report_node` (collection_report_node): Publish - publishes

### Downstream Relationships

- `demo_control_surface` (demo_control_surface): Subscribe - demo output

### Structured Contract Details

- Message type: `std_msgs/String JSON`
- Schema: `coral_g.report.v1`
- Contains: Human/demo summary: run_id, next_cell_goal, collection_stats, distribution_shift, robot_summary, field_policy, and heatmap_cells.
- Publishers: `collection_report_node`
- Subscribers: `demo_control_surface`

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Required fields: `stamp`, `source`, `run_id`, `next_cell_goal`, `collection_stats`, `distribution_shift`, `robot_summary`, `field_policy`, `heatmap_cells[]`.
- JSON convention: every JSON message includes `schema`, `stamp`, and `source`; coordinates use the `map` frame unless a contract says otherwise.

## Implementation Brief

Review `topic_report` as a first-class ROS topic contract, not as an implementation component.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Message type: `std_msgs/String JSON`
- Schema: `coral_g.report.v1`
- Contains: Human/demo summary: run_id, next_cell_goal, collection_stats, distribution_shift, robot_summary, field_policy, and heatmap_cells.
- Required fields: `stamp`, `source`, `run_id`, `next_cell_goal`, `collection_stats`, `distribution_shift`, `robot_summary`, `field_policy`, `heatmap_cells[]`.
- Confirm every publisher emits the required shape and every subscriber reads the same shape.
- Do not add custom ROS message packages for v1 JSON topics.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `topic_report` (`/report`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: /report carries std_msgs/String JSON using coral_g.report.v1. Neighbor relationships: collection_report_node -> topic_report (publish: publishes); topic_report -> demo_control_surface (subscribe: demo output). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `topic_report`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `topic_report` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Publisher, subscriber, message type, required fields, and JSON schema expectations are confirmed.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# NavigateToPose

## Assignment

- Graph ID: `action_navigate_to_pose`
- Kind: `action`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm long-running navigation ownership, goal, feedback, result, and cancel expectations.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: NavigateToPose coordinates goal_navigation_node with Nav2.
- Workflow area: ROS interface
- Workflow role: Long-running action boundary between a CORAL-G client and Nav2 server.
- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Owns: action goal, feedback, result, and cancel expectations between `goal_navigation_node` and `Nav2`.
- Does not own: Nav2 internals or unrelated navigation topics outside NavigateToPose integration.

## Contract Surface

### Upstream Relationships

- `goal_navigation_node` (goal_navigation_node): Action client - action client

### Downstream Relationships

- `nav2` (Nav2): Action server - action server

### Structured Contract Details

- Client: goal_navigation_node
- Server: Nav2
- Goal: map-frame target pose (position + orientation)
- Feedback: navigation progress and current pose (position + orientation)
- Result: succeeded, canceled, or failed
- Cancel: cancel active goal on timeout or replacement goal

### Source-Backed Contract Notes

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Contract: `goal_navigation_node` receives `/coral_g/goal_pose`, calls Nav2 NavigateToPose through `BasicNavigator`, and reports simplified status on `/coral_g/navigation_status`.
- Required frame: incoming goals must use `header.frame_id: "map"`.

## Implementation Brief

Review this action as the Nav2 NavigateToPose boundary.

- Built from: ROS-native / prebuilt - Provided mostly by TurtleBot, Gazebo, SLAM Toolbox, Nav2, or standard ROS 2 interfaces.
- Confirm the client/server ownership, map-frame goal semantics, feedback/result/cancel behavior, timeout cancellation, and simplified navigation status publication.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `action_navigate_to_pose` (`NavigateToPose`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: NavigateToPose coordinates goal_navigation_node with Nav2. Neighbor relationships: goal_navigation_node -> action_navigate_to_pose (action-client: action client); action_navigate_to_pose -> nav2 (action-server: action server). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `action_navigate_to_pose`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `action_navigate_to_pose` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Client/server ownership and goal-feedback-result-cancel behavior are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# reset_reseed

## Assignment

- Graph ID: `service_reset_reseed`
- Kind: `service`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm request/response shape and when the service should be used.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: reset_reseed is a request/response boundary between demo_control_surface and debris_simulation_node.
- Workflow area: ROS interface
- Workflow role: Synchronous request/response boundary for optional demo reset behavior.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: service request `{ reason, random_seed }` and response `{ accepted, message }`.
- Does not own: continuous mission state, which should stay topic-driven in v1.

## Contract Surface

### Upstream Relationships

- `demo_control_surface` (demo_control_surface): Service client - service client

### Downstream Relationships

- `debris_simulation_node` (debris_simulation_node): Service provider - service provider

### Structured Contract Details

- Client: demo_control_surface
- Provider: debris_simulation_node
- Request: { reason, random_seed }
- Response: { accepted, message }

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Contract: optional future reset/reseed command for `debris_simulation_node`; use only if the demo needs manual reset.
- Proposed request/response shape is source-backed, but continuous mission state should remain topic-driven.

## Implementation Brief

Review this service as an optional manual reset/reseed boundary only.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- It must not replace topic-driven mission updates or introduce hidden state changes outside the debris simulation reset use case.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `service_reset_reseed` (`reset_reseed`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: reset_reseed is a request/response boundary between demo_control_surface and debris_simulation_node. Neighbor relationships: demo_control_surface -> service_reset_reseed (service-client: service client); service_reset_reseed -> debris_simulation_node (service-provider: service provider). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `service_reset_reseed`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `service_reset_reseed` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Request/response behavior and demo usage are clear and remain optional.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# preset_ocean_conditions

## Assignment

- Graph ID: `param_preset_ocean_conditions`
- Kind: `parameter`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm default value, owning node, and when the team may change it.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Define the deterministic ocean-current, wind, wave, cleanup-zone, and grid defaults used when no external API is connected.
- Workflow area: Configuration
- Workflow role: Owned configuration default that keeps demo behavior deterministic and reviewable.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: default/configuration value `10m x 10m mock current/wind/wave grid, 1m cells` for `twin_input_node`.
- Does not own: runtime editing UI, teammate assignment state, or graph topology.

## Contract Surface

### Upstream Relationships

- None recorded in the current graph.

### Downstream Relationships

- `twin_input_node` (twin_input_node): Parameter owner - configures

### Structured Contract Details

- Owner: twin_input_node
- Value: 10m x 10m mock current/wind/wave grid, 1m cells
- Purpose: Define the deterministic ocean-current, wind, wave, cleanup-zone, and grid defaults used when no external API is connected.

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `preset_ocean_conditions`: 10m x 10m cleanup zone, 1m grid cells, deterministic mock current, wind, and wave fields, `tick_rate_hz=1.0`.
- Reason: Makes the environmental assumption explicit and readable: this parameter is not just a generic mock mode, it defines the ocean-condition field used by the PoC when no external API is connected.

## Implementation Brief

Review `param_preset_ocean_conditions` as configuration owned by `twin_input_node`.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `preset_ocean_conditions`: 10m x 10m cleanup zone, 1m grid cells, deterministic mock current, wind, and wave fields, `tick_rate_hz=1.0`.
- Reason: Makes the environmental assumption explicit and readable: this parameter is not just a generic mock mode, it defines the ocean-condition field used by the PoC when no external API is connected.
- Confirm the owner uses this value consistently and that any change is deliberate, documented, and coordinated with dependent nodes.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `param_preset_ocean_conditions` (`preset_ocean_conditions`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Define the deterministic ocean-current, wind, wave, cleanup-zone, and grid defaults used when no external API is connected. Neighbor relationships: param_preset_ocean_conditions -> twin_input_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `param_preset_ocean_conditions`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `param_preset_ocean_conditions` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Default value, reason, owner, and allowed change process are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# debris_count=100

## Assignment

- Graph ID: `param_simulation_defaults`
- Kind: `parameter`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm default value, owning node, and when the team may change it.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Seed enough virtual debris particles to make the density field visible and repeatable.
- Workflow area: Configuration
- Workflow role: Owned configuration default that keeps demo behavior deterministic and reviewable.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: default/configuration value `count=100, horizon=60s, seed=23` for `debris_simulation_node`.
- Does not own: runtime editing UI, teammate assignment state, or graph topology.

## Contract Surface

### Upstream Relationships

- None recorded in the current graph.

### Downstream Relationships

- `debris_simulation_node` (debris_simulation_node): Parameter owner - configures

### Structured Contract Details

- Owner: debris_simulation_node
- Value: count=100, horizon=60s, seed=23
- Purpose: Seed enough virtual debris particles to make the density field visible and repeatable.

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `debris_count=100`, `simulation_horizon_sec=60.0`, `random_seed=23`.
- Reason: Uses enough particles to show the density field clearly while keeping Team 23 demo output repeatable.

## Implementation Brief

Review `param_simulation_defaults` as configuration owned by `debris_simulation_node`.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `debris_count=100`, `simulation_horizon_sec=60.0`, `random_seed=23`.
- Reason: Uses enough particles to show the density field clearly while keeping Team 23 demo output repeatable.
- Confirm the owner uses this value consistently and that any change is deliberate, documented, and coordinated with dependent nodes.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `param_simulation_defaults` (`debris_count=100`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Seed enough virtual debris particles to make the density field visible and repeatable. Neighbor relationships: param_simulation_defaults -> debris_simulation_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `param_simulation_defaults`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `param_simulation_defaults` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Default value, reason, owner, and allowed change process are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# field_policy

## Assignment

- Graph ID: `param_field_policy`
- Kind: `parameter`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm default value, owning node, and when the team may change it.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Score each reachable grid cell as a utility field while enforcing enough fuel margin to return to base.
- Workflow area: Configuration
- Workflow role: Owned configuration default that keeps demo behavior deterministic and reviewable.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: default/configuration value `density_reward=1.0, travel_cost=0.2, storage_penalty=0.5, fuel_penalty=0.5, return_reserve=0.2` for `field_planner_node`.
- Does not own: runtime editing UI, teammate assignment state, or graph topology.

## Contract Surface

### Upstream Relationships

- None recorded in the current graph.

### Downstream Relationships

- `field_planner_node` (field_planner_node): Parameter owner - configures

### Structured Contract Details

- Owner: field_planner_node
- Value: density_reward=1.0, travel_cost=0.2, storage_penalty=0.5, fuel_penalty=0.5, return_reserve=0.2
- Purpose: Score each reachable grid cell as a utility field while enforcing enough fuel margin to return to base.

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `density_reward_weight=1.0`, `travel_cost_weight=0.2`, `storage_penalty_weight=0.5`, `fuel_penalty_weight=0.5`, `map_risk_weight=0.5`, `return_reserve=0.2`, `min_density_reward=0.1`.
- Reason: Keeps the field planner simple and explainable: every cell is scored directly, unreachable cells are rejected, and return-to-base feasibility is enforced before a next cell is selected.

## Implementation Brief

Review `param_field_policy` as configuration owned by `field_planner_node`.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `density_reward_weight=1.0`, `travel_cost_weight=0.2`, `storage_penalty_weight=0.5`, `fuel_penalty_weight=0.5`, `map_risk_weight=0.5`, `return_reserve=0.2`, `min_density_reward=0.1`.
- Reason: Keeps the field planner simple and explainable: every cell is scored directly, unreachable cells are rejected, and return-to-base feasibility is enforced before a next cell is selected.
- Confirm the owner uses this value consistently and that any change is deliberate, documented, and coordinated with dependent nodes.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `param_field_policy` (`field_policy`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Score each reachable grid cell as a utility field while enforcing enough fuel margin to return to base. Neighbor relationships: param_field_policy -> field_planner_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `param_field_policy`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `param_field_policy` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Default value, reason, owner, and allowed change process are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# return_policy

## Assignment

- Graph ID: `param_return_policy`
- Kind: `parameter`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm default value, owning node, and when the team may change it.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Defines the Gazebo/map base pose and switches from next-cell cleanup goals to base goals before storage or fuel becomes critical.
- Workflow area: Configuration
- Workflow role: Owned configuration default that keeps demo behavior deterministic and reviewable.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: default/configuration value `base=(0,0,0), storage=0.8, fuel=0.25` for `mission_planner_node`.
- Does not own: runtime editing UI, teammate assignment state, or graph topology.

## Contract Surface

### Upstream Relationships

- None recorded in the current graph.

### Downstream Relationships

- `mission_planner_node` (mission_planner_node): Parameter owner - configures
- `robot_state_node` (robot_state_node): Parameter owner - base reference

### Structured Contract Details

- Owner: mission_planner_node
- Value: base=(0,0,0), storage=0.8, fuel=0.25
- Purpose: Defines the Gazebo/map base pose and switches from next-cell cleanup goals to base goals before storage or fuel becomes critical.

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `base_x=0.0`, `base_y=0.0`, `base_yaw=0.0`, `storage_return_threshold=0.8`, `fuel_return_threshold=0.25`.
- Reason: Folds the former base-station concept into the return policy: the Gazebo/map base pose is the origin marker, and the thresholds decide when the mission planner or field planner returns there.

## Implementation Brief

Review `param_return_policy` as configuration owned by `mission_planner_node`.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `base_x=0.0`, `base_y=0.0`, `base_yaw=0.0`, `storage_return_threshold=0.8`, `fuel_return_threshold=0.25`.
- Reason: Folds the former base-station concept into the return policy: the Gazebo/map base pose is the origin marker, and the thresholds decide when the mission planner or field planner returns there.
- Confirm the owner uses this value consistently and that any change is deliberate, documented, and coordinated with dependent nodes.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `param_return_policy` (`return_policy`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Defines the Gazebo/map base pose and switches from next-cell cleanup goals to base goals before storage or fuel becomes critical. Neighbor relationships: param_return_policy -> mission_planner_node (parameter-owner: configures); param_return_policy -> robot_state_node (parameter-owner: base reference). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `param_return_policy`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `param_return_policy` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Default value, reason, owner, and allowed change process are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# goal_timeout_sec=120

## Assignment

- Graph ID: `param_goal_timeout`
- Kind: `parameter`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm default value, owning node, and when the team may change it.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Cancel a navigation goal that exceeds the demo timeout window.
- Workflow area: Configuration
- Workflow role: Owned configuration default that keeps demo behavior deterministic and reviewable.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: default/configuration value `120.0` for `goal_navigation_node`.
- Does not own: runtime editing UI, teammate assignment state, or graph topology.

## Contract Surface

### Upstream Relationships

- None recorded in the current graph.

### Downstream Relationships

- `goal_navigation_node` (goal_navigation_node): Parameter owner - configures

### Structured Contract Details

- Owner: goal_navigation_node
- Value: 120.0
- Purpose: Cancel a navigation goal that exceeds the demo timeout window.

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `goal_timeout_sec=120.0`.
- Reason: Preserves the existing navigation timeout default for demo-scale reachable goals.

## Implementation Brief

Review `param_goal_timeout` as configuration owned by `goal_navigation_node`.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `goal_timeout_sec=120.0`.
- Reason: Preserves the existing navigation timeout default for demo-scale reachable goals.
- Confirm the owner uses this value consistently and that any change is deliberate, documented, and coordinated with dependent nodes.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `param_goal_timeout` (`goal_timeout_sec=120`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Cancel a navigation goal that exceeds the demo timeout window. Neighbor relationships: param_goal_timeout -> goal_navigation_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `param_goal_timeout`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `param_goal_timeout` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Default value, reason, owner, and allowed change process are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# state_defaults

## Assignment

- Graph ID: `param_robot_state_defaults`
- Kind: `parameter`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm default value, owning node, and when the team may change it.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Track storage, fuel, and telemetry tick behavior; base pose comes from return_policy.
- Workflow area: Configuration
- Workflow role: Owned configuration default that keeps demo behavior deterministic and reviewable.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: default/configuration value `storage_capacity_items, fuel_capacity_norm, state_tick_rate_hz` for `robot_state_node`.
- Does not own: runtime editing UI, teammate assignment state, or graph topology.

## Contract Surface

### Upstream Relationships

- None recorded in the current graph.

### Downstream Relationships

- `robot_state_node` (robot_state_node): Parameter owner - configures

### Structured Contract Details

- Owner: robot_state_node
- Value: storage_capacity_items, fuel_capacity_norm, state_tick_rate_hz
- Purpose: Track storage, fuel, and telemetry tick behavior; base pose comes from return_policy.

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `storage_capacity_items`, `fuel_capacity_norm`, `state_tick_rate_hz`.
- Reason: Supports normalized storage/fuel tracking without duplicating the base pose owned by `return_policy`.

## Implementation Brief

Review `param_robot_state_defaults` as configuration owned by `robot_state_node`.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `storage_capacity_items`, `fuel_capacity_norm`, `state_tick_rate_hz`.
- Reason: Supports normalized storage/fuel tracking without duplicating the base pose owned by `return_policy`.
- Confirm the owner uses this value consistently and that any change is deliberate, documented, and coordinated with dependent nodes.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `param_robot_state_defaults` (`state_defaults`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Track storage, fuel, and telemetry tick behavior; base pose comes from return_policy. Neighbor relationships: param_robot_state_defaults -> robot_state_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `param_robot_state_defaults`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `param_robot_state_defaults` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Default value, reason, owner, and allowed change process are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.

---

# collection_defaults

## Assignment

- Graph ID: `param_collection_defaults`
- Kind: `parameter`
- Hosted status source: Neon-backed dashboard and `GET /api/node-status`
- Local fallback status: `queued`
- Local fallback people: unassigned
- Current focus: Confirm default value, owning node, and when the team may change it.

## Project Overview

CORAL-G is a ROS 2 Digital Twin Application System for a bounded ocean-cleanup proof of concept. The demo uses a TurtleBot/Gazebo/Nav2 robot stack, LiDAR for implemented robot sensing, mock/static environmental inputs for current, wind, and wave fields, and a digital twin loop where robot observations update the twin map, fused twin state constrains debris prediction, utility-field planning chooses the next grid cell or base return, collection updates material beliefs, and return-to-base handles offload/refuel behavior.

This packet is a build/review brief for one stable graph node. Work through the explicit topic, action, service, and parameter relationships listed here. Do not rename ROS topics, schema names, message types, action/service boundaries, parameter ownership, or graph links unless the team explicitly approves a contract redesign.

## Plain-Language Notes

- `pose` means position plus orientation. In 2D examples this is usually `x`, `y`, and heading/yaw.
- `frame_id: "map"` means coordinates are in the shared world map, not relative to the robot body.
- `density` is a predicted debris reward signal, not a guarantee that debris is physically present.
- `utility` is the planner's final score after rewards, costs, and return safety are considered.
- `status` says whether a message is usable, waiting, degraded, idle, active, succeeded, failed, and so on.
- `unknown` is an intentional category for missing or unrecognized evidence; do not guess it away.

## Node Responsibility

- Responsibility: Constrain manual/demo collection reports to known v1 material labels.
- Workflow area: Configuration
- Workflow role: Owned configuration default that keeps demo behavior deterministic and reviewable.
- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Owns: default/configuration value `allowed_materials, default_material, collection_radius_m` for `collection_report_node`.
- Does not own: runtime editing UI, teammate assignment state, or graph topology.

## Contract Surface

### Upstream Relationships

- None recorded in the current graph.

### Downstream Relationships

- `collection_report_node` (collection_report_node): Parameter owner - configures

### Structured Contract Details

- Owner: collection_report_node
- Value: allowed_materials, default_material, collection_radius_m
- Purpose: Constrain manual/demo collection reports to known v1 material labels.

### Source-Backed Contract Notes

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `allowed_materials`, `default_material`, `collection_radius_m`.
- Reason: Constrains manual/demo collection reports to known v1 material labels.

## Implementation Brief

Review `param_collection_defaults` as configuration owned by `collection_report_node`.

- Built from: CORAL-G custom DTAS - Project-specific digital-twin, planning, scoring, feedback, or demo contract logic.
- Default: `allowed_materials`, `default_material`, `collection_radius_m`.
- Reason: Constrains manual/demo collection reports to known v1 material labels.
- Confirm the owner uses this value consistently and that any change is deliberate, documented, and coordinated with dependent nodes.

## Builder Agent Prompt

You are building or reviewing CORAL-G graph node `param_collection_defaults` (`collection_defaults`). Use `project-handoff/contracts/contracts-plan.md` and `visual-node-map/src/app/graph-data.ts` as source truth. Preserve all existing ROS topic names, schema names, message types, action/service boundaries, parameter ownership, and graph relationships. Focus on this node's responsibility: Constrain manual/demo collection reports to known v1 material labels. Neighbor relationships: param_collection_defaults -> collection_report_node (parameter-owner: configures). Implement or verify only behavior supported by the source docs; when a detail is missing, write it down as an implementation gap instead of inventing a new contract. Acceptance requires the node's upstream/downstream contracts, expected behavior, failure/degraded behavior, and demo evidence to be explainable to the team.

## Claiming Teammate Prompt

Open https://visual-node-map-kappa.vercel.app, select `param_collection_defaults`, and read this packet before claiming. Get the shared admin key through a private team channel only; do not write it in docs, commits, screenshots, or chat. If you are taking this node, enter the key in the inspector, add your own name to People, set status to `in_progress`, and set Current focus to the immediate implementation or review task. Start by checking component nodes before claiming related topic, action, service, or parameter nodes. Reload the page and reselect `param_collection_defaults` to confirm the Neon-backed update persisted.

## Acceptance Checks

- Teammate or agent can explain what this node owns and what it does not own.
- Upstream and downstream graph relationships are understood and unchanged.
- Live status, people, and current focus are current in the hosted dashboard when the node is claimed.
- No ROS topic name, schema name, message type, action/service boundary, parameter owner, or graph link changed accidentally.
- Default value, reason, owner, and allowed change process are clear.

## Status Data Note

Neon is the active hosted source for mutable assignment fields: status, people, current focus, and updated date. Use the hosted inspector at https://visual-node-map-kappa.vercel.app or `GET /api/node-status` to read live status. Keep `team-data/node-status.json` as fallback/export/reseed data only unless a deliberate sync task is requested.
