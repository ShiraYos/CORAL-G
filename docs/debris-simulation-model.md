# Debris Simulation Model

This document is the current debris-model contract for the CORAL-G runtime work.
It supersedes the earlier cluster-drift shorthand used during the first
dashboard/debug packets.

## Source Evidence

The parent project and visual handoff docs describe the intended model as:

- A bounded ocean cleanup-zone digital twin with virtual garbage-boids drifting
  in current fields.
- An actual simulated world that contains hidden true trash items, including
  position, material, and movement behavior.
- A digital prediction model that contains the robot's belief about debris
  location/material/movement and updates from collection evidence.
- A physical-side debris simulation node that owns hidden debris particles and
  material behavior, emits only `environment_observation` and
  `collection_event`, and never exposes hidden truth to planners.
- A `debris_prediction_node` that owns independent prediction particles and
  publishes `debris_density_map` as a density reward field.

Primary source files:

- `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G_Final_Solution_Outline.md`
- `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/project-handoff/contracts/contracts-plan.md`
- `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/project-handoff/node-context/physical_debris_simulation_node.md`
- `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/project-handoff/node-context/debris_prediction_node.md`
- `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/project-handoff/node-context/topic_debris_density_map.md`

## Correct Vocabulary

### Particle

A particle is a simulated debris item or sampled debris mass unit. Particles are
the moving simulation objects.

Physical truth particles are hidden inside the physical-side simulation. Each
physical particle should carry at least:

- `id` for debug only, never planner input
- `material`
- `x`, `y`
- `vx`, `vy`
- `ax`, `ay`
- `status`: `active`, `collected`, or `washed_out`

Prediction particles are independent digital-belief particles inside
`debris_prediction_node`. They must not mirror physical particle IDs one-to-one.
They may carry material class or material weight internally, but material fields
are not part of the public `debris_density_map` contract. The current material
classes are `plastic`, `wood`, and `metal`; digital prediction also uses
`unknown` as the weak prior/fallback before enough collection evidence exists.

### Density Cell

A density cell is the planner-facing abstraction. It is not a simulated object.
It represents predicted debris mass/probability in a valid water/free cell.

`/debris_density_map` should expose normalized density cells:

- each valid cell has `x`, `y`, and `density`
- density values are in `[0.0, 1.0]`
- the sum of density across all valid published cells should be bounded to `1.0`
  when there is any remaining predicted mass
- invalid land/blocked cells should not carry positive density
- physical particle IDs must never appear in this topic

### Cluster

Cluster is no longer a first-class model concept for runtime implementation.
Earlier code used `CLUSTERS` as a transitional stand-in for debris. That should
be treated as implementation debt.

If the UI later needs hotspot labels, it should derive them from adjacent
high-density cells at render/report time. Do not reintroduce clusters as a
truth or prediction state object.

### Washed Out

When a physical or prediction particle touches land/non-water:

- it is removed from active simulation
- its status becomes `washed_out`
- washed-out counts are tracked separately from collected counts
- dashboard/reporting may show washed-out counts
- a washed-out particle is not a `collection_event`

Physical truth keeps a constant active cloud size. On every tick,
`environment_node` removes collected/washed-out particles from the active set,
increments cumulative debug counters, and respawns enough new hidden particles
over free environment cells to return to 100 active particles.

For the current map model, land/non-water means blocked cells and any position
outside the valid map/field bounds. Unknown cells should be treated as unsafe for
positive density and should not attract particles unless a later contract says
otherwise.

### Force Field Layers

The generated environment field is not one vector conceptually. It should be
treated as layered inputs:

- current: primary water advection
- wind: surface drift, usually stronger for lightweight floating materials
- wave/turbulence: diffusion/noise/spread
- tide: optional periodic water movement layer; if "tines" meant a different
  concept, rename this before implementation
- map/land mask: blocks, removes, or prevents motion into non-water cells

Runtime v1 keeps the ROS field interface unchanged. The field generator
combines these layers into each cell's existing `current_x`, `current_y`,
`wind_x`, `wind_y`, and `wave_height` values:

- wind remains mostly uniform with only small spatial variation
- current varies by cell using base flow, shear, and spatial variation
- tide is folded into current as a low-frequency directional bias
- vortices/spirals are folded into current as local rotational perturbations
- wave/turbulence remains a scalar height/spread signal
- map-aware shore effects redirect landward current into a shoreline tangent,
  lightly damp wind near land, and damp wave height near land while preserving
  open-water wave energy
- blocked and unknown cells have no environment vector and remain invalid for
  debris motion

Do not add new `EnvironmentCell` fields until there is a concrete consumer that
needs separate layer vectors. Physical and prediction particles should continue
to consume the combined cell values through shared material response logic.

Each material has response coefficients per layer. Current runtime classes:

```json
{
  "plastic": {"current": 1.0, "wind": 0.65, "wave": 0.35},
  "wood": {"current": 0.8, "wind": 0.25, "wave": 0.25},
  "metal": {"current": 0.35, "wind": 0.02, "wave": 0.08},
  "unknown": {"current": 0.65, "wind": 0.25, "wave": 0.2}
}
```

The exact coefficients can stay simple and deterministic for the course demo.
`plastic` should be the most wind-sensitive, `metal` should be the least
wind-sensitive and slowest to drift, and `wood` should sit between them.

### Boid Behavior

Boid behavior belongs at the particle level. Use only the smallest useful set of
rules:

- advection by force-field layers
- material-conditioned response
- mild cohesion so related trash can sit together
- mild separation so particles do not collapse to one coordinate
- optional alignment with nearby particle velocity
- map/land rejection or wash-out

Boid behavior should not produce a public "cluster" state. It should move
particles, and density should be derived from particle positions.

## Contract Boundaries

Physical truth:

- owns hidden physical particles
- keeps 100 active particles by tick-based respawn
- may expose particles only on debug/dashboard surfaces
- publishes `environment_observation`
- publishes `collection_event` for robot contact only
- tracks `washed_out` separately

Digital prediction:

- consumes `twin_state`
- owns independent prediction particles
- keeps 100 active prediction particles by the same tick-based respawn rule as
  physical truth
- uses observed material evidence internally
- biases internal prediction particle material assignment from
  `/twin_state.material_evidence`
- maintains a cell posterior distribution for planner-facing density
- updates that distribution from the previous normalized density plus
  truth-originated `collection_event` evidence embedded in `/twin_state`
- normalizes density cells for planner reward; active, observed, washed-out, and
  respawn counts are separate debug/status fields rather than the density value
- publishes normalized `debris_density_map`
- tracks predicted `observed_removed`, `washed_out`, and `respawned` events in
  debug/report summaries
- does not consume `/dashboard`
- does not consume hidden physical particles
- does not publish material probabilities on `/debris_density_map`; material
  belief may appear only on debug-only `/prediction_dashboard`

Planner/reporting:

- planner reads density cells, not particles or clusters
- mission report may summarize density and washed-out counts
- neither planner nor report should require physical particle IDs

Dashboard/debug:

- may show hidden physical particles
- may show predicted particles only if explicitly marked as prediction/debug
- should list physical particle samples under the physical canvas and digital
  prediction events under the prediction canvas
- may consume `/prediction_dashboard` for prediction particle inspection; that
  topic is debug-only and forbidden as planner input
- may expose lab-only reset controls for local iteration; reset controls are not
  mission inputs and must not be consumed by planners
- should support layer toggles:
  - map/land mask
  - force-field current/wind/wave/tide vectors
  - physical particles
  - predicted density heatmap
  - washed-out counts
  - side-by-side truth vs prediction
  - optional overlay comparison

## Current Implementation Gap

The current runtime still has obsolete cluster-shaped implementation details:

- The physical side now uses hidden particles, but the runtime node is still
  named `environment_node` for low-conflict compatibility.
- `debris_prediction_node.py` now uses independent prediction particles and
  normalized density cells, but still publishes `cells` as a compatibility alias
  for the current planner/preview.
- The dashboard now has a side-by-side debug view for physical particles and
  predicted density/prediction particles, with layer toggles. It remains a
  debug-only surface.

These are transitional artifacts from the first debug packets. They should not
be extended except as compatibility shims while replacing them.

## Next Implementation Sequence

1. Physical particle truth v1:
   - keep executable name `environment_node` for low-conflict runtime
   - completed as a runtime step with 100 hidden physical particles seeded over
     free environment cells
   - add material response coefficients
   - add washed-out status/counts
   - keep collection events compatible
   - expose particles only in `/dashboard`

2. Prediction particles to normalized density:
   - completed as a runtime step with 100 prediction particles
   - completed with tick-based respawn aligned to physical truth so active
     prediction mass returns to 100 after observed removals or wash-out
   - `density_cells` are normalized over active prediction particles
   - updated so collection events revise a persistent cell posterior rather than
     treating particle IDs as the belief contract
   - `cells` remains as a temporary compatibility alias
   - material evidence remains internal
   - updated with named materials: `plastic`, `wood`, `metal`, plus `unknown`
     prior/fallback for digital prediction
   - updated so `/twin_state.material_evidence` biases internal prediction
     particle material assignment and therefore material-conditioned drift
   - `/prediction_dashboard` lists recent digital events for inspection

3. Dashboard side-by-side:
   - completed as a debug/runtime step
   - left: physical debug particles, under-canvas physical particle list, and
     washed-out/collected counts
   - right: predicted density heatmap, prediction particle overlay, under-canvas
     digital event list, and predicted washed-out count
   - layer toggles for map, force, physical particles, prediction density, and
     robot position

4. Planner/report compatibility:
   - completed for `field_planner_node`
   - planner consumes normalized `density_cells` first, with `cells` only as a
     temporary compatibility alias
   - planner uses `prediction_counts.active` as remaining debris mass, with
     legacy `clusters_remaining` only as a fallback for older density payloads
   - planner targets density cell centers and no longer reads `cluster_x`,
     `cluster_y`, or `cluster_id`
   - planner scores cells with density reward, travel cost, storage penalty,
     fuel penalty, map risk, return cost, and fuel margin in `/next_cell_goal`
     components
   - planner return-to-base goals use the base pose embedded in `/twin_state`
   - report summarizes density, collection, and washed-out counts
   - no cluster dependency remains in planner decision logic
