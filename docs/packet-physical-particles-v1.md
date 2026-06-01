# Packet: Physical Particles V1

Goal:
Replace the current cluster-shaped physical drift placeholder with hidden
physical debris particles, while preserving existing public mission contracts.

Product spine:
The team can inspect physical truth as moving debris particles, not clusters,
and can see collected versus washed-out counts without leaking physical truth to
the planner or digital prediction.

Context:
The visual/handoff docs define `physical_debris_simulation_node` as the owner of
hidden physical debris particles and material behavior. Runtime code still uses
`environment_node`; keep the runtime name for now, but change the internal
model.

Outcome:
- `environment_node` keeps its node/executable name.
- Internal truth becomes a deterministic particle cloud over free environment
  cells.
- Physical truth seeds 100 small particles by default.
- Each particle has material, position, velocity, acceleration, and status.
- Particles move from layered force-field response plus small boid behavior.
- Particles that touch land/non-water become `washed_out`.
- Robot contact changes particle status to `collected`.
- Collected and washed-out particles are logged into cumulative counters and
  replaced on the next tick so the physical truth maintains 100 active
  particles.
- Collection radius is a lowered code constant, currently `0.2m`.
- `/collection_event` remains compatible, but may report multiple collected
  items in one tick.
- `/environment_observation` remains unchanged except existing environment data.
- `/dashboard` may expose debug-only physical particles, material counts,
  collected count, remaining active count, and washed-out count.

Constraints:
- Minimal code changes.
- No executable or node rename.
- No planner behavior changes.
- No prediction rewrite in this packet.
- No physical particle IDs in planner-facing topics.
- Do not add a public physical-particle ROS topic.
- Do not keep extending cluster semantics.

Repo/context discovered:
- `docs/debris-simulation-model.md` defines the corrected vocabulary.
- `environment_node.py` previously owned placeholder `CLUSTERS`.
- `debris_prediction_node.py` now owns a separate 100-particle prediction model
  and publishes normalized density.
- `collection_event` already supports `count`, `items[]`, and `materials`.
- The dashboard is debug-only and can expose hidden truth.

Plan:
1. Add a small pure helper module for debris particles and material response.
2. Seed deterministic physical particles from free environment cells instead of
   prior demo hotspots.
3. Move active particles each tick:
   - force layer response: current, wind, wave/turbulence
   - material coefficients
   - mild cohesion/separation only within a small radius
   - map/field bounds and non-free cells cause `washed_out`
4. Update collection detection to collect particles within robot radius.
5. Aggregate collection events by material/location per tick.
6. Extend `/dashboard` with debug-only `physical_particles` and
   `physical_debris.counts`.
7. Keep old cluster dashboard fields only as a temporary compatibility summary
   if needed by the existing preview.

Verification strategy:
Baseline:
- `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py`
- `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v`

Targeted:
- Unit test deterministic particle seeding.
- Unit test material response produces different movement for at least two
  materials under the same field.
- Unit test particle wash-out when next position is blocked/out of bounds.
- Unit test collection event can aggregate multiple particles and keeps
  `materials` counts correct.
- Unit test `/dashboard` exposes physical particles and washed-out counts while
  `/environment_observation` does not expose hidden particles.

Stretch:
- Docker `colcon build --symlink-install`.
- Run browser preview and confirm physical particle layer shows active,
  collected, and washed-out counters.

Product-readiness:
- A teammate can explain: particles are truth, density cells are planner-facing,
  clusters are obsolete, and washed-out is separate from collected.

Chunk planner decision:
No chunk planner needed for this single physical-truth packet. Use
`$chunked-delivery-planner` only if scope expands into prediction rewrite,
planner density compatibility, or dashboard side-by-side redesign.

Direction warning triggers:
- Stop if implementation requires changing ROS topic names or schemas.
- Stop if hidden particle IDs enter `/debris_density_map`, `/twin_state`, or
  planner input.
- Stop if cluster objects remain the internal source of truth.
- Stop if washed-out is emitted as a collection event without a separate
  explicit decision.
- Stop if "tines" is confirmed to mean something other than tides and the force
  layer model needs renaming.

Handoff notes:
After this packet, implement `Prediction Particles To Normalized Density`, which
replaces predicted cluster points with independent prediction particles and
publishes normalized `density_cells` with no cluster fields.
