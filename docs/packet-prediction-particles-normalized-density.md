# Packet: Prediction Particles + Normalized Density

Goal:
Replace cluster-shaped digital prediction with independent prediction particles
and publish normalized density cells, while adding a lab-only dashboard reset
control for fast iteration.

Product spine:
The team can see a physical particle truth model and a separate digital density
belief model. The planner still receives a density reward field, not particles
or clusters.

Outcome:
- `debris_prediction_node` seeds 100 independent prediction particles by default.
- Prediction particles move from environment cells embedded in `/twin_state`.
- Prediction particles that hit blocked/non-water/out-of-bounds cells become
  `washed_out`.
- Collection events statistically remove nearest predicted mass; they do not
  match physical particle IDs one-to-one.
- The prediction side uses the same tick-based active-count rule as physical
  truth: after observed removals or wash-out, it respawns enough prediction
  particles to return to 100 active particles.
- `/prediction_dashboard` records recent digital events such as
  `observed_removed`, `washed_out`, and `respawned` for inspection only.
- `/debris_density_map` publishes normalized `density_cells`.
- `cells` remains a temporary compatibility alias for current planner/preview
  consumers.
- A dashboard debug button publishes `/debug_reset` so local iteration can reset
  physical and prediction particles.

Constraints:
- No planner behavior change in this packet.
- No physical particle IDs in `/debris_density_map`.
- No material probabilities in `/debris_density_map`.
- Debug reset is lab-only and must not become mission logic.

Verification:
- `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py`
- `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v`
- Docker `colcon build --symlink-install`
- Browser preview confirms 100 prediction particles, normalized density cells,
  physical particles, aligned respawn counts, digital events, and working debug
  reset.
