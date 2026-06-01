# Packet: Material-Conditioned Prediction

## Goal

Make digital prediction use observed material evidence internally while keeping
planner-facing density material-free.

## Product Spine

After collection evidence says the debris mix is mostly plastic, wood, or metal,
the digital prediction particles drift with that material behavior, but the
planner still receives only normalized density cells.

## Outcome

- Runtime material names are `plastic`, `wood`, and `metal`.
- Digital prediction keeps `unknown` as its weak prior/fallback.
- `/twin_state.material_evidence` biases internal prediction particle material
  assignment.
- Material response coefficients make plastic most wind-sensitive, metal least
  wind-sensitive, and wood intermediate.
- `/debris_density_map` still has no material fields or material probabilities.
- `/prediction_dashboard` may show `material_belief` as debug-only metadata.

## Constraints

- No physical truth leak into prediction.
- No planner changes.
- No ROS interface changes.
- No material probabilities on `/debris_density_map`.

## Verification

- Unit tests cover material response differences, material evidence biasing
  internal prediction particles, and density-map cleanliness.
- Baseline Python compile, unit tests, and Docker `colcon build` remain the
  packet checks.
