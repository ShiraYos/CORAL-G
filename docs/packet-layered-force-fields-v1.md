# Packet: Layered Force Fields V1

## Goal

Make the generated environment field more realistic while preserving existing
ROS interfaces and planner contracts.

## Product Spine

Debris motion now starts from a believable water field: stable wind, spatially
varying current, low-frequency tide bias, and local vortex/spiral circulation.

## Outcome

- `EnvironmentCell` stays unchanged.
- The generator still outputs combined `current_x`, `current_y`, `wind_x`,
  `wind_y`, and `wave_height`.
- `environment_field.py` composes those values from base current, shear,
  spatial variation, tide bias, vortices, slow wind variation, and wave
  variation.
- Physical and prediction particles inherit the richer field through existing
  material response logic.

## Constraints

- No planner changes.
- No Bayesian density update changes.
- No physical truth topic exposure.
- No new ROS interface fields.
- No dashboard layout rewrite.

## Verification

- Unit tests cover wind/current variation, vortex rotation, and blocked/unknown
  cell masking.
- Baseline Python compile, unit tests, and Docker `colcon build` remain the
  packet checks.
