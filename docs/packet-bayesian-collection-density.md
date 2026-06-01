# Packet: Bayesian Collection Density Update

## Goal

Make the digital debris belief update from its previous distribution plus
truth-originated collection events, without exposing hidden physical truth.

## Product Spine

The team can inspect how physical collection evidence changes the digital
debris belief while the planner still receives only normalized density cells.

## Outcome

- `/debris_density_map` keeps normalized `density_cells` for planner reward.
- `debris_prediction_node` keeps a persistent cell posterior distribution.
- `/collection_event` evidence reduces posterior density near the event
  location and the result is renormalized.
- Active, observed, washed-out, and respawn counts remain separate debug/status
  metadata, surfaced through `/prediction_dashboard`.
- Prediction particles remain debug samples, not the public belief contract.

## Constraints

- No planner behavior changes.
- No physical truth changes.
- No dashboard topic consumed by prediction or planner.
- No physical particle IDs or material probabilities in `/debris_density_map`.

## Verification

- Unit tests cover collection-driven posterior update, duplicate event dedupe,
  normalized density, and dashboard-only belief metadata.
- Baseline Python compile and unit test commands remain the packet checks.
