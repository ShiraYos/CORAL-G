# Fresh-Chat Handoff: CORAL-G Full Integration

Use $chunked-delivery-planner.
Use $agent-worktree-workflow if worktree isolation is enabled, a worktree path
exists, or an isolated dev preview is started.

## Active Plan

- Goal: deliver the full CORAL-G mission loop from debris/digital state through
  Nav2 execution, collection feedback, density clearing, and repeat/return/idle
  completion.
- Product spine: `debris/environment + robot/base/map inputs -> /twin_state -> /debris_density_map -> /next_cell_goal -> mission_planner_node Nav2 goal -> navigation result -> /collection_event -> prediction clears collected debris -> next target, return-to-base, or idle mission-complete`.
- Ledger:
  [full-integration-ledger.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-ledger.md)
- Handoff source:
  [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md)
- Current chunk: 6. Partner Gazebo/RViz pre-lab package
- Status: `needs_human`; Chunks 2, 3, 4, and 5 are verified.
- Default autonomy: proceed inside accepted chunks; stop at human gates.

## Rolling Chunk Window

| Order | Chunk | Status | Owner/Audience | Product Spine Fit | Scope | Acceptance Criteria | Verification | Gates / Stop Conditions |
|---|---|---|---|---|---|---|---|---|
| Current | 6. Partner Gazebo/RViz pre-lab package | needs_human | Lab partner / reviewer | yes | Partner runs the prepared repo in Gazebo/RViz/Nav2 and records observations. | Partner can run the repo, see planner target, and confirm at least one real `/next_cell_goal` reaches Nav2. | Partner-run checklist and captured observations/log categories. | Needs partner environment; stop if failure categories are not separable. |
| Next 1 | 7. Final lab Linux verification | queued | Lab team | yes | Validate the full demo in the actual lab environment. | TurtleBot reaches at least one target; success feeds `/collection_event`; prediction clears; planner advances, returns, or idles cleanly. | Lab runbook execution and observed topic/robot behavior. | Lab-only risks remain. |

## Completed

- Minimal digital mission loop through Chunk 8 is complete in the superseded
  [digital-mission-loop-ledger.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/digital-mission-loop-ledger.md).
- Full integration direction recorded in
  [full-integration-plan.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-plan.md).
- Old digital-loop ledger and handoff now point to this full-integration plan.
- Chunk 2 verified: `coral_g_nodes.launch.py` exposes demo tuning args,
  `params/coral_g_demo.yaml` records the demo profile, low normalized density is
  unit-tested, and `full-integration-plan.md` contains a Mermaid readiness graph.
- Chunk 3 verified: `mission_planner_node` publishes baseline-compatible
  `/collection_event` after mocked successful cleanup navigation; tests prove
  prediction records observed removal and the planner can continue from updated
  density.
- Chunk 4 verified: active Nav2 goals are protected from planner churn,
  `coral_g_full_demo.launch.py` composes Gazebo + Nav2/SLAM + CORAL-G +
  optional RViz, Docker `--show-args` validates launch arguments, and
  [coral-g-full-demo-runbook.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/coral-g-full-demo-runbook.md)
  gives partner/lab launch, tiered expectations, and triage steps.
- Chunk 5 verified: local code gates and Docker ROS launch checks passed; direct
  local Gazebo/RViz was skipped because `ros2`, `gazebo`/`gz`, and `rviz2` are
  not available in local PATH.

## Next Task

- Start with: run
  [coral-g-full-demo-runbook.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/coral-g-full-demo-runbook.md)
  in the partner Gazebo/RViz/Nav2 environment.
- Scope for Chunk 6: partner confirms build, launches Gazebo/Nav2/CORAL-G,
  checks topics/dashboard, and records whether one real `/next_cell_goal`
  reaches Nav2.
- Acceptance criteria: partner can run the repo, see planner target, and confirm
  at least one real `/next_cell_goal` reaches Nav2. Stretch: TurtleBot reaches
  one target and `/collection_event` feeds prediction.
- Stop for human if: partner logs do not distinguish Nav2 activation, map/pose
  alignment, controller behavior, and CORAL-G contract flow.

## Worktree Closeout

- `$agent-worktree-workflow` invoked: yes, via skill inspection and worktree
  state recording.
- Worktree closeout decision: `skip`
- Worktree path:
  `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G`
- Branch: `full-integration-planning`
- Preview URL: none
- Automation URL: none
- Dev server status: skipped; no web/dev server applies to planning docs
- Cleanup/retention result: current checkout retained; no separate repo
  worktree script exists
- Reason: no repo worktree tooling and no preview server for this planning
  chunk

## Fresh-Chat Prompt

```text
Use $chunked-delivery-planner.
Use $agent-worktree-workflow if worktree isolation is enabled, a worktree path exists, or an isolated dev preview was started.

Workspace or worktree:
/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G

Read these first:
- /Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G/docs/full-integration-handoff.md
- /Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G/docs/full-integration-ledger.md
- /Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G/docs/full-integration-plan.md

Current status:
Chunks 2, 3, 4, and 5 are verified. The full-integration plan/ledger/handoff exist, the old digital-loop tracker is superseded, demo planner tuning is exposed through launch/YAML, the readiness graph exists, mocked Nav2 cleanup success emits /collection_event, active Nav2 goals are protected from planner churn, and optional direct local Gazebo/RViz is skipped because ROS/Gazebo/RViz executables are unavailable here.

Rolling chunk window:
6. Partner Gazebo/RViz pre-lab package - needs_human
7. Final lab Linux verification - queued

Worktree/dev-server disposition:
Current checkout on branch full-integration-planning is retained. Dev server skipped; no preview applies.

Next exact task:
Run the prepared repo in a partner Gazebo/RViz/Nav2 environment using docs/coral-g-full-demo-runbook.md, then record observations and failure category.

Hard stops:
Do not migrate schemas/topics, do not rename environment_node, do not consume /dashboard or /prediction_dashboard from mission nodes, and do not make live Gazebo/Nav2 a local code acceptance gate.
```

## Context

- Repo/workspace:
  `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G`
- Baseline: `c9c95a9` on `debris-minimal-sim`
- Current branch: `full-integration-planning`
- Older PoC branch: `origin/coral-g-digital-twin-poc`; use as reference only
  for goal locking, collection-on-success, initial pose, finite target demo,
  and runbook language.
- Auth/data context: no local secrets required for this planning work.

## Commands

- Setup/read status: `git status --short --branch`
- Compile: `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py`
- Unit suite: `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v`
- Docker build when useful: `docker run --rm -v "$PWD":/ws -w /ws ros:humble bash -lc 'apt-get update >/tmp/apt.log && apt-get install -y python3-colcon-common-extensions >/tmp/apt-install.log && colcon build --symlink-install'`

## Human Gates

- Sign off Chunk 1 before implementation.
- Stop before schema/topic migration, dashboard/debug mission dependencies,
  mandatory local Gazebo/Nav2 gate, product-spine drift, or unclear
  partner/lab acceptance changes.
