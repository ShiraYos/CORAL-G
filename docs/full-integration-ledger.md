# CORAL-G Full Integration Ledger

## Purpose

- Goal: deliver the full CORAL-G mission loop from debris state through digital
  planning, Nav2 goal execution, collection feedback, density clearing, and
  repeat/return/idle completion.
- Product spine: `debris/environment + robot/base/map inputs -> /twin_state -> /debris_density_map -> /next_cell_goal -> mission_planner_node Nav2 goal -> navigation result -> /collection_event -> prediction clears collected debris -> next target, return-to-base, or idle mission-complete`.
- Success criteria: local logic is complete with mocked Nav2; optional local
  Gazebo is attempted if feasible; partner Gazebo/RViz pre-lab run is prepared
  from this repo; final lab Linux acceptance is clearly bounded.
- Ledger owner: Codex.
- Last updated: 2026-06-02.

## Operating Rules

- Default autonomy: proceed inside an accepted chunk; stop at human gates.
- Human gates: stop before schema/topic migration, renaming
  `environment_node`, consuming `/dashboard` or `/prediction_dashboard` from
  mission nodes, requiring live Gazebo/Nav2 as a local code gate, or expanding
  dashboard integration-status UI beyond accepted hover/status behavior.
- Verification baseline: prefer local compile/unit/mocked-ROS checks first;
  Docker build/direct node suite where useful; browser/dashboard checks only
  after UI/dashboard changes; Gazebo/Nav2 checks are optional locally and
  required only at partner/lab boundaries.
- Handoff file:
  [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md).
- Rolling window size: current chunk plus next 2 to 3 meaningful chunks.

## Status Legend

- `queued`: not started
- `in_progress`: actively being worked
- `blocked`: cannot proceed without external fix or missing dependency
- `needs_human`: decision gate reached
- `fixed`: implementation or review pass complete, not yet verified
- `verified`: checks passed for the chunk
- `signed_off`: human accepted the chunk

## Worktree / Preview State

- Workspace/worktree path:
  `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G`
- Branch: `full-integration-planning`
- Base ref: `c9c95a9b82ee1410df625930b020f1fe02c532c0`
- Worktree tooling: no repo `AGENTS.md`, no `package.json`, and no repo-owned
  `wt:*` scripts found.
- Worktree closeout decision for planning chunk: `skip`.
- Dev server / preview: skipped; this planning chunk has no web/dev server.

## Rolling Chunk Window

Refresh this before each Handoff Point and whenever chunk status, next task,
scope, gate, or discovery changes the near-term plan.

| Order | Chunk | Status | Owner/Audience | Product Spine Fit | Scope | Acceptance Criteria | Verification | Gates / Stop Conditions |
|---|---|---|---|---|---|---|---|---|
| Current | 6. Partner Gazebo/RViz pre-lab package | needs_human | Lab partner / reviewer | yes; makes the repo runnable before final lab | Partner runs the prepared build/launch/checklist in a Gazebo/RViz/Nav2 environment and records observations. | Partner can run the repo, see planner target, and confirm at least one real `/next_cell_goal` reaches Nav2; stretch reaches one target and clears it. | Partner-run checklist and captured observations/log categories. | Needs partner environment; stop if failure categories are not separable between Nav2 activation, map/pose, controller behavior, and CORAL-G flow. |
| Next 1 | 7. Final lab Linux verification | queued | Lab team | yes; validates complete demo in actual lab environment | Validate the full demo in the actual lab environment. | TurtleBot reaches at least one target; success feeds `/collection_event`; prediction clears; planner advances, returns, or idles cleanly. | Lab runbook execution and observed topic/robot behavior. | Lab-only risks: Nav2 recovery, SLAM/pose alignment, real-time timing, classroom machine differences. |

## Chunks

| Chunk | Status | Owner/Audience | Scope | Acceptance Criteria | Verification | Notes |
|---|---|---|---|---|---|---|
| 1. Full-integration delivery plan and worktree setup | signed_off | Codex + project owner | Create durable full-integration plan, ledger, and handoff; mark old minimal-loop tracker superseded; record worktree/dev-server state. | New ledger and handoff exist; old tracker clearly points here; assumptions and verification boundaries are captured; no runtime behavior changed. | Technical verification: passed by file creation/readback and `git status`. Product-readiness verification: passed; user approved continuing. | Still on spine: yes. Direction warning: none. Worktree closeout: skip, current checkout is the planning branch and no dev server applies. |
| 2. Planner tuning, launch parameter surface, and visual status graph | verified | Local developer / reviewer | Fix threshold mismatch, expose launch/YAML tuning, and add readiness visualization without confusing mission colors. | Valid normalized density produces expected planner behavior under demo config; default/demo choices documented; visual graph distinguishes complete/local/partner/lab readiness. | Technical verification: passed `py_compile`, 61 unit tests, and `git diff --check`. Product-readiness verification: passed; launch/YAML profile exposes demo threshold and docs graph shows readiness. | Still on spine: yes. Direction warning: none. Dashboard UI was not changed. |
| 3. Local mission lifecycle closure with mocked Nav2 | verified | Local developer / test reviewer | Close Nav2 success back into baseline-compatible collection and density clearing. | Mock success causes `/collection_event`; prediction records observed removal; planner can continue from updated density. | Technical verification: passed `py_compile`, 63 unit tests, and `git diff --check`. Product-readiness verification: passed for local mocked lifecycle closure. | Still on spine: yes. Direction warning: none. No schema/topic migration. |
| 4. Nav2 active-goal protection and repeatable demo launch | verified | Local developer / partner runner | Add active-goal protection and compose reliable launch/runbook for partner testing. | Active goal is protected; launch args are visible; partner can follow one command sequence. | Technical verification: passed `py_compile`, 64 unit tests, `git diff --check`, Docker `colcon build --symlink-install`, Docker `ros2 launch my_tb3_world coral_g_nodes.launch.py --show-args`, and Docker `ros2 launch my_tb3_world coral_g_full_demo.launch.py --show-args`. Product-readiness verification: passed; full launcher and tiered teammate runbook exist. | Still on spine: yes. Direction warning: none. Live Gazebo/Nav2 is not required for local code acceptance. |
| 5. Local max-out and optional local Gazebo/Nav2 attempt | verified | Local developer | Run the strongest feasible local verification, including optional Gazebo if dependencies support it. | Local acceptance is complete; optional Gazebo result is recorded as pass/fail/skipped with cause. | Technical verification: passed local compile/unit checks, Docker build, Docker launch `--show-args`, and executable availability checks. Product-readiness verification: passed; optional local Gazebo/RViz skipped because local executables are unavailable. | Still on spine: yes. Direction warning: none. Environment absence recorded as a verification boundary, not a product failure. |
| 6. Partner Gazebo/RViz pre-lab package | needs_human | Lab partner / reviewer | Partner runs prepared build, launch, reset, topic checks, and failure triage in Gazebo/RViz/Nav2 environment. | Partner can run the repo, see planner target, and confirm at least one real `/next_cell_goal` reaches Nav2; stretch reaches one target and clears it. | Partner-run checklist and captured observations/log categories. | Still on spine: yes. Direction warning trigger: unclear whether failure is Nav2 activation, map/pose alignment, controller behavior, or CORAL-G contract flow. |
| 7. Final lab Linux verification | queued | Lab team | Validate the full demo in the actual lab environment. | TurtleBot reaches at least one target; success feeds `/collection_event`; prediction clears; planner advances, returns, or idles cleanly. | Lab runbook execution and observed topic/robot behavior. | Still on spine: yes. Lab-only risks: Nav2 recovery, SLAM/pose alignment, real-time timing, classroom machine differences. |

## Decisions

| Date | Chunk | Decision | Rationale | Status |
|---|---|---|---|---|
| 2026-06-02 | 1 | Keep `c9c95a9` / `debris-minimal-sim` as canonical baseline. | It contains the verified digital mission loop and stronger tests. | accepted in plan, awaiting sign-off |
| 2026-06-02 | 1 | Treat `origin/coral-g-digital-twin-poc` as reference material, not a merge target. | It adds useful Nav2/demo ideas but deletes current debris loop docs, dashboard, nodes, and tests. | accepted in plan, awaiting sign-off |
| 2026-06-02 | 1 | Keep root mission topics and baseline `dtas.*` contracts. | Avoids migration churn and preserves verified contracts. | accepted in plan, awaiting sign-off |
| 2026-06-02 | 1 | Split verification into local, optional local Gazebo, partner Gazebo/RViz, and final lab Linux boundaries. | Maximizes local confidence while reserving environment-sensitive checks for partner/lab. | accepted in plan, awaiting sign-off |
| 2026-06-02 | 1 | Prefer docs graph plus dashboard hover/status metadata for integration readiness. | Avoids adding a strong map color that competes with mission visualization. | accepted in plan, awaiting sign-off |
| 2026-06-02 | 1 | Human signed off Chunk 1 and allowed implementation to continue. | User confirmed the surfaced needs-human concerns were good and said to keep going. | signed_off |
| 2026-06-02 | 2 | Keep the `field_planner_node` code default `min_density_reward=0.1`, but set the CORAL-G demo launch/profile default to `0.001`. | Preserves conservative node behavior while letting normalized density produce visible demo mission goals. | verified |
| 2026-06-02 | 2 | Add a docs Mermaid readiness graph, not dashboard UI, for this chunk. | Satisfies visual progress without adding new mission-map colors or debug surface. | verified |
| 2026-06-02 | 3 | Publish baseline-compatible `/collection_event` from `mission_planner_node` only after successful cleanup Nav2 results. | Closes the local mission lifecycle without changing topic schemas or consuming debug/dashboard topics. | verified |
| 2026-06-02 | 4 | Ignore new planner goals while an accepted Nav2 goal is active. | Prevents planner churn from canceling/replacing an in-flight navigation goal. | verified |
| 2026-06-02 | 4 | Add `coral_g_full_demo.launch.py` as the composed Gazebo + Nav2/SLAM + CORAL-G + optional RViz launcher. | Gives teammate one primary launcher while keeping dashboard separate for source-checkout/browser access. | verified |
| 2026-06-02 | 5 | Skip direct local Gazebo/RViz attempt because local ROS/Gazebo/RViz executables are unavailable. | Avoids treating local environment absence as a product failure; Docker ROS checks still verify build and launch arguments. | verified |

## Findings

| Date | Chunk | Finding | Severity | Follow-up |
|---|---|---|---|---|
| 2026-06-02 | 1 | `field_planner_node.min_density_reward=0.1` can idle on valid normalized density; preview used `0.001`. | high for full demo reliability | Chunk 2 |
| 2026-06-02 | 1 | Current baseline has `mission_planner_node` Nav2 boundary and launch files; older PoC has useful goal-locking/collection ideas but incompatible topic model. | medium | Chunks 3-4 |
| 2026-06-02 | 1 | Repo has no worktree/dev-server scripts; current checkout is the planning branch worktree. | low | Record worktree closeout as skipped unless a separate implementation worktree is explicitly requested. |
| 2026-06-02 | 2 | `coral_g_nodes.launch.py` hardcoded the conservative threshold; launch/profile tuning was the right surface for demo behavior. | medium | Completed in Chunk 2. |
| 2026-06-02 | 3 | Prediction currently records observed removal and recomputes density; it does not permanently reduce the target debris count because the predictor respawns to its configured count. | medium | Chunk 5 or 6 should decide whether the demo wants finite target depletion or current continuous-belief behavior. |
| 2026-06-02 | 4 | Docker `--show-args` confirms both the CORAL-G nodes launcher and composed full-demo launcher expose expected arguments. | low | Use runbook for partner execution. |
| 2026-06-02 | 5 | `ros2`, `gazebo`, `gz`, and `rviz2` were not found in local PATH. | medium | Partner Gazebo/RViz boundary is now the next human gate. |

## Verification Log

| Date | Chunk | Command/Check | Result | Evidence |
|---|---|---|---|---|
| 2026-06-02 | 1 | `git status --short --branch` | passed | Branch is `full-integration-planning`; docs are uncommitted planning changes. |
| 2026-06-02 | 1 | `find docs -maxdepth 1 -type f ...` | passed | Existing minimal-loop ledger/handoff and new full plan found. |
| 2026-06-02 | 1 | Worktree script inspection | passed | No `AGENTS.md`, no `package.json`, no repo `wt:*` scripts. |
| 2026-06-02 | 2 | `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py` | passed | No compile output. |
| 2026-06-02 | 2 | `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v` | passed | 61 tests passed. |
| 2026-06-02 | 2 | `git diff --check` | passed | No whitespace errors. |
| 2026-06-02 | 3 | `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py` | passed | No compile output. |
| 2026-06-02 | 3 | `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v` | passed | 63 tests passed. |
| 2026-06-02 | 3 | `git diff --check` | passed | No whitespace errors. |
| 2026-06-02 | 4 | `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py` | passed | No compile output. |
| 2026-06-02 | 4 | `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v` | passed | 64 tests passed. |
| 2026-06-02 | 4 | `git diff --check` | passed | No whitespace errors. |
| 2026-06-02 | 4 | Docker `colcon build --symlink-install` plus `ros2 launch my_tb3_world coral_g_nodes.launch.py --show-args` | passed | Launch args include `demo_params_file`, current/wind/wave/drift controls, and `min_density_reward` default `0.001`. |
| 2026-06-02 | 4 | Docker `colcon build --symlink-install` plus `ros2 launch my_tb3_world coral_g_full_demo.launch.py --show-args` | passed | Launch args include `use_rviz`, initial pose, `min_density_reward`, and forwarded CORAL-G demo controls. |
| 2026-06-02 | 5 | `command -v ros2`, `command -v gazebo`, `command -v gz`, `command -v rviz2` | skipped optional Gazebo | All checks returned no executable path. |

## Handoff Points

| Date | Trigger | Chunk | Worktree Closeout Decision | Worktree / Branch | Dev Server / Preview | Closeout Result | Fresh-Chat Prompt | Next Chunk Window |
|---|---|---|---|---|---|---|---|---|
| 2026-06-02 | Planning ledger created; old tracker superseded; next task changed | 1. Full-integration delivery plan and worktree setup | skip | `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G` on `full-integration-planning` | skipped; no dev server applies to docs/planning chunk | skipped; current checkout retained as planning worktree | [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md) | Chunk 1 needs human sign-off; next Chunk 2 planner tuning/launch/status graph. |
| 2026-06-02 | Chunk 2 verified; next task changed | 2. Planner tuning, launch parameter surface, and visual status graph | skip | `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G` on `full-integration-planning` | skipped; no dev server started; dashboard UI unchanged | skipped; current checkout retained | [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md) | Chunk 3 queued: local mission lifecycle closure with mocked Nav2. |
| 2026-06-02 | Chunk 3 verified; next task changed | 3. Local mission lifecycle closure with mocked Nav2 | skip | `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G` on `full-integration-planning` | skipped; no dev server started | skipped; current checkout retained | [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md) | Chunk 4 queued: Nav2 active-goal protection and repeatable demo launch. |
| 2026-06-02 | Chunk 4 verified; next task changed | 4. Nav2 active-goal protection and repeatable demo launch | skip | `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G` on `full-integration-planning` | skipped; no dev server started | skipped; current checkout retained | [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md) | Chunk 5 queued: local max-out and optional local Gazebo/Nav2 attempt. |
| 2026-06-02 | Chunk 5 verified; partner gate reached | 5. Local max-out and optional local Gazebo/Nav2 attempt | skip | `/Users/olafbobryk/Documents/Tue/CBL/Digital Twin/CORAL-G` on `full-integration-planning` | skipped; no dev server started | skipped; current checkout retained | [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md) | Chunk 6 needs_human: partner Gazebo/RViz pre-lab run. |

## Next Handoff

- Current chunk: 6. Partner Gazebo/RViz pre-lab package
- Current status: `needs_human`
- Rolling chunk window: Chunks 6-7 above
- Worktree closeout decision: `skip`
- Worktree status: current checkout retained on `full-integration-planning`
- Dev server status: skipped; no dev server applies
- Fresh-chat prompt: see
  [full-integration-handoff.md](/Users/olafbobryk/Documents/Tue/CBL/Digital%20Twin/CORAL-G/docs/full-integration-handoff.md)
- Next task: run the prepared repo in a partner Gazebo/RViz/Nav2 environment using `docs/coral-g-full-demo-runbook.md`, then record observations and failure category
- Stop conditions: schema/topic migration, dashboard/debug mission dependency,
  mandatory local Gazebo/Nav2 gate, or product-spine drift
