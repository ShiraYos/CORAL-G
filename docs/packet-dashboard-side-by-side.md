# Packet: Dashboard Side By Side

Goal:
Make the debug dashboard show physical truth and digital prediction as separate
surfaces, without changing ROS mission contracts or planner behavior.

Product spine:
The team can visually compare hidden physical debris particles against the
planner-facing predicted density field without confusing particles, density
cells, or obsolete clusters.

Outcome:
- The dashboard has a left physical-truth canvas and a right digital-prediction
  canvas.
- Both canvases share map bounds so positions are comparable.
- Physical truth shows debug-only particles, collection state, washed-out state,
  and robot position.
- Digital prediction shows normalized density cells and predicted washed-out
  counts.
- Digital prediction also overlays simulated prediction particles from the
  debug-only `/prediction_dashboard` topic. The heatmap remains the true
  planner-facing output.
- Physical particle samples are listed under the physical canvas.
- Recent digital prediction events are listed under the digital prediction
  canvas.
- Layer toggles can hide/show map, current force vectors, physical particles,
  prediction density, and robot position.
- The existing restart button remains lab/debug-only.

Constraints:
- No ROS topic/schema changes.
- No planner behavior changes.
- Do not expose physical particles outside `/dashboard`.
- Do not put prediction particles into `/debris_density_map`; use only the
  debug-only `/prediction_dashboard` surface.
- Do not reintroduce clusters as a model concept.
- No Gazebo marker work and no standalone renderer app in this packet.

Verification:
- `python3 -m py_compile src/my_tb3_world/my_tb3_world/*.py src/my_tb3_world/launch/*.py tools/*.py`
- `PYTHONPATH=src/my_tb3_world python3 -m unittest src/my_tb3_world/test/test_my_tb3_world_nodes.py -v`
- Docker `colcon build --symlink-install`
- Browser preview confirms the side-by-side canvases, toggles, density stats,
  physical particle list, digital event list, physical counts, and restart
  button render.
