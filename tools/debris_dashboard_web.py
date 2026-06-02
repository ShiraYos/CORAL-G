#!/usr/bin/env python3

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


DEMO_ODOM_POINTS = [
    ('water_test_position', 0.3, 0.9),
]
PLANNER_STALE_AFTER_SEC = 6.0

HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CORAL-G Debris Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f4f7f6;
      color: #17201d;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; }
    main {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      padding: 10px 8px;
      max-width: none;
      margin: 0 auto;
    }
    h1 { margin: 0; font-size: 22px; letter-spacing: 0; }
    h2 { margin: 0 0 10px; font-size: 14px; letter-spacing: 0; color: #31413c; }
    .top { grid-column: 1 / -1; display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    .topActions { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
    .badge { border: 1px solid #b8c9c1; border-radius: 999px; padding: 6px 10px; font-size: 13px; background: white; }
    button { border: 1px solid #b8c9c1; border-radius: 6px; padding: 7px 10px; background: #ffffff; color: #17201d; font: inherit; cursor: pointer; }
    button:hover { background: #edf4f1; }
    button:disabled { cursor: progress; color: #66756f; }
    .surface, .panel {
      background: white;
      border: 1px solid #d7e0dc;
      border-radius: 8px;
      box-shadow: 0 8px 24px rgba(18, 38, 31, 0.08);
    }
    .surface { padding: 8px; }
    .panel { padding: 12px; }
    .overview { grid-column: 1 / -1; display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; }
    .comparison { grid-column: 1 / -1; display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
    .surfaceHeader { display: flex; align-items: flex-start; justify-content: space-between; gap: 10px; margin-bottom: 10px; flex-wrap: wrap; }
    .surfaceHeader h2 { margin: 0; }
    canvas { width: 100%; aspect-ratio: 1 / 1; display: block; border-radius: 6px; background: #fbfdfc; }
    .pieCanvas { width: 168px; height: 168px; aspect-ratio: 1 / 1; background: transparent; }
    .distributionRow { display: grid; grid-template-columns: 180px 1fr; gap: 14px; align-items: center; margin-top: 12px; padding-top: 12px; border-top: 1px solid #edf2f0; }
    .distributionList { display: grid; gap: 6px; font-size: 13px; color: #31413c; }
    .distLine { display: grid; grid-template-columns: 12px 1fr auto; gap: 7px; align-items: center; }
    .canvasList { margin-top: 12px; border-top: 1px solid #edf2f0; padding-top: 10px; }
    .canvasList h3 { margin: 0 0 6px; font-size: 12px; color: #31413c; letter-spacing: 0; }
    .tableScroll { max-height: 210px; overflow: auto; border: 1px solid #edf2f0; border-radius: 6px; }
    table { width: 100%; border-collapse: collapse; font-size: 13px; }
    th, td { text-align: left; padding: 7px 4px; border-bottom: 1px solid #edf2f0; }
    th { color: #60736c; font-size: 11px; font-weight: 600; }
    .ok { color: #0f7b4f; }
    .warn { color: #9b5b12; }
    .legend { display: grid; grid-template-columns: 14px 1fr; gap: 7px; align-items: center; margin-top: 7px; font-size: 12px; color: #52645e; }
    .swatch { width: 14px; height: 14px; border-radius: 3px; border: 1px solid #c6d3cd; }
    .truth { background: #c97036; }
    .prediction { background: #2e5fd0; }
    .density { background: rgba(46, 95, 208, 0.45); }
    .current { background: #1d7285; }
    .wind { background: #6c5ce7; }
    .wave { background: #00a6a6; }
    .sum { background: #17201d; }
    .planner { background: #b0265b; }
    .unknown { background: #9aa6ad; }
    .plastic { background: #2e8f6f; }
    .wood { background: #8b6f3e; }
    .metal { background: #5b6670; }
    .distribution { margin-top: 8px; font-size: 12px; color: #31413c; line-height: 1.45; }
    .distribution strong { font-weight: 700; }
    .toggles { display: grid; grid-template-columns: 1fr 1fr; gap: 8px 12px; margin-top: 10px; }
    .toggle { display: flex; align-items: center; gap: 7px; color: #31413c; font-size: 13px; }
    .toggle input { width: 16px; height: 16px; accent-color: #1d7285; }
    .plannerGrid { display: grid; gap: 6px; font-size: 13px; color: #31413c; }
    .plannerLine { display: grid; grid-template-columns: 96px 1fr; gap: 8px; align-items: baseline; }
    .plannerLine span:first-child { color: #60736c; font-size: 12px; }
    .plannerComponents { margin-top: 10px; padding-top: 8px; border-top: 1px solid #edf2f0; }
    .note { color: #60736c; font-size: 12px; margin-top: 8px; line-height: 1.35; }
    .rulesPanel { grid-column: 1 / -1; }
    .rulesIntro { color: #52645e; font-size: 12px; line-height: 1.4; margin: 0 0 10px; }
    .rulesTable th, .rulesTable td { text-align: right; }
    .rulesTable th:first-child, .rulesTable td:first-child { text-align: left; }
    @media (max-width: 860px) {
      main { grid-template-columns: 1fr; padding: 10px 6px; }
      .top { align-items: flex-start; flex-direction: column; }
      .overview { grid-template-columns: 1fr; }
      .comparison { grid-template-columns: 1fr; }
    }
    @media (max-width: 520px) {
      main { padding: 10px; }
      .surfaceHeader { display: block; }
      .distributionRow { grid-template-columns: 1fr; }
      .tableScroll { max-height: 160px; }
    }
  </style>
</head>
<body>
<main>
  <div class="top">
    <h1>CORAL-G Debris Dashboard</h1>
    <div class="topActions">
      <button id="restartDebug" type="button">Restart Debug Sim</button>
      <div id="connection" class="badge">waiting for /dashboard</div>
    </div>
  </div>
  <aside class="overview">
    <section class="panel">
      <h2>Display</h2>
      <div class="toggles">
        <label class="toggle"><input id="layerMap" type="checkbox" checked>Map</label>
        <label class="toggle"><input id="layerPhysical" type="checkbox" checked>Physical particles</label>
        <label class="toggle"><input id="layerPredictionParticles" type="checkbox" checked>Prediction particles</label>
        <label class="toggle"><input id="layerDensity" type="checkbox" checked>Density</label>
        <label class="toggle"><input id="layerRobot" type="checkbox" checked>Robot</label>
        <label class="toggle"><input id="layerCurrent" type="checkbox" checked>Current</label>
        <label class="toggle"><input id="layerWind" type="checkbox" checked>Wind</label>
        <label class="toggle"><input id="layerWave" type="checkbox" checked>Wave</label>
        <label class="toggle"><input id="layerSum" type="checkbox" checked>Sum force</label>
        <label class="toggle"><input id="layerPlannerIntent" type="checkbox" checked>Planner intent</label>
      </div>
      <div class="note">Dashboard/debug only. These controls do not affect ROS mission topics.</div>
    </section>
    <section class="panel">
      <h2>Legend</h2>
      <div class="legend"><span class="swatch plastic"></span><span>plastic particles: high wind response</span></div>
      <div class="legend"><span class="swatch wood"></span><span>wood particles: medium current/wind response</span></div>
      <div class="legend"><span class="swatch metal"></span><span>metal particles: low wind response</span></div>
      <div class="legend"><span class="swatch unknown"></span><span>unknown: digital prior/fallback</span></div>
      <div class="legend"><span class="swatch density"></span><span>blue heatmap: predicted density cells</span></div>
      <div class="legend"><span class="swatch current"></span><span>current arrows: base flow, noise curl, eddies, shore redirect</span></div>
      <div class="legend"><span class="swatch wind"></span><span>wind arrows: surface drift</span></div>
      <div class="legend"><span class="swatch wave"></span><span>wave arrows: turbulence and tide-height signal</span></div>
      <div class="legend"><span class="swatch sum"></span><span>sum force arrows: combined local field</span></div>
      <div class="legend"><span class="swatch planner"></span><span>planner intent: selected /next_cell_goal</span></div>
      <div id="debrisDistribution" class="distribution">debris distribution: waiting for prediction dashboard</div>
      <div id="materialEvidence" class="note">material evidence: none</div>
    </section>
    <section class="panel">
      <h2>Robot</h2>
      <div id="robotPose">x=? y=?</div>
      <div id="robotStatus">pose: waiting</div>
    </section>
    <section class="panel">
      <h2>Planner</h2>
      <div class="plannerGrid">
        <div class="plannerLine"><span>mode</span><strong id="plannerMode">waiting</strong></div>
        <div class="plannerLine"><span>goal</span><strong id="plannerGoal">x=? y=? yaw=?</strong></div>
        <div class="plannerLine"><span>utility</span><strong id="plannerUtility">?</strong></div>
        <div class="plannerLine"><span>return</span><strong id="plannerReturn">?</strong></div>
        <div class="plannerLine"><span>reason</span><strong id="plannerReason">waiting for /next_cell_goal</strong></div>
        <div class="plannerLine"><span>updated</span><strong id="plannerUpdated">stale</strong></div>
      </div>
      <div id="plannerComponents" class="plannerGrid plannerComponents"></div>
      <div class="note">Debug-only mirror of planner intent. Not mission input.</div>
    </section>
  </aside>
  <div class="comparison">
    <section class="surface">
      <div class="surfaceHeader">
        <h2>Physical Truth</h2>
      </div>
      <canvas id="physicalArena" width="900" height="900"></canvas>
      <div class="distributionRow">
        <canvas id="physicalDistributionChart" class="pieCanvas" width="220" height="220"></canvas>
        <div>
          <h3>Physical Distribution</h3>
          <div id="physicalDistributionList" class="distributionList"></div>
        </div>
      </div>
      <div class="canvasList">
        <h3>Physical Events</h3>
        <div class="tableScroll">
          <table>
            <thead><tr><th>Event</th><th>Material</th><th>Status</th></tr></thead>
            <tbody id="physicalEvents"></tbody>
          </table>
        </div>
      </div>
    </section>
    <section class="surface">
      <div class="surfaceHeader">
        <h2>Digital Prediction</h2>
      </div>
      <canvas id="predictionArena" width="900" height="900"></canvas>
      <div class="distributionRow">
        <canvas id="digitalDistributionChart" class="pieCanvas" width="220" height="220"></canvas>
        <div>
          <h3>Digital Prediction Distribution</h3>
          <div id="digitalDistributionList" class="distributionList"></div>
        </div>
      </div>
      <div class="canvasList">
        <h3>Digital Events</h3>
        <div class="tableScroll">
          <table>
            <thead><tr><th>Event</th><th>Material</th><th>Status</th></tr></thead>
            <tbody id="digitalEvents"></tbody>
          </table>
        </div>
      </div>
    </section>
  </div>
  <section class="panel rulesPanel">
    <h2>Debris Motion Rules</h2>
    <p class="rulesIntro">
      Particle acceleration combines local current, wind, wave jitter, and boid-style cohesion/separation. Material coefficients below multiply each force layer; higher values mean that material responds more strongly to that layer.
    </p>
    <div class="tableScroll">
      <table class="rulesTable">
        <thead>
          <tr>
            <th>Material</th>
            <th>Current</th>
            <th>Wind</th>
            <th>Wave</th>
            <th>Cohesion</th>
            <th>Separation</th>
          </tr>
        </thead>
        <tbody id="materialRules"></tbody>
      </table>
    </div>
    <div class="canvasList">
      <h3>Boid Rules</h3>
      <div class="tableScroll">
        <table>
          <thead><tr><th>Rule</th><th>Value</th><th>Effect</th></tr></thead>
          <tbody id="boidRules"></tbody>
        </table>
      </div>
    </div>
  </section>
</main>
<script>
const views = {
  physical: {
    canvas: document.getElementById('physicalArena'),
    ctx: document.getElementById('physicalArena').getContext('2d')
  },
  prediction: {
    canvas: document.getElementById('predictionArena'),
    ctx: document.getElementById('predictionArena').getContext('2d')
  }
};
let latestPayload = null;

function layerState() {
  return {
    map: document.getElementById('layerMap').checked,
    physical: document.getElementById('layerPhysical').checked,
    predictionParticles: document.getElementById('layerPredictionParticles').checked,
    density: document.getElementById('layerDensity').checked,
    robot: document.getElementById('layerRobot').checked,
    current: document.getElementById('layerCurrent').checked,
    wind: document.getElementById('layerWind').checked,
    wave: document.getElementById('layerWave').checked,
    sum: document.getElementById('layerSum').checked,
    plannerIntent: document.getElementById('layerPlannerIntent').checked,
  };
}

function plannerIntent(payload) {
  return payload.planner_intent || {};
}

function plannerGoal(intent) {
  const goal = intent.goal || {};
  if (
    intent.mode === 'idle' ||
    goal.frame_id !== 'map' ||
    !Number.isFinite(goal.x) ||
    !Number.isFinite(goal.y)
  ) {
    return null;
  }
  return goal;
}

function worldBounds(payload) {
  const points = [];
  const cellSize = ((payload.environment || {}).cell_size_m || 0.5);
  const half = cellSize / 2;
  for (const cell of ((payload.environment || {}).cells || [])) {
    points.push([cell.x - half, cell.y - half], [cell.x + half, cell.y + half]);
  }
  for (const particle of physicalParticles(payload)) points.push([particle.x, particle.y]);
  for (const cell of ((((payload.prediction || {}).density_cells) || ((payload.prediction || {}).active_cells)) || [])) {
    points.push([cell.x, cell.y]);
    if (Number.isFinite(cell.cluster_x) && Number.isFinite(cell.cluster_y)) {
      points.push([cell.cluster_x, cell.cluster_y]);
    }
  }
  for (const particle of (((payload.prediction || {}).prediction_particles) || [])) {
    points.push([particle.x, particle.y]);
  }
  const pose = (payload.robot || {}).pose || {};
  if (Number.isFinite(pose.x) && Number.isFinite(pose.y)) points.push([pose.x, pose.y]);
  const goal = plannerGoal(plannerIntent(payload));
  if (goal) points.push([goal.x, goal.y]);
  if (!points.length) return { minX: -2, minY: -2, maxX: 2, maxY: 2 };
  const xs = points.map(([x]) => x);
  const ys = points.map(([, y]) => y);
  const minX = Math.min(...xs) - 0.6;
  const maxX = Math.max(...xs) + 0.6;
  const minY = Math.min(...ys) - 0.6;
  const maxY = Math.max(...ys) + 0.6;
  const span = Math.max(maxX - minX, maxY - minY, 1);
  const cx = (minX + maxX) / 2;
  const cy = (minY + maxY) / 2;
  return {
    minX: cx - span / 2,
    maxX: cx + span / 2,
    minY: cy - span / 2,
    maxY: cy + span / 2
  };
}

function worldToCanvas(view, bounds, x, y) {
  const pad = 24;
  const size = view.canvas.width - pad * 2;
  return [
    pad + ((x - bounds.minX) / (bounds.maxX - bounds.minX)) * size,
    pad + ((bounds.maxY - y) / (bounds.maxY - bounds.minY)) * size
  ];
}

function worldLengthToCanvas(view, bounds, length) {
  const pad = 24;
  const size = view.canvas.width - pad * 2;
  return length * size / (bounds.maxX - bounds.minX);
}

function physicalParticles(payload) {
  if (Array.isArray(payload.physical_particles) && payload.physical_particles.length) {
    return payload.physical_particles;
  }
  return payload.clusters || [];
}

function cellRect(view, bounds, cell, cellSize) {
  const half = cellSize / 2;
  const [left, top] = worldToCanvas(view, bounds, cell.x - half, cell.y + half);
  const [right, bottom] = worldToCanvas(view, bounds, cell.x + half, cell.y - half);
  return {
    x: left,
    y: top,
    width: right - left,
    height: bottom - top,
  };
}

const MATERIAL_COLORS = {
  unknown: '#9aa6ad',
  plastic: '#2e8f6f',
  wood: '#8b6f3e',
  metal: '#5b6670'
};

function materialColor(material) {
  return MATERIAL_COLORS[material] || MATERIAL_COLORS.unknown;
}

function drawArrow(view, bounds, x, y, vx, vy, cellSize, color, widthScale = 1) {
  if (!Number.isFinite(vx) || !Number.isFinite(vy) || Math.hypot(vx, vy) <= 0.001) return;
  const { ctx } = view;
  const [cx, cy] = worldToCanvas(view, bounds, x, y);
  const cellPixels = worldLengthToCanvas(view, bounds, cellSize);
  const scale = Math.min(cellPixels * 0.95, 46);
  const sx = cx;
  const sy = cy;
  const ex = sx + vx * scale;
  const ey = sy - vy * scale;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = Math.max(1.3, Math.min(2.8, cellPixels * 0.075)) * widthScale;
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  const angle = Math.atan2(ey - sy, ex - sx);
  const head = Math.max(5, Math.min(9, cellPixels * 0.16));
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - head * Math.cos(angle - 0.5), ey - head * Math.sin(angle - 0.5));
  ctx.lineTo(ex - head * Math.cos(angle + 0.5), ey - head * Math.sin(angle + 0.5));
  ctx.closePath();
  ctx.fill();
}

function drawMapAndForce(view, payload, bounds, layers) {
  const { ctx } = view;
  const env = payload.environment || {};
  const cells = env.cells || [];
  const cellSize = env.cell_size_m || 0.5;
  for (const cell of cells) {
    const rect = cellRect(view, bounds, cell, cellSize);
    if (layers.map) {
      ctx.fillStyle = cell.occupancy === 'blocked' ? '#b84f4b' : cell.occupancy === 'unknown' ? '#c5cbd0' : '#e7f5ef';
      ctx.strokeStyle = '#d6e2dd';
      ctx.lineWidth = 1.5;
      ctx.fillRect(rect.x, rect.y, rect.width, rect.height);
      ctx.strokeRect(rect.x, rect.y, rect.width, rect.height);
    }
    if ((layers.current || layers.wind || layers.wave || layers.sum) && cell.has_environment) {
      ctx.save();
      ctx.beginPath();
      ctx.rect(rect.x, rect.y, rect.width, rect.height);
      ctx.clip();
      if (layers.current) {
        drawArrow(view, bounds, cell.x, cell.y, cell.current_x || 0, cell.current_y || 0, cellSize, '#1d7285');
      }
      if (layers.wind) {
        drawArrow(view, bounds, cell.x, cell.y, cell.wind_x || 0, cell.wind_y || 0, cellSize, '#6c5ce7');
      }
      const wave = cell.wave_height || 0;
      const waveDir = ((Math.round((cell.x + cell.y) * 10) % 2) === 0) ? 1 : -1;
      const waveX = 0.12 * waveDir;
      const waveY = wave * 0.55;
      if (layers.wave) {
        drawArrow(view, bounds, cell.x, cell.y, waveX, waveY, cellSize, '#00a6a6');
      }
      if (layers.sum) {
        drawArrow(
          view,
          bounds,
          cell.x,
          cell.y,
          (cell.current_x || 0) + (cell.wind_x || 0) + waveX,
          (cell.current_y || 0) + (cell.wind_y || 0) + waveY,
          cellSize,
          '#17201d',
          1.25
        );
      }
      ctx.restore();
    }
  }
}

function drawPredictionDensity(view, payload, bounds, layers) {
  if (!layers.density) return;
  const { ctx } = view;
  const prediction = payload.prediction || {};
  const activeCells = prediction.density_cells || prediction.active_cells || [];
  const cellSize = prediction.cell_size_m || ((payload.environment || {}).cell_size_m || 0.5);
  for (const cell of activeCells) {
    const rect = cellRect(view, bounds, cell, cellSize);
    const density = Math.max(0, Math.min(1, cell.density || 0));
    ctx.fillStyle = `rgba(46, 95, 208, ${density > 0 ? 0.12 + Math.min(0.68, density * 12) : 0.035})`;
    ctx.fillRect(rect.x + 1, rect.y + 1, Math.max(0, rect.width - 2), Math.max(0, rect.height - 2));
    ctx.strokeStyle = density > 0 ? 'rgba(46, 95, 208, 0.72)' : 'rgba(46, 95, 208, 0.12)';
    ctx.lineWidth = density > 0 ? 1.5 : 1.0;
    ctx.strokeRect(rect.x + 1, rect.y + 1, Math.max(0, rect.width - 2), Math.max(0, rect.height - 2));
  }
}

function drawPredictionParticles(view, payload, bounds, layers) {
  if (!layers.predictionParticles) return;
  const { ctx } = view;
  const particles = ((payload.prediction || {}).prediction_particles) || [];
  for (const particle of particles) {
    const [cx, cy] = worldToCanvas(view, bounds, particle.x, particle.y);
    ctx.beginPath();
    ctx.arc(cx, cy, particle.status === 'active' ? 3.8 : 2.8, 0, Math.PI * 2);
    ctx.fillStyle = particle.status === 'active'
      ? materialColor(particle.material)
      : particle.status === 'washed_out'
        ? 'rgba(91, 103, 112, 0.56)'
        : 'rgba(45, 138, 95, 0.62)';
    ctx.fill();
  }
}

function drawPhysicalParticles(view, payload, bounds, layers) {
  if (!layers.physical) return;
  const { ctx } = view;
  for (const particle of physicalParticles(payload)) {
    const [cx, cy] = worldToCanvas(view, bounds, particle.x, particle.y);
    ctx.beginPath();
    ctx.arc(cx, cy, particle.status === 'active' ? 4.2 : 3.2, 0, Math.PI * 2);
    ctx.fillStyle = particle.status === 'collected'
      ? '#2d8a5f'
      : particle.status === 'washed_out'
        ? '#6d7880'
        : materialColor(particle.material);
    ctx.fill();
  }
}

function drawRobot(view, payload, bounds, layers) {
  if (!layers.robot) return;
  const { ctx } = view;
  const pose = (payload.robot || {}).pose || {};
  if (Number.isFinite(pose.x) && Number.isFinite(pose.y)) {
    const [rx, ry] = worldToCanvas(view, bounds, pose.x, pose.y);
    ctx.beginPath();
    ctx.arc(rx, ry, 18, 0, Math.PI * 2);
    ctx.fillStyle = '#2e5fd0';
    ctx.fill();
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 4;
    ctx.stroke();
  }
}

function drawPlannerIntent(view, payload, bounds, layers) {
  if (!layers.plannerIntent) return;
  const intent = plannerIntent(payload);
  const goal = plannerGoal(intent);
  if (!goal) return;
  const { ctx } = view;
  const [gx, gy] = worldToCanvas(view, bounds, goal.x, goal.y);
  const pose = (payload.robot || {}).pose || {};
  if (Number.isFinite(pose.x) && Number.isFinite(pose.y)) {
    const [rx, ry] = worldToCanvas(view, bounds, pose.x, pose.y);
    ctx.save();
    ctx.strokeStyle = intent.stale ? 'rgba(176, 38, 91, 0.38)' : 'rgba(176, 38, 91, 0.82)';
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = 2.2;
    ctx.setLineDash([8, 6]);
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(gx, gy);
    ctx.stroke();
    ctx.setLineDash([]);
    const angle = Math.atan2(gy - ry, gx - rx);
    const head = 12;
    ctx.beginPath();
    ctx.moveTo(gx, gy);
    ctx.lineTo(gx - head * Math.cos(angle - 0.45), gy - head * Math.sin(angle - 0.45));
    ctx.lineTo(gx - head * Math.cos(angle + 0.45), gy - head * Math.sin(angle + 0.45));
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
  ctx.save();
  ctx.strokeStyle = '#b0265b';
  ctx.fillStyle = '#ffffff';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(gx, gy, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(gx - 14, gy);
  ctx.lineTo(gx + 14, gy);
  ctx.moveTo(gx, gy - 14);
  ctx.lineTo(gx, gy + 14);
  ctx.stroke();
  ctx.restore();
}

function drawView(view, payload, bounds, layers, mode) {
  const { canvas, ctx } = view;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawMapAndForce(view, payload, bounds, layers);
  if (mode === 'physical') {
    drawPhysicalParticles(view, payload, bounds, layers);
  } else {
    drawPredictionDensity(view, payload, bounds, layers);
    drawPredictionParticles(view, payload, bounds, layers);
  }
  drawPlannerIntent(view, payload, bounds, layers);
  drawRobot(view, payload, bounds, layers);
}

function draw(payload) {
  const bounds = worldBounds(payload);
  const layers = layerState();
  drawView(views.physical, payload, bounds, layers, 'physical');
  drawView(views.prediction, payload, bounds, layers, 'prediction');
}

function physicalDistribution(payload) {
  const materialCounts = (((payload.physical_debris || {}).material_counts) || {});
  const distribution = {};
  for (const name of ['plastic', 'wood', 'metal', 'unknown']) {
    distribution[name] = (materialCounts[name] || {}).active || 0;
  }
  return distribution;
}

function digitalDistribution(payload) {
  const probabilities = ((((payload.prediction || {}).material_belief || {}).probabilities) || {});
  const distribution = {};
  for (const name of ['unknown', 'plastic', 'wood', 'metal']) {
    distribution[name] = probabilities[name] || 0;
  }
  return distribution;
}

function drawPie(canvasId, distribution, mode) {
  const canvas = document.getElementById(canvasId);
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const entries = Object.entries(distribution).filter(([, value]) => value > 0);
  const total = entries.reduce((sum, [, value]) => sum + value, 0);
  const cx = canvas.width / 2;
  const cy = canvas.height / 2;
  const radius = Math.min(canvas.width, canvas.height) * 0.38;
  if (total <= 0) {
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.strokeStyle = '#c6d3cd';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#60736c';
    ctx.font = '16px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('waiting', cx, cy + 5);
    return;
  }
  let start = -Math.PI / 2;
  for (const [name, value] of entries) {
    const angle = (value / total) * Math.PI * 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, radius, start, start + angle);
    ctx.closePath();
    ctx.fillStyle = materialColor(name);
    ctx.fill();
    start += angle;
  }
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 3;
  ctx.stroke();
  ctx.fillStyle = '#17201d';
  ctx.font = '14px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText(mode, cx, cy + 5);
}

function renderDistributionList(elementId, distribution, suffix) {
  const element = document.getElementById(elementId);
  const total = Object.values(distribution).reduce((sum, value) => sum + value, 0);
  element.innerHTML = ['unknown', 'plastic', 'wood', 'metal']
    .filter(name => distribution[name] !== undefined)
    .map(name => {
      const value = distribution[name] || 0;
      const display = suffix === '%' ? `${Math.round(value * 100)}%` : String(value);
      return `
        <div class="distLine">
          <span class="swatch ${name}"></span>
          <span>${name}</span>
          <span>${total > 0 ? display : '0'}</span>
        </div>
      `;
    })
    .join('');
}

function renderMaterialRules(payload) {
  const physical = payload.physical_debris || {};
  const response = physical.material_response || {};
  const rules = physical.boid_rules || {};
  document.getElementById('materialRules').innerHTML = ['plastic', 'wood', 'metal', 'unknown']
    .filter(name => response[name])
    .map(name => {
      const material = response[name] || {};
      return `
        <tr>
          <td>${name}</td>
          <td>${material.current ?? '-'}</td>
          <td>${material.wind ?? '-'}</td>
          <td>${material.wave ?? '-'}</td>
          <td>${material.cohesion ?? '-'}</td>
          <td>${material.separation ?? '-'}</td>
        </tr>
      `;
    })
    .join('');
  const boidRows = [
    ['neighbor radius', `${rules.neighbor_radius_m ?? '?'}m`, 'distance for cohesion/separation neighbors'],
    ['separation scale', rules.separation_scale ?? '?', 'strength of close-particle repulsion'],
    ['water damping', rules.water_damping ?? '?', 'fraction of previous velocity removed per tick'],
    ['wave jitter scale', rules.wave_jitter_scale ?? '?', 'sideways/randomized wave perturbation strength'],
  ];
  document.getElementById('boidRules').innerHTML = boidRows.map(row => `
    <tr>
      <td>${row[0]}</td>
      <td>${row[1]}</td>
      <td>${row[2]}</td>
    </tr>
  `).join('');
}

function formatNumber(value, digits = 3) {
  return Number.isFinite(value) ? String(Math.round(value * (10 ** digits)) / (10 ** digits)) : '?';
}

function renderPlannerIntent(payload) {
  const intent = plannerIntent(payload);
  const goal = plannerGoal(intent);
  const mode = intent.mode || 'waiting';
  const status = intent.status || (intent.stale ? 'stale' : 'waiting');
  document.getElementById('plannerMode').textContent = `${mode} / ${status}`;
  document.getElementById('plannerGoal').textContent = goal
    ? `x=${formatNumber(goal.x)} y=${formatNumber(goal.y)} yaw=${formatNumber(goal.yaw || 0)}`
    : 'no map-frame goal';
  document.getElementById('plannerUtility').textContent = formatNumber(intent.utility);
  document.getElementById('plannerReturn').textContent = intent.return_feasible === undefined
    ? '?'
    : String(Boolean(intent.return_feasible));
  document.getElementById('plannerReason').textContent = intent.reason || 'waiting for /next_cell_goal';
  const age = Number.isFinite(intent.age_sec) ? `${formatNumber(intent.age_sec, 1)}s old` : 'no update';
  document.getElementById('plannerUpdated').textContent = `${intent.stale ? 'stale' : 'fresh'} (${age})`;
  const components = intent.components || {};
  const rows = [
    ['density', components.density_reward],
    ['travel', components.travel_cost],
    ['fuel', components.fuel_penalty],
    ['storage', components.storage_penalty],
    ['map risk', components.map_risk],
    ['return', components.return_cost],
    ['fuel margin', components.fuel_margin],
  ];
  document.getElementById('plannerComponents').innerHTML = rows
    .filter(([, value]) => value !== undefined)
    .map(([label, value]) => `
      <div class="plannerLine"><span>${label}</span><strong>${formatNumber(value)}</strong></div>
    `)
    .join('');
}

function updateText(payload) {
  const robot = payload.robot || {};
  const pose = robot.pose || {};
  document.getElementById('connection').textContent = `${payload.status || 'unknown'} | ${payload.stamp || ''}`;
  const prediction = payload.prediction || {};
  const materialBelief = prediction.material_belief || {};
  const probabilities = materialBelief.probabilities || {};
  const evidence = materialBelief.evidence || {};
  const distributionText = ['unknown', 'plastic', 'wood', 'metal']
    .filter(name => probabilities[name] !== undefined)
    .map(name => `${name} ${Math.round((probabilities[name] || 0) * 100)}%`)
    .join(' | ');
  document.getElementById('debrisDistribution').innerHTML = distributionText
    ? `<strong>debris distribution:</strong> ${distributionText}`
    : 'debris distribution: waiting for prediction dashboard';
  const evidenceText = ['plastic', 'wood', 'metal']
    .filter(name => evidence[name] !== undefined)
    .map(name => `${name} ${evidence[name] || 0}`)
    .join(' | ');
  document.getElementById('materialEvidence').textContent = evidenceText
    ? `material evidence: ${evidenceText}`
    : 'material evidence: none';
  document.getElementById('robotPose').textContent = `x=${pose.x ?? '?'} y=${pose.y ?? '?'}`;
  document.getElementById('robotStatus').textContent = `pose: ${robot.pose_received ? 'received' : 'waiting'}`;
  renderPlannerIntent(payload);

  const physicalDist = physicalDistribution(payload);
  const digitalDist = digitalDistribution(payload);
  drawPie('physicalDistributionChart', physicalDist, 'truth');
  drawPie('digitalDistributionChart', digitalDist, 'belief');
  renderDistributionList('physicalDistributionList', physicalDist, 'count');
  renderDistributionList('digitalDistributionList', digitalDist, '%');
  renderMaterialRules(payload);

  document.getElementById('physicalEvents').innerHTML = ((payload.physical_events || []).slice().reverse()).map(event => `
    <tr>
      <td>${event.type || '?'}</td>
      <td>${event.material || '?'}</td>
      <td class="${event.status === 'collected' ? 'ok' : 'warn'}">${event.status || '?'}</td>
    </tr>
  `).join('');
  document.getElementById('digitalEvents').innerHTML = ((prediction.events || []).slice().reverse()).map(event => {
    return `
      <tr>
        <td>${event.type || '?'}</td>
        <td>${event.material || '-'}</td>
        <td>${event.type || '?'}</td>
      </tr>
    `;
  }).join('');
}

async function poll() {
  try {
    const res = await fetch('/api/dashboard', { cache: 'no-store' });
    const payload = await res.json();
    if (payload && payload.schema) {
      latestPayload = payload;
      updateText(payload);
      draw(payload);
    }
  } catch (err) {
    document.getElementById('connection').textContent = 'dashboard server unavailable';
  }
}
async function restartDebug() {
  const button = document.getElementById('restartDebug');
  button.disabled = true;
  button.textContent = 'Restarting...';
  try {
    await fetch('/api/debug/restart', { method: 'POST' });
  } finally {
    setTimeout(() => {
      button.disabled = false;
      button.textContent = 'Restart Debug Sim';
    }, 900);
  }
}
document.getElementById('restartDebug').addEventListener('click', restartDebug);
for (const id of ['layerMap', 'layerPhysical', 'layerPredictionParticles', 'layerDensity', 'layerRobot', 'layerCurrent', 'layerWind', 'layerWave', 'layerSum', 'layerPlannerIntent']) {
  document.getElementById(id).addEventListener('change', () => {
    if (latestPayload) draw(latestPayload);
  });
}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
"""


class DashboardState:
    def __init__(self):
        self.lock = threading.Lock()
        self.planner_intent = {
            'debug_only': True,
            'forbidden_as_mission_input': True,
            'status': 'waiting_for_next_cell_goal',
            'mode': 'waiting',
            'stale': True,
        }
        self.planner_received_monotonic = None
        self.payload = {
            'schema': 'dtas.dashboard.v1',
            'status': 'waiting_for_dashboard',
            'clusters': [],
            'physical_particles': [],
            'environment': {'cells': []},
            'robot': {'pose': {}},
            'prediction': {},
            'planner_intent': self.planner_intent,
        }
        self.prediction = {}
        self.restart_requested = False

    def set_payload(self, payload):
        with self.lock:
            if self.prediction:
                payload['prediction'] = self.prediction
            payload['planner_intent'] = self._planner_snapshot_locked()
            self.payload = payload

    def set_prediction(self, payload):
        density_cells = payload.get('density_cells', payload.get('cells', []))
        active_cells = [
            cell for cell in density_cells
            if cell.get('density', 0.0) > 0.0
        ]
        density_sum = round(sum(cell.get('density', 0.0) for cell in active_cells), 6)
        prediction = {
            'schema': payload.get('schema'),
            'source': payload.get('source'),
            'stamp': payload.get('stamp'),
            'status': payload.get('status'),
            'cell_size_m': payload.get('cell_size_m'),
            'simulation_horizon_sec': payload.get('simulation_horizon_sec'),
            'cell_count': len(density_cells),
            'density_cells': density_cells,
            'active_cells': active_cells,
            'density_sum': density_sum,
            'prediction_particle_count': payload.get('prediction_particle_count'),
            'prediction_counts': payload.get('prediction_counts'),
            'clusters_remaining': payload.get('clusters_remaining'),
            'prediction_drift_enabled': payload.get('prediction_drift_enabled'),
            'prediction_drift_scale': payload.get('prediction_drift_scale'),
        }
        with self.lock:
            prediction.update({
                key: value
                for key, value in self.prediction.items()
                if key in (
                    'prediction_particles',
                    'prediction_dashboard_schema',
                    'prediction_lifetime_counts',
                    'material_belief',
                    'events',
                )
            })
            self.prediction = prediction
            self.payload['prediction'] = prediction

    def set_prediction_dashboard(self, payload):
        prediction_particles = payload.get('prediction_particles', [])
        with self.lock:
            prediction = dict(self.prediction)
            prediction.update({
                'prediction_dashboard_schema': payload.get('schema'),
                'prediction_particles': prediction_particles,
                'prediction_particle_count': payload.get(
                    'prediction_particle_count',
                    prediction.get('prediction_particle_count'),
                ),
                'prediction_counts': payload.get(
                    'prediction_counts',
                    prediction.get('prediction_counts'),
                ),
                'prediction_lifetime_counts': payload.get(
                    'lifetime_counts',
                    prediction.get('prediction_lifetime_counts'),
                ),
                'events': payload.get('events', prediction.get('events', [])),
                'material_belief': payload.get(
                    'material_belief',
                    prediction.get('material_belief', {}),
                ),
            })
            self.prediction = prediction
            self.payload['prediction'] = prediction

    def set_planner_intent(self, payload):
        intent = dict(payload)
        intent['debug_only'] = True
        intent['forbidden_as_mission_input'] = True
        with self.lock:
            self.planner_intent = intent
            self.planner_received_monotonic = time.monotonic()
            self.payload['planner_intent'] = self._planner_snapshot_locked()

    def _planner_snapshot_locked(self):
        intent = dict(self.planner_intent)
        if self.planner_received_monotonic is None:
            intent['age_sec'] = None
            intent['stale'] = True
            return intent

        age = time.monotonic() - self.planner_received_monotonic
        intent['age_sec'] = round(age, 1)
        intent['stale_after_sec'] = PLANNER_STALE_AFTER_SEC
        intent['stale'] = age > PLANNER_STALE_AFTER_SEC
        return intent

    def get_payload(self):
        with self.lock:
            payload = dict(self.payload)
            payload['planner_intent'] = self._planner_snapshot_locked()
            return payload

    def request_restart(self):
        with self.lock:
            self.restart_requested = True

    def consume_restart_request(self):
        with self.lock:
            requested = self.restart_requested
            self.restart_requested = False
            return requested


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                body = HTML.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == '/api/dashboard':
                body = json.dumps(state.get_payload()).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            self.send_response(404)
            self.end_headers()

        def do_POST(self):
            if self.path == '/api/debug/restart':
                state.request_restart()
                body = json.dumps({'ok': True, 'scope': 'all'}).encode('utf-8')
                self.send_response(202)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            self.send_response(404)
            self.end_headers()

        def log_message(self, _format, *_args):
            return

    return Handler


def main():
    parser = argparse.ArgumentParser(description='Serve a debug web view of /dashboard.')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--demo-odom', action='store_true')
    parser.add_argument('--interval-sec', type=float, default=2.0)
    args = parser.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    state = DashboardState()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'Dashboard web preview: http://{args.host}:{args.port}', flush=True)

    rclpy.init()
    node = rclpy.create_node('debris_dashboard_web')
    debug_reset_pub = node.create_publisher(String, '/debug_reset', 10)
    odom_pub = None
    if args.demo_odom:
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        odom_pub = node.create_publisher(Odometry, '/odom', odom_qos)

    def dashboard_cb(msg):
        state.set_payload(json.loads(msg.data))

    def density_cb(msg):
        state.set_prediction(json.loads(msg.data))

    def prediction_dashboard_cb(msg):
        state.set_prediction_dashboard(json.loads(msg.data))

    def planner_intent_cb(msg):
        state.set_planner_intent(json.loads(msg.data))

    node.create_subscription(String, '/dashboard', dashboard_cb, 10)
    node.create_subscription(String, '/debris_density_map', density_cb, 10)
    node.create_subscription(String, '/prediction_dashboard', prediction_dashboard_cb, 10)
    planner_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(String, '/next_cell_goal', planner_intent_cb, planner_qos)

    current_started = 0.0
    last_publish = 0.0
    demo_index = 0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if state.consume_restart_request():
                reset_msg = String()
                reset_msg.data = json.dumps({
                    'schema': 'dtas.debug_reset.v1',
                    'stamp': time.time(),
                    'source': 'debris_dashboard_web',
                    'scope': 'all',
                })
                debug_reset_pub.publish(reset_msg)
            if not args.demo_odom or odom_pub is None:
                continue
            now = time.monotonic()
            hold_last = demo_index >= len(DEMO_ODOM_POINTS)
            label, x, y = DEMO_ODOM_POINTS[-1 if hold_last else demo_index]
            if current_started == 0.0 and not hold_last:
                current_started = now
                print(f'[demo_odom] {label}: x={x} y={y}', flush=True)

            if now - last_publish >= 0.2:
                odom = Odometry()
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                last_publish = now

            if not hold_last and now - current_started >= args.interval_sec:
                demo_index += 1
                current_started = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
