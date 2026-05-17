const state = {
  manifest: null,
  frame: null,
  frameIndex: 0,
  frameCache: new Map(),
  framePromiseCache: new Map(),
  frameRequestId: 0,
  selection: null,
  pickables: [],
  camera: {
    yaw: 0.55,
    pitch: 0.38,
    distance: 55,
    target: [0, 0, 0],
  },
  interaction: {
    active: false,
    mode: "orbit",
    lastX: 0,
    lastY: 0,
    moved: false,
  },
};

const canvas = document.getElementById("viewer-canvas");
const ctx = canvas.getContext("2d");
const frameSelect = document.getElementById("frame-select");
const frameSlider = document.getElementById("frame-slider");
const statsGrid = document.getElementById("stats-grid");
const selectionPanel = document.getElementById("selection-panel");
const datasetMeta = document.getElementById("dataset-meta");
const frameLabel = document.getElementById("frame-label");
const frameSummary = document.getElementById("frame-summary");
const resetCameraButton = document.getElementById("reset-camera");

const toggles = {
  cloud: document.getElementById("toggle-cloud"),
  detection: document.getElementById("toggle-detection"),
  groundtruth: document.getElementById("toggle-groundtruth"),
  tp: document.getElementById("toggle-tp"),
  fp: document.getElementById("toggle-fp"),
  matchedGt: document.getElementById("toggle-matched-gt"),
  fn: document.getElementById("toggle-fn"),
  underseg: document.getElementById("toggle-underseg"),
};

const BOX_EDGE_INDICES = [
  [0, 1], [1, 2], [2, 3], [3, 0],
  [4, 5], [5, 6], [6, 7], [7, 4],
  [0, 4], [1, 5], [2, 6], [3, 7],
];

const STATUS_COLORS = {
  tp: "#39a0ed",
  fp: "#ff5a5f",
  detection: "#f7b267",
  matched: "#58d68d",
  fn: "#ffd166",
  underseg: "#c77dff",
};

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function mul(a, scalar) {
  return [a[0] * scalar, a[1] * scalar, a[2] * scalar];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function length3(v) {
  return Math.sqrt(dot(v, v));
}

function normalize(v) {
  const len = length3(v);
  if (len < 1e-6) {
    return [0, 0, 0];
  }
  return [v[0] / len, v[1] / len, v[2] / len];
}

function getCameraBasis() {
  const { yaw, pitch, distance, target } = state.camera;
  const position = [
    target[0] + distance * Math.cos(pitch) * Math.cos(yaw),
    target[1] + distance * Math.cos(pitch) * Math.sin(yaw),
    target[2] + distance * Math.sin(pitch),
  ];
  const forward = normalize(sub(target, position));
  const worldUp = Math.abs(forward[2]) > 0.97 ? [0, 1, 0] : [0, 0, 1];
  const right = normalize(cross(forward, worldUp));
  const up = normalize(cross(right, forward));
  return { position, forward, right, up };
}

function projectPoint(point, basis) {
  const relative = sub(point, basis.position);
  const x = dot(relative, basis.right);
  const y = dot(relative, basis.up);
  const z = dot(relative, basis.forward);
  if (z <= 0.05) {
    return null;
  }
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const focal = Math.min(width, height) * 0.82;
  return {
    x: width * 0.5 + (x / z) * focal,
    y: height * 0.5 - (y / z) * focal,
    depth: z,
  };
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (canvas.width !== width * dpr || canvas.height !== height * dpr) {
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
}

function computeFrameTarget(frame) {
  const centers = [];
  for (const object of frame.detections || []) {
    centers.push(object.center);
  }
  for (const object of frame.groundtruths || []) {
    centers.push(object.center);
  }
  if (centers.length === 0 && frame.cloud && frame.cloud.length > 0) {
    let sum = [0, 0, 0];
    for (const point of frame.cloud) {
      sum = add(sum, point);
    }
    return mul(sum, 1 / frame.cloud.length);
  }
  if (centers.length === 0) {
    return [0, 0, 0];
  }
  let sum = [0, 0, 0];
  for (const center of centers) {
    sum = add(sum, center);
  }
  return mul(sum, 1 / centers.length);
}

function resetCameraToFrame() {
  if (!state.frame) {
    return;
  }
  state.camera.target = computeFrameTarget(state.frame);
  state.camera.distance = clamp(
    Math.max(18, Math.sqrt(state.frame.cloud_total_points || 1) * 0.18),
    18,
    95
  );
  state.camera.yaw = 0.55;
  state.camera.pitch = 0.38;
  requestRender();
}

function boxCorners(object) {
  const [cx, cy, cz] = object.center;
  const [length, width, height] = object.size;
  const halfL = Math.max(length * 0.5, 0.01);
  const halfW = Math.max(width * 0.5, 0.01);
  const cosYaw = Math.cos(object.yaw);
  const sinYaw = Math.sin(object.yaw);
  const base = [
    [halfL, halfW],
    [-halfL, halfW],
    [-halfL, -halfW],
    [halfL, -halfW],
  ].map(([x, y]) => [
    cx + x * cosYaw - y * sinYaw,
    cy + x * sinYaw + y * cosYaw,
  ]);
  return [
    [base[0][0], base[0][1], cz + height],
    [base[1][0], base[1][1], cz + height],
    [base[2][0], base[2][1], cz + height],
    [base[3][0], base[3][1], cz + height],
    [base[0][0], base[0][1], cz],
    [base[1][0], base[1][1], cz],
    [base[2][0], base[2][1], cz],
    [base[3][0], base[3][1], cz],
  ];
}

function isVisibleByToggle(object) {
  if (object.role === "detection") {
    if (!toggles.detection.checked) {
      return false;
    }
    if (object.status === "tp") {
      return toggles.tp.checked;
    }
    if (object.status === "fp") {
      return toggles.fp.checked;
    }
    return true;
  }
  if (!toggles.groundtruth.checked) {
    return false;
  }
  if (object.status === "matched") {
    return toggles.matchedGt.checked;
  }
  if (object.status === "underseg") {
    return toggles.underseg.checked;
  }
  return toggles.fn.checked;
}

function renderCloud(frame, basis) {
  if (!toggles.cloud.checked || !frame.cloud) {
    return;
  }
  ctx.fillStyle = "rgba(194, 214, 231, 0.42)";
  for (const point of frame.cloud) {
    const projected = projectPoint(point, basis);
    if (!projected) {
      continue;
    }
    const size = projected.depth < 20 ? 2 : 1;
    ctx.fillRect(projected.x, projected.y, size, size);
  }
}

function renderBoxes(frame, basis) {
  state.pickables = [];
  const objects = [...(frame.groundtruths || []), ...(frame.detections || [])];
  const projectedItems = [];

  for (const object of objects) {
    if (!isVisibleByToggle(object)) {
      continue;
    }
    const corners = boxCorners(object);
    const projectedCorners = corners.map((corner) => projectPoint(corner, basis));
    if (projectedCorners.some((corner) => corner === null)) {
      continue;
    }
    const center2d = projectPoint(object.center, basis);
    if (!center2d) {
      continue;
    }
    projectedItems.push({
      object,
      corners: projectedCorners,
      center2d,
      depth: center2d.depth,
    });
  }

  projectedItems.sort((lhs, rhs) => rhs.depth - lhs.depth);

  for (const item of projectedItems) {
    const isSelected =
      state.selection &&
      state.selection.role === item.object.role &&
      state.selection.id === item.object.id &&
      state.selection.status === item.object.status;
    const color = STATUS_COLORS[item.object.status] || STATUS_COLORS.detection;
    ctx.lineWidth = isSelected ? 2.6 : 1.35;
    ctx.strokeStyle = color;
    ctx.beginPath();
    for (const [from, to] of BOX_EDGE_INDICES) {
      ctx.moveTo(item.corners[from].x, item.corners[from].y);
      ctx.lineTo(item.corners[to].x, item.corners[to].y);
    }
    ctx.stroke();

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(item.center2d.x, item.center2d.y, isSelected ? 4.8 : 3.2, 0, Math.PI * 2);
    ctx.fill();

    state.pickables.push({
      x: item.center2d.x,
      y: item.center2d.y,
      object: item.object,
    });
  }
}

function renderBackground() {
  const gradient = ctx.createLinearGradient(0, 0, 0, canvas.clientHeight);
  gradient.addColorStop(0, "#08111a");
  gradient.addColorStop(1, "#03070d");
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);
}

function requestRender() {
  window.requestAnimationFrame(render);
}

function render() {
  resizeCanvas();
  renderBackground();
  if (!state.frame) {
    return;
  }
  const basis = getCameraBasis();
  renderCloud(state.frame, basis);
  renderBoxes(state.frame, basis);
}

function setSelection(object) {
  state.selection = object;
  if (!object) {
    selectionPanel.textContent = "Click a box center to inspect details.";
    requestRender();
    return;
  }
  const rows = [
    `role: ${object.role}`,
    `status: ${object.status}`,
    `type: ${object.type}`,
    `id: ${object.id}`,
    `track_id: ${object.track_id}`,
    `confidence: ${Number(object.confidence).toFixed(4)}`,
    `center: [${object.center.map((v) => Number(v).toFixed(2)).join(", ")}]`,
    `size: [${object.size.map((v) => Number(v).toFixed(2)).join(", ")}]`,
    `yaw: ${Number(object.yaw).toFixed(3)}`,
    `matched_index: ${object.matched_index}`,
    `jaccard: ${Number(object.jaccard).toFixed(4)}`,
    `matched_points: ${object.matched_points}`,
    `is_in_roi: ${object.is_in_roi}`,
    `is_in_main_lanes: ${object.is_in_main_lanes}`,
  ];
  selectionPanel.textContent = rows.join("\n");
  requestRender();
}

function renderStats(stats) {
  statsGrid.innerHTML = "";
  const entries = [
    ["Detections", stats.detections],
    ["Groundtruths", stats.groundtruths],
    ["Strict Matches", stats.strict_matches],
    ["False Positives", stats.false_positives],
    ["False Negatives", stats.false_negatives],
    ["Underseg", stats.underseg],
    ["Precision", Number(stats.precision).toFixed(3)],
    ["Recall", Number(stats.recall).toFixed(3)],
  ];
  for (const [label, value] of entries) {
    const card = document.createElement("div");
    card.className = "stat-card";
    card.innerHTML =
      `<span class="stat-label">${label}</span><span class="stat-value">${value}</span>`;
    statsGrid.appendChild(card);
  }
}

function updateFrameSummary(frame) {
  frameLabel.textContent = frame.frame_label;
  frameSummary.textContent =
    `${frame.cloud_total_points} pts | det ${frame.detections.length}` +
    (frame.has_groundtruth ? ` | gt ${frame.groundtruths.length}` : "");
}

async function resolveFrame(manifestFrame) {
  const cachedFrame = state.frameCache.get(manifestFrame.file);
  if (cachedFrame) {
    return cachedFrame;
  }

  let pendingPromise = state.framePromiseCache.get(manifestFrame.file);
  if (!pendingPromise) {
    pendingPromise = fetch(`./${manifestFrame.file}`)
      .then((response) => response.json())
      .then((frame) => {
        state.frameCache.set(manifestFrame.file, frame);
        state.framePromiseCache.delete(manifestFrame.file);
        return frame;
      })
      .catch((error) => {
        state.framePromiseCache.delete(manifestFrame.file);
        throw error;
      });
    state.framePromiseCache.set(manifestFrame.file, pendingPromise);
  }
  return pendingPromise;
}

async function loadFrame(index, resetCamera = false) {
  if (!state.manifest || index < 0 || index >= state.manifest.frames.length) {
    return;
  }
  const requestId = ++state.frameRequestId;
  const manifestFrame = state.manifest.frames[index];
  frameSelect.value = String(index);
  frameSlider.value = String(index);
  const frame = await resolveFrame(manifestFrame);
  if (requestId !== state.frameRequestId) {
    return;
  }
  state.frameIndex = index;
  state.frame = frame;
  setSelection(null);
  renderStats(frame.stats);
  updateFrameSummary(frame);
  if (resetCamera || index === 0) {
    resetCameraToFrame();
  } else {
    requestRender();
  }
}

function populateFrameControls() {
  frameSelect.innerHTML = "";
  state.manifest.frames.forEach((frame, index) => {
    const option = document.createElement("option");
    option.value = String(index);
    option.textContent = `${index}: ${frame.label}`;
    frameSelect.appendChild(option);
  });
  frameSlider.min = "0";
  frameSlider.max = String(Math.max(0, state.manifest.frames.length - 1));
  datasetMeta.textContent =
    `${state.manifest.frame_count} frames | ` +
    `${state.manifest.has_groundtruth ? "with GT matching" : "detection only"} | ` +
    `${state.manifest.max_points_per_frame} pts exported/frame`;
}

function findNearestPickable(x, y) {
  let best = null;
  let bestDistance = 24;
  for (const item of state.pickables) {
    const distance = Math.hypot(item.x - x, item.y - y);
    if (distance < bestDistance) {
      bestDistance = distance;
      best = item.object;
    }
  }
  return best;
}

function installInteractions() {
  canvas.addEventListener("contextmenu", (event) => event.preventDefault());

  canvas.addEventListener("mousedown", (event) => {
    state.interaction.active = true;
    state.interaction.mode = event.button === 2 ? "pan" : "orbit";
    state.interaction.lastX = event.clientX;
    state.interaction.lastY = event.clientY;
    state.interaction.moved = false;
    canvas.classList.add("dragging");
  });

  window.addEventListener("mouseup", (event) => {
    if (state.interaction.active && !state.interaction.moved && event.button !== 2) {
      const rect = canvas.getBoundingClientRect();
      const picked = findNearestPickable(
        event.clientX - rect.left,
        event.clientY - rect.top
      );
      setSelection(picked);
    }
    state.interaction.active = false;
    canvas.classList.remove("dragging");
  });

  window.addEventListener("mousemove", (event) => {
    if (!state.interaction.active) {
      return;
    }
    const deltaX = event.clientX - state.interaction.lastX;
    const deltaY = event.clientY - state.interaction.lastY;
    state.interaction.lastX = event.clientX;
    state.interaction.lastY = event.clientY;
    if (Math.abs(deltaX) + Math.abs(deltaY) > 2) {
      state.interaction.moved = true;
    }
    if (state.interaction.mode === "orbit") {
      state.camera.yaw -= deltaX * 0.008;
      state.camera.pitch = clamp(state.camera.pitch + deltaY * 0.008, -1.35, 1.35);
    } else {
      const basis = getCameraBasis();
      const scale = state.camera.distance * 0.0022;
      state.camera.target = add(
        state.camera.target,
        add(mul(basis.right, -deltaX * scale), mul(basis.up, deltaY * scale))
      );
    }
    requestRender();
  });

  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      const zoomFactor = 1 + Math.sign(event.deltaY) * 0.09;
      state.camera.distance = clamp(state.camera.distance * zoomFactor, 4, 180);
      requestRender();
    },
    { passive: false }
  );
}

async function boot() {
  const response = await fetch("./manifest.json");
  state.manifest = await response.json();
  populateFrameControls();
  await loadFrame(0, true);
  requestRender();
}

frameSelect.addEventListener("change", (event) => {
  loadFrame(Number(event.target.value), false);
});

frameSlider.addEventListener("input", (event) => {
  loadFrame(Number(event.target.value), false);
});

for (const toggle of Object.values(toggles)) {
  toggle.addEventListener("change", requestRender);
}

resetCameraButton.addEventListener("click", () => {
  resetCameraToFrame();
});

window.addEventListener("resize", requestRender);

installInteractions();
boot().catch((error) => {
  datasetMeta.textContent = `Failed to load viewer data: ${error}`;
  console.error(error);
});
