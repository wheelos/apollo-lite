const liveState = {
  port: null,
  channel: "",
  frameCount: 0,
  lastTimestampSec: 0,
  lastPointCount: 0,
  obstacleChannel: "/apollo/perception/obstacles",
  obstacleCount: 0,
  obstacleTimestampSec: 0,
  obstacleFrame: "",
  overlayError: "",
  websocket: null,
  reconnectTimer: null,
  cameraInitialized: false,
  userMovedCamera: false,
  imuFrameId: "",
  sourceFrame: "",
};

function setStatus(mode, text) {
  const pill = document.getElementById("status-pill");
  pill.classList.remove("running", "error");
  if (mode) {
    pill.classList.add(mode);
  }
  document.getElementById("status-text").textContent = text;
}

function updateMetrics() {
  document.getElementById("channel-value").textContent = liveState.channel || "-";
  document.getElementById("imu-frame-value").textContent = liveState.imuFrameId || "-";
  document.getElementById("source-frame-value").textContent = liveState.sourceFrame || "-";
  document.getElementById("backend-value").textContent = liveState.port ? `${location.hostname}:${liveState.port}` : "-";
  document.getElementById("frames-value").textContent = String(liveState.frameCount);
  document.getElementById("timestamp-value").textContent = liveState.lastTimestampSec ? liveState.lastTimestampSec.toFixed(3) : "-";
  document.getElementById("points-value").textContent = String(liveState.lastPointCount || 0);
  document.getElementById("obstacle-channel-value").textContent = liveState.obstacleChannel || "-";
  document.getElementById("obstacle-count-value").textContent = String(liveState.obstacleCount || 0);
  document.getElementById("obstacle-frame-value").textContent = liveState.obstacleFrame || "-";
  document.getElementById("obstacle-timestamp-value").textContent =
    liveState.obstacleTimestampSec ? liveState.obstacleTimestampSec.toFixed(3) : "-";
  document.getElementById("overlay-error-value").textContent = liveState.overlayError || "OK";
}

async function fetchState() {
  const response = await fetch("/api/plugins/live_pointcloud/state");
  if (!response.ok) {
    throw new Error(`${response.status} ${response.statusText}`);
  }
  return response.json();
}

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x050b12);

const camera = new THREE.PerspectiveCamera(72, 1, 0.1, 5000);
camera.position.set(0, -20, 15);
camera.up.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(window.devicePixelRatio || 1);
document.getElementById("viewer").appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.minDistance = 1;
controls.maxDistance = 500;
controls.addEventListener("start", () => {
  liveState.userMovedCamera = true;
});
controls.update();

const grid = new THREE.GridHelper(80, 40, 0x2b4b66, 0x173047);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(3));

const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);
const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(8, -10, 18);
scene.add(directionalLight);

const obstacleGeometry = new THREE.BufferGeometry();
const obstacleMaterial = new THREE.LineBasicMaterial({
  vertexColors: true,
  transparent: true,
  opacity: 0.95,
});
const obstacleLinesObject = new THREE.LineSegments(obstacleGeometry, obstacleMaterial);
scene.add(obstacleLinesObject);

const geometry = new THREE.BufferGeometry();
const material = new THREE.PointsMaterial({
  size: 0.12,
  color: 0x63d3ff,
  sizeAttenuation: true,
});
const pointsObject = new THREE.Points(geometry, material);
scene.add(pointsObject);

function resizeRenderer() {
  const container = document.getElementById("viewer");
  const width = container.clientWidth;
  const height = container.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / Math.max(height, 1);
  camera.updateProjectionMatrix();
}

window.addEventListener("resize", resizeRenderer);
resizeRenderer();

function animate() {
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();

function fitCameraToPointCloud(bounds) {
  const radius = Math.max(bounds.radius || 1, 6);
  const center = bounds.center.clone();
  controls.target.copy(center);
  const distance = Math.max(radius * 1.6, 8);
  camera.position.set(
    center.x,
    center.y - distance,
    center.z + distance * 0.45
  );
  camera.lookAt(center);
  controls.update();
}

function updatePointCloud(buffer) {
  const view = new DataView(buffer);
  const count = view.getUint32(0, true);
  const timestampSec = view.getFloat64(4, true);
  const positions = new Float32Array(count * 3);
  let offset = 12;
  for (let i = 0; i < count * 3; ++i) {
    positions[i] = view.getFloat32(offset, true);
    offset += 4;
  }
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geometry.computeBoundingSphere();
  if (geometry.boundingSphere && (!liveState.cameraInitialized || !liveState.userMovedCamera)) {
    fitCameraToPointCloud(geometry.boundingSphere);
    liveState.cameraInitialized = true;
  }
  liveState.frameCount += 1;
  liveState.lastTimestampSec = timestampSec;
  liveState.lastPointCount = count;
  updateMetrics();
}

function obstacleColor(type) {
  switch (type) {
    case "PEDESTRIAN":
      return new THREE.Color(0xffd166);
    case "BICYCLE":
      return new THREE.Color(0x63e6be);
    case "VEHICLE":
      return new THREE.Color(0x7cff6b);
    default:
      return new THREE.Color(0xff6b8f);
  }
}

function pushLoopSegments(target, points) {
  const count = points.length / 3;
  if (count < 2) {
    return;
  }
  for (let i = 0; i < count; ++i) {
    const next = (i + 1) % count;
    const baseA = i * 3;
    const baseB = next * 3;
    target.push(
      points[baseA], points[baseA + 1], points[baseA + 2],
      points[baseB], points[baseB + 1], points[baseB + 2]
    );
  }
}

function pushVerticalSegments(target, base, top) {
  const count = Math.min(base.length, top.length) / 3;
  for (let i = 0; i < count; ++i) {
    const index = i * 3;
    target.push(
      base[index], base[index + 1], base[index + 2],
      top[index], top[index + 1], top[index + 2]
    );
  }
}

function pushRepeatedColor(target, color, vertexCount) {
  for (let i = 0; i < vertexCount; ++i) {
    target.push(color.r, color.g, color.b);
  }
}

function updateObstacleOverlay(payload) {
  const linePositions = [];
  const lineColors = [];
  const obstacles = Array.isArray(payload.obstacles) ? payload.obstacles : [];
  for (const obstacle of obstacles) {
    const base = Array.isArray(obstacle.base) ? obstacle.base : [];
    const top = Array.isArray(obstacle.top) ? obstacle.top : [];
    const before = linePositions.length / 3;
    pushLoopSegments(linePositions, base);
    pushLoopSegments(linePositions, top);
    pushVerticalSegments(linePositions, base, top);
    const addedVertices = linePositions.length / 3 - before;
    pushRepeatedColor(lineColors, obstacleColor(obstacle.type), addedVertices);
  }

  obstacleGeometry.setAttribute("position", new THREE.Float32BufferAttribute(linePositions, 3));
  obstacleGeometry.setAttribute("color", new THREE.Float32BufferAttribute(lineColors, 3));
  obstacleGeometry.computeBoundingSphere();

  liveState.obstacleChannel = payload.channel || liveState.obstacleChannel;
  liveState.obstacleCount = obstacles.length;
  liveState.obstacleTimestampSec = Number(payload.timestamp_sec || 0);
  liveState.obstacleFrame = payload.frame || "";
  liveState.overlayError = "";
  updateMetrics();
}


function scheduleReconnect() {
  if (liveState.reconnectTimer) {
    return;
  }
  liveState.reconnectTimer = window.setTimeout(() => {
    liveState.reconnectTimer = null;
    connect();
  }, 1500);
}

async function connect() {
  try {
    const state = await fetchState();
    liveState.port = state.port || null;
    liveState.channel = state.channel || "";
    liveState.obstacleChannel = state.perception_obstacle_channel || "/apollo/perception/obstacles";
    liveState.imuFrameId = state.imu_frame_id || "";
    liveState.frameCount = Number(state.frame_count || 0);
    liveState.lastTimestampSec = Number(state.last_timestamp_sec || 0);
    liveState.lastPointCount = Number(state.last_point_count || 0);
    liveState.obstacleCount = Number(state.last_obstacle_count || 0);
    liveState.obstacleTimestampSec = Number(state.last_obstacle_timestamp_sec || 0);
    liveState.obstacleFrame = state.obstacle_display_frame || "";
    liveState.overlayError = state.obstacle_overlay_error || "";
    updateMetrics();
    if (!state.running || !state.port) {
      setStatus("error", "Viewer backend is not running");
      scheduleReconnect();
      return;
    }
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${scheme}://${location.hostname}:${state.port}/ws`);
    ws.binaryType = "arraybuffer";
    ws.onopen = () => {
      liveState.websocket = ws;
      setStatus("running", "Streaming");
    };
    ws.onmessage = (event) => {
      if (typeof event.data === "string") {
        const payload = JSON.parse(event.data);
        if (payload.type === "status") {
          liveState.channel = payload.channel || liveState.channel;
          liveState.obstacleChannel = payload.perception_obstacle_channel || liveState.obstacleChannel;
          liveState.imuFrameId = payload.target_frame || liveState.imuFrameId;
          liveState.sourceFrame = payload.source_frame || liveState.sourceFrame;
          liveState.port = payload.port || liveState.port;
          liveState.lastTimestampSec = Number(payload.last_timestamp_sec || liveState.lastTimestampSec || 0);
          liveState.lastPointCount = Number(payload.last_point_count || liveState.lastPointCount || 0);
          liveState.frameCount = Number(payload.frame_count || liveState.frameCount || 0);
          liveState.obstacleCount = Number(payload.last_obstacle_count || liveState.obstacleCount || 0);
          liveState.obstacleTimestampSec = Number(
            payload.last_obstacle_timestamp_sec || liveState.obstacleTimestampSec || 0
          );
          liveState.obstacleFrame = payload.obstacle_display_frame || liveState.obstacleFrame;
          liveState.overlayError = payload.obstacle_overlay_error || "";
          updateMetrics();
        } else if (payload.type === "obstacles") {
          updateObstacleOverlay(payload);
        }
        return;
      }
      updatePointCloud(event.data);
    };
    ws.onclose = () => {
      if (liveState.websocket === ws) {
        liveState.websocket = null;
      }
      setStatus("error", "Disconnected");
      scheduleReconnect();
    };
    ws.onerror = () => {
      ws.close();
    };
  } catch (error) {
    setStatus("error", String(error));
    scheduleReconnect();
  }
}

connect();
