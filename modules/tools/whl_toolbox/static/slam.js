let scene, camera, renderer, controls;
let robotModel;
let currentMode = "localization";
let currentMapPointCloud = null;
let currentScanPointCloud = null;
let isDownsampled = false;
let isFollowView = false;
let mapGroup = null;
let mapObjects = new Map();
let eventSource = null;

function init() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x081018);

  camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 5000);
  camera.position.set(10, 10, 20);
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;

  const grid = new THREE.GridHelper(200, 40, 0x8899aa, 0x304154);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const light = new THREE.DirectionalLight(0xffffff, 1.0);
  light.position.set(20, 30, 40);
  scene.add(light);

  robotModel = new THREE.Mesh(
    new THREE.BoxGeometry(1.8, 4.2, 1.3),
    new THREE.MeshStandardMaterial({ color: 0x6bd3ff, roughness: 0.45 })
  );
  scene.add(robotModel);

  currentScanPointCloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0x52d091, size: 0.09 }));
  currentMapPointCloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0x94a7b8, size: 0.05 }));
  mapGroup = new THREE.Group();
  scene.add(currentScanPointCloud);
  scene.add(currentMapPointCloud);

  document.getElementById("modeToggleBtn").onclick = switchMode;
  document.getElementById("toggleViewBtn").onclick = () => {
    isFollowView = !isFollowView;
    document.getElementById("toggleViewBtn").textContent = isFollowView ? "切换为自由视角" : "切换为跟随视角";
  };
  document.getElementById("toggleMapBtn").onclick = () => {
    isDownsampled = !isDownsampled;
    document.getElementById("toggleMapBtn").textContent = isDownsampled ? "显示原始地图" : "显示降采样地图";
    loadMap();
  };

  connectStream();
  loadMap();
  window.addEventListener("resize", onResize);
  animate();
}

function switchMode() {
  currentMode = currentMode === "localization" ? "mapping" : "localization";
  document.getElementById("modeLabel").textContent = currentMode === "localization" ? "定位" : "建图";
  document.getElementById("modeToggleBtn").textContent = currentMode === "localization" ? "切换到建图" : "切换到定位";
  document.getElementById("loc-controls").classList.toggle("hidden", currentMode !== "localization");
  currentScanPointCloud.visible = currentMode === "localization";
  currentMapPointCloud.visible = currentMode === "localization";
  mapGroup.visible = currentMode === "mapping";
  if (currentMode === "mapping" && !scene.children.includes(mapGroup)) {
    scene.add(mapGroup);
  }
}

function onResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

function updatePoints(target, points) {
  const flat = new Float32Array(points.flat());
  target.geometry.dispose();
  target.geometry = new THREE.BufferGeometry();
  target.geometry.setAttribute("position", new THREE.BufferAttribute(flat, 3));
}

async function loadMap() {
  const url = isDownsampled
    ? "/api/plugins/slam_visualization/map_downsampled.pcd"
    : "/api/plugins/slam_visualization/map_raw.pcd";
  const response = await fetch(url);
  if (!response.ok) {
    document.getElementById("status").textContent = "地图文件缺失";
    return;
  }
  const text = await response.text();
  const lines = text.split("\n");
  const dataIndex = lines.findIndex((line) => line.trim().toLowerCase() === "data ascii");
  const points = [];
  for (let i = dataIndex + 1; i < lines.length; i += 1) {
    const parts = lines[i].trim().split(/\s+/);
    if (parts.length < 3) continue;
    points.push([Number(parts[0]), Number(parts[1]), Number(parts[2])]);
  }
  updatePoints(currentMapPointCloud, points);
}

function connectStream() {
  eventSource = new EventSource("/api/plugins/slam_visualization/stream");
  eventSource.addEventListener("ready", () => {
    document.getElementById("status").textContent = "SLAM 数据流已连接";
  });
  eventSource.addEventListener("pose_update", (event) => {
    const payload = JSON.parse(event.data);
    if (!payload.data || payload.mode !== currentMode) return;
    const [x, y, z, qx, qy, qz, qw] = payload.data;
    const previous = robotModel.position.clone();
    robotModel.position.set(x, y, z);
    robotModel.quaternion.set(qx, qy, qz, qw);
    document.getElementById("coordinates-display").textContent = `X: ${x.toFixed(2)}, Y: ${y.toFixed(2)}, Z: ${z.toFixed(2)}`;
    if (isFollowView) {
      const delta = new THREE.Vector3().subVectors(robotModel.position, previous);
      camera.position.add(delta);
      controls.target.copy(robotModel.position);
    }
  });
  eventSource.addEventListener("scan_update", (event) => {
    if (currentMode !== "localization") return;
    const payload = JSON.parse(event.data);
    updatePoints(currentScanPointCloud, payload.points || []);
  });
  eventSource.addEventListener("new_point_cloud_chunk", (event) => {
    if (currentMode !== "mapping") return;
    const payload = JSON.parse(event.data);
    if (mapObjects.has(payload.id)) return;
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute((payload.points || []).flat(), 3));
    const material = new THREE.PointsMaterial({ color: 0xffffff, size: 0.05, sizeAttenuation: true });
    const chunk = new THREE.Points(geometry, material);
    chunk.matrixAutoUpdate = false;
    chunk.matrix.fromArray(payload.pose || []);
    mapGroup.add(chunk);
    mapObjects.set(payload.id, chunk);
  });
  eventSource.addEventListener("map_correction", (event) => {
    const payload = JSON.parse(event.data);
    (payload.updated_poses || []).forEach((item) => {
      const object = mapObjects.get(item.id);
      if (!object) return;
      object.matrix.fromArray(item.pose || []);
    });
  });
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

init();
