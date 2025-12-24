import * as THREE from 'three';

const MAX_POINTS = 200000;
const HEIGHT_COLOR_MAPPING = {
  0.5: 0xFF0000,
  1.0: 0xFF7F00,
  1.5: 0xFFFF00,
  2.0: 0x00FF00,
  2.5: 0x0000FF,
  3.0: 0x4B0082,
  10.0: 0x9400D3,
};

export default class PointCloud {
  constructor() {
    this.points = null;
    this.initialized = false;

    // Pre-allocated TypedArray
    this.positions = new Float32Array(MAX_POINTS * 3);
    this.colors = new Float32Array(MAX_POINTS * 3);
  }

  initialize() {
    // Create circular point texture
    const sprite = document.createElement('canvas');
    sprite.width = sprite.height = 64;
    const ctx = sprite.getContext('2d');
    ctx.fillStyle = 'white';
    ctx.beginPath();
    ctx.arc(32, 32, 30, 0, Math.PI * 2);
    ctx.fill();
    const texture = new THREE.CanvasTexture(sprite);

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(this.positions, 3)
    );
    geometry.setAttribute(
      'color',
      new THREE.BufferAttribute(this.colors, 3)
    );

    const material = new THREE.PointsMaterial({
      size: 0.05,          // Smaller point
      map: texture,
      alphaTest: 0.5,
      transparent: true,
      vertexColors: true,
      sizeAttenuation: true, // Scaling with distance
    });

    this.points = new THREE.Points(geometry, material);
    this.points.frustumCulled = false;
    this.initialized = true;
  }

  update(pointCloud, adcMesh) {
    if (!this.initialized || !this.points) return;
    const len = pointCloud.num.length;

    if (len % 3 !== 0) {
      console.warn('PointCloud length should be multiples of 3!');
      return;
    }

    const pointCount = Math.min(len / 3, MAX_POINTS);

    // Batch update position and color
    let posIdx = 0,
        colIdx = 0;
    for (let i = 0; i < pointCount; i++) {
      const x = pointCloud.num[i * 3];
      const y = pointCloud.num[i * 3 + 1];
      const z = pointCloud.num[i * 3 + 2];

      // Write positions
      this.positions[posIdx++] = x;
      this.positions[posIdx++] = y;
      this.positions[posIdx++] = z + 0.8;

      // Color by height range
      let key = 10.0;
      if (z < 0.5) key = 0.5;
      else if (z < 1.0) key = 1.0;
      else if (z < 1.5) key = 1.5;
      else if (z < 2.0) key = 2.0;
      else if (z < 2.5) key = 2.5;
      else if (z < 3.0) key = 3.0;

      const colorHex = HEIGHT_COLOR_MAPPING[key];
      const color = new THREE.Color(colorHex);
      this.colors[colIdx++] = color.r;
      this.colors[colIdx++] = color.g;
      this.colors[colIdx++] = color.b;
    }

    // Mark unused points as hidden (or 0,0,-10 etc.)
    for (let i = pointCount; i < MAX_POINTS; i++) {
      const base = i * 3;
      this.positions[base] = 0;
      this.positions[base + 1] = 0;
      this.positions[base + 2] = -10;
      this.colors[base] = 0;
      this.colors[base + 1] = 0;
      this.colors[base + 2] = 0;
    }

    // The marker needs to be updated.
    this.points.geometry.attributes.position.needsUpdate = true;
    this.points.geometry.attributes.color.needsUpdate = true;

    // Follow the parent object's position and orientation
    this.points.position.copy(adcMesh.position);
    this.points.rotation.set(0, 0, adcMesh.rotation.y);
  }
}
