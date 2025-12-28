
import * as THREE from 'three';

const MAX_POINTS = 200000;
// Base size for the points in the shader
const POINT_SIZE_BASE = 50.0;

export default class PointCloud {
  constructor() {
    this.points = null;
    this.initialized = false;

    // 1. Pre-allocate Memory: Avoid GC during updates
    this.positions = new Float32Array(MAX_POINTS * 3);
  }

  initialize() {
    if (this.initialized) return;

    // --- Geometry ---
    const geometry = new THREE.BufferGeometry();
    // Use DynamicDrawUsage for data that updates every frame
    const posAttr = new THREE.BufferAttribute(this.positions, 3);
    posAttr.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute('position', posAttr);

    // --- Material (Shader) ---
    // Moving color calculation to GPU for massive performance gain
    const material = new THREE.ShaderMaterial({
      uniforms: {
        uTexture: { value: this._createCircleTexture() },
        uGradient: { value: this._createGradientTexture() },
        uSize: { value: POINT_SIZE_BASE },
        uHeightOffset: { value: 0.8 }, // Logic from original code
        uMinHeight: { value: 0.5 },
        uMaxHeight: { value: 3.0 },
        uOpacity: { value: 1.0 }
      },
      vertexShader: `
        uniform float uSize;
        uniform float uHeightOffset;
        varying float vHeight;

        void main() {
          vec3 transformed = position;
          transformed.z += uHeightOffset; // Apply offset on GPU
          vHeight = transformed.z;

          vec4 mvPosition = modelViewMatrix * vec4(transformed, 1.0);
          gl_Position = projectionMatrix * mvPosition;

          // Size attenuation
          gl_PointSize = uSize * (20.0 / -mvPosition.z);
        }
      `,
      fragmentShader: `
        uniform sampler2D uTexture;
        uniform sampler2D uGradient;
        uniform float uMinHeight;
        uniform float uMaxHeight;
        uniform float uOpacity;
        varying float vHeight;

        void main() {
          // Circle Shape
          vec4 texColor = texture2D(uTexture, gl_PointCoord);
          if (texColor.a < 0.5) discard;

          // Color Mapping
          float h = clamp((vHeight - uMinHeight) / (uMaxHeight - uMinHeight), 0.0, 1.0);
          vec3 gradientColor = texture2D(uGradient, vec2(h, 0.5)).rgb;

          gl_FragColor = vec4(gradientColor, uOpacity);
        }
      `,
      transparent: false, // Performance boost: no alpha sorting needed
      depthTest: true,
      depthWrite: true,
    });

    this.points = new THREE.Points(geometry, material);

    // Disable Frustum Culling if points are always around the ego car
    // Calculating bounding box for 200k points is expensive in JS
    this.points.frustumCulled = false;

    // Disable raycasting (expensive)
    this.points.raycast = () => {};

    this.initialized = true;
  }

  update(pointCloudData, adcMesh) {
    if (!this.initialized || !this.points) return;

    const rawData = pointCloudData?.num;
    if (!rawData || rawData.length === 0) {
        // Hide all points if no data
        this.points.geometry.setDrawRange(0, 0);
        return;
    }

    // 1. Fast Data Transfer (Memcpy)
    const len = rawData.length;
    const limit = Math.min(len, MAX_POINTS * 3);

    // .set() is much faster than looping
    if (rawData.subarray) {
        this.positions.set(rawData.subarray(0, limit));
    } else {
        this.positions.set(rawData.slice(0, limit));
    }

    // 2. Draw Range Optimization
    // Only render valid points, zero shader cost for unused buffer
    this.points.geometry.setDrawRange(0, limit / 3);

    // 3. Update Flags
    const attr = this.points.geometry.attributes.position;
    attr.needsUpdate = true;
    attr.updateRange.offset = 0;
    attr.updateRange.count = limit;

    // 4. Sync Transform
    if (adcMesh) {
      this.points.position.copy(adcMesh.position);
      this.points.rotation.set(0, 0, adcMesh.rotation.y);
      this.points.updateMatrix();
    }
  }

  dispose() {
    if (this.points) {
      this.points.geometry.dispose();
      this.points.material.uniforms.uTexture.value.dispose();
      this.points.material.uniforms.uGradient.value.dispose();
      this.points.material.dispose();
    }
    this.points = null;
    this.positions = null;
    this.initialized = false;
  }

  // --- Helpers ---
  _createGradientTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 128; canvas.height = 1;
    const ctx = canvas.getContext('2d');
    const colors = ['#FF0000', '#FF7F00', '#FFFF00', '#00FF00', '#0000FF', '#4B0082', '#9400D3'];
    const step = 128 / colors.length;
    colors.forEach((c, i) => { ctx.fillStyle = c; ctx.fillRect(i * step, 0, step, 1); });
    const tex = new THREE.CanvasTexture(canvas);
    tex.minFilter = THREE.NearestFilter;
    tex.magFilter = THREE.NearestFilter;
    return tex;
  }

  _createCircleTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 32; canvas.height = 32;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#FFFFFF';
    ctx.beginPath();
    ctx.arc(16, 16, 14, 0, Math.PI * 2);
    ctx.fill();
    return new THREE.CanvasTexture(canvas);
  }
}
