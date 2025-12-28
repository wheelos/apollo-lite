import * as THREE from 'three';

import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';

const _ = require('lodash');

const DELTA_Z_OFFSET = 0.04;
const textureLoader = new THREE.TextureLoader();

// 辅助函数：统一处理 Z-Offset
export function addOffsetZ(mesh, value) {
  if (value && mesh) {
    const zOffset = value * DELTA_Z_OFFSET;
    mesh.position.z += zOffset;
  }
}

/**
 * 工业级：绘制带厚度的线（替换原 ThreeLine2D 逻辑）
 */
export function drawThickBandFromPoints(
  points, thickness = 0.5, color = 0xffffff, opacity = 1, zOffset = 0,
) {
  if (!points || points.length < 2) return new THREE.Group();

  const positions = [];
  points.forEach((p) => positions.push(p.x, p.y, p.z || 0));

  const geometry = new LineGeometry();
  geometry.setPositions(positions);

  const material = new LineMaterial({
    color: new THREE.Color(color),
    linewidth: thickness, // 注意：此处单位由 resolution 决定
    transparent: opacity < 1,
    opacity: opacity,
    depthWrite: false, // 预测线通常不写深度，防止闪烁
  });

  // 必须设置 resolution，否则宽度为 0。
  // 在 render 循环中应用 renderer.getSize() 效果更佳
  material.resolution.set(window.innerWidth, window.innerHeight);

  const line = new Line2(geometry, material);
  line.computeLineDistances();
  addOffsetZ(line, zOffset);
  return line;
}

/**
 * 绘制普通虚线 (升级至 BufferGeometry)
 */
export function drawDashedLineFromPoints(
  points, color = 0xff0000, linewidth = 1, dashSize = 4, gapSize = 2,
  zOffset = 0, opacity = 1, matrixAutoUpdate = true,
) {
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  geometry.computeLineDistances();
  const material = new THREE.LineDashedMaterial({
    color,
    dashSize,
    linewidth,
    gapSize,
    transparent: opacity < 1,
    opacity,
  });
  const mesh = new THREE.Line(geometry, material);
  addOffsetZ(mesh, zOffset);
  mesh.matrixAutoUpdate = matrixAutoUpdate;
  if (!matrixAutoUpdate) mesh.updateMatrix();
  return mesh;
}

/**
 * 绘制实心多边形平面 (适配 0.147)
 */
export function drawShapeFromPoints(points,
  material = new THREE.MeshBasicMaterial({ color: 0xff0000 }),
  bezierCurve = false, order = 0, matrixAutoUpdate = true) {

  const shape = new THREE.Shape();
  if (points && points.length > 0) {
    shape.moveTo(points[0].x, points[0].y);
    if (!bezierCurve) {
      for (let i = 1; i < points.length; i++) {
        shape.lineTo(points[i].x, points[i].y);
      }
    } else {
      // 简化处理贝塞尔，确保工业级稳定性
      for (let i = 1; i < points.length - 1; i++) {
        shape.quadraticCurveTo(points[i].x, points[i].y, points[i+1].x, points[i+1].y);
      }
    }
  }

  const geometry = new THREE.ShapeGeometry(shape);
  const mesh = new THREE.Mesh(geometry, material);
  addOffsetZ(mesh, order);
  mesh.matrixAutoUpdate = matrixAutoUpdate;
  if (!matrixAutoUpdate) mesh.updateMatrix();
  return mesh;
}

/**
 * 绘制 3D 障碍物实心框 (CubeGeometry 已改名为 BoxGeometry)
 */
export function drawSolidBox(dimension, color, linewidth) {
  const geometry = new THREE.BoxGeometry(dimension.x, dimension.y, dimension.z);
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 0.8,
  });
  const box = new THREE.Mesh(geometry, material);

  // 添加边缘线
  const edges = new THREE.EdgesGeometry(geometry);
  const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color, linewidth }));
  box.add(line);

  return box;
}

/**
 * 统一资源释放 (关键：防止内存泄漏)
 */
export function disposeMesh(mesh) {
  if (!mesh) return;
  if (mesh.geometry) mesh.geometry.dispose();
  if (mesh.material) {
    if (Array.isArray(mesh.material)) {
      mesh.material.forEach((m) => m.dispose());
    } else {
      mesh.material.dispose();
    }
    if (mesh.material.map) mesh.material.map.dispose();
  }
}

export function drawCircle(radius, material, segments = 32) {
  const geometry = new THREE.CircleGeometry(radius, segments);
  return new THREE.Mesh(geometry, material);
}

export function drawEllipse(aRadius, bRadius, material) {
  const curve = new THREE.EllipseCurve(0, 0, aRadius, bRadius, 0, 2 * Math.PI, false, 0);
  const points = curve.getPoints(50);
  const geometry = new THREE.BufferGeometry().setFromPoints(points);
  // 高斯分布通常用 Line 或 Mesh 表示
  return new THREE.LineLoop(geometry, material);
}

// 保持其他辅助函数接口一致...
export function drawImage(img, width, height, x = 0, y = 0, z = 0) {
  const material = new THREE.MeshBasicMaterial({
    map: textureLoader.load(img),
    transparent: true,
    depthWrite: false,
    side: THREE.DoubleSide
  });
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, height), material);
  mesh.position.set(x, y, z);
  return mesh;
}
