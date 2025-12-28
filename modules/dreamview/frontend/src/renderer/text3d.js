import * as THREE from 'three';
// 1. 必须显式从 jsm 导入扩展组件
import { TextGeometry } from 'three/examples/jsm/geometries/TextGeometry.js';
import { FontLoader } from 'three/examples/jsm/loaders/FontLoader.js';

const _ = require('lodash');

const fonts = {};
let fontsLoaded = false;

// 2. 实例化 FontLoader (不再使用 THREE.FontLoader)
const loader = new FontLoader();
const fontPath = 'fonts/gentilis_bold.typeface.json';

loader.load(fontPath, (font) => {
  fonts.gentilis_bold = font;
  fontsLoaded = true;
},
(xhr) => {
  if (xhr.total > 0) {
    console.log(`${fontPath}: ${(xhr.loaded / xhr.total * 100).toFixed(2)}% loaded`);
  }
},
(error) => {
  console.error(`An error happened when loading ${fontPath}`, error);
});

export const TEXT_ALIGN = {
  CENTER: 'center',
  LEFT: 'left',
};

const LETTER_OFFSET = 0.05;

export default class Text3D {
  constructor() {
    this.charMeshes = {};
    this.charPointers = {};
    this.charWidths = {};
  }

  reset() {
    this.charPointers = {};
    // 性能优化：重置时将所有已有 mesh 设为不可见，而不是销毁
    _.forEach(this.charMeshes, (meshes) => {
        meshes.forEach(m => { m.visible = false; });
    });
  }

  drawText(text, scene, color = 0xFFEA00, textAlign = TEXT_ALIGN.CENTER) {
    const textMesh = this.composeText(text, color, textAlign);
    if (textMesh === null) {
      return;
    }

    const camera = scene.getObjectByName('camera');
    if (camera !== undefined) {
      textMesh.quaternion.copy(camera.quaternion);
    }

    // 确保子节点可见
    textMesh.children.forEach((c) => { c.visible = true; });
    textMesh.visible = true;

    return textMesh;
  }

  composeText(text, color, textAlign) {
    if (!fontsLoaded || !text) {
      return null;
    }

    const charIndices = _.map(text, (l) => l.charCodeAt(0) - 32);
    const textMesh = new THREE.Object3D();
    let offsetSum = 0;

    for (let j = 0; j < charIndices.length; j++) {
      const idx = charIndices[j];
      const charStr = text[j];

      if (this.charPointers[idx] === undefined) {
        this.charPointers[idx] = 0;
      }
      if (this.charMeshes[idx] === undefined) {
        this.charMeshes[idx] = [];
      }

      let pIdx = this.charPointers[idx];
      let mesh = this.charMeshes[idx][pIdx];

      if (mesh === undefined) {
        // 如果池中没有可用 mesh，则创建
        const { charMesh, charWidth } = this.drawChar3D(charStr, color);
        mesh = charMesh;
        this.charWidths[idx] = isFinite(charWidth) ? charWidth : 0.2;
        this.charMeshes[idx].push(mesh);
      } else {
        // 性能优化：复用时仅更新颜色，不重新创建 Geometry
        mesh.material.color.setHex(color);
      }

      mesh.position.set(offsetSum, 0, 0);
      offsetSum = offsetSum + (this.charWidths[idx] || 0.2) + LETTER_OFFSET;
      this.charPointers[idx]++;
      textMesh.add(mesh);
    }

    if (textAlign === 'center') {
      const offset = offsetSum / 2;
      textMesh.children.forEach((child) => {
        child.position.x -= offset;
      });
    }

    return textMesh;
  }

  // 3. 修改 drawChar3D 以适配 TextGeometry API
  drawChar3D(char, color, font = fonts.gentilis_bold, size = 0.6, height = 0) {
    if (!font) return { charMesh: new THREE.Group(), charWidth: 0 };

    const charGeo = new TextGeometry(char, {
      font,
      size,
      height, // 2D 效果通常设为 0
      curveSegments: 4, // 性能优化：自动驾驶场景不需要太高的曲线细分数
    });

    const charMaterial = new THREE.MeshBasicMaterial({ color });
    const charMesh = new THREE.Mesh(charGeo, charMaterial);

    charGeo.computeBoundingBox();
    const { max, min } = charGeo.boundingBox;

    return { charMesh, charWidth: max.x - min.x };
  }
}
