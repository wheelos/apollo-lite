import * as THREE from 'three';
// 1. 显式导入 JSM 模块
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

// 2. 实例化 Loader (不再挂载在 THREE 下)
const mtlLoader = new MTLLoader();
const textureLoader = new THREE.TextureLoader();

// 3. 设置跨域 (0.147 标准写法)
textureLoader.setCrossOrigin('anonymous');

const loadedMaterialAndObject = {};

export function loadObject(materialFile, objectFile, scale, callback) {
  function placeMtlAndObj(loaded) {
    if (callback) {
      // 深度克隆，确保每个实例独立
      const object = loaded.clone();
      callback(object);
    }
  }

  if (loadedMaterialAndObject[objectFile]) {
    placeMtlAndObj(loadedMaterialAndObject[objectFile]);
  } else {
    // 使用 Promise 处理异步加载流
    new Promise((resolve) => {
      if (materialFile) {
        mtlLoader.load(materialFile, (materials) => {
          materials.preload();
          resolve(materials);
        }, undefined, (error) => {
          console.error(`Failed to load MTL: ${materialFile}`, error);
          resolve(null); // 即使材质加载失败也尝试继续加载模型
        });
      } else {
        resolve(null);
      }
    }).then((materials) => {
      // 4. 在此处实例化 OBJLoader
      const objLoader = new OBJLoader();

      if (materials) {
        objLoader.setMaterials(materials);
      }

      objLoader.load(objectFile, (loaded) => {
        loaded.name = objectFile;
        // 确保 scale 存在，防止解构错误
        if (scale) {
          loaded.scale.set(scale.x, scale.y, scale.z);
        }

        // 缓存原始加载的对象
        loadedMaterialAndObject[objectFile] = loaded;
        placeMtlAndObj(loaded);
      }, undefined, (error) => {
        console.error(`Failed to load OBJ: ${objectFile}`, error);
      });
    }).catch((err) => {
      console.error('Unexpected error during object loading:', err);
    });
  }
}

/**
 * 加载纹理
 */
export function loadTexture(textureFile, onLoadCallback, onErrorCallback) {
  textureLoader.load(textureFile, onLoadCallback, undefined, onErrorCallback);
}

/**
 * 加载材质
 */
export function loadMaterial(materialFile, onLoadCallback) {
  // 注意：MTLLoader 已经是独立类
  mtlLoader.load(materialFile, onLoadCallback);
}
