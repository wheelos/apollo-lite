import _ from 'lodash';
import STORE from 'store';
import * as THREE from 'three';
import { drawDashedLineFromPoints, drawThickBandFromPoints } from 'utils/draw';

/**
 * 将原始轨迹点归一化为渲染坐标系中的点
 */
function normalizePlanningTrajectory(trajectory, coordinates) {
  if (!trajectory || trajectory.length === 0) {
    return [];
  }

  const result = [];
  const minInterval = _.get(PARAMETERS, 'planning.minInterval', 0.1);

  for (let i = 0; i < trajectory.length; ++i) {
    const point = trajectory[i];
    // 使用 Apollo 坐标转换工具 applyOffset
    const normalizedPoint = coordinates.applyOffset(point);

    if (normalizedPoint === null) {
      continue;
    }

    if (result.length > 0) {
      // 计算 L1 距离，过滤掉过于密集的点，防止渲染闪烁或线条畸变
      const lastPoint = result[result.length - 1];
      const distance = Math.abs(lastPoint.x - normalizedPoint.x)
                     + Math.abs(lastPoint.y - normalizedPoint.y);
      if (distance < minInterval) {
        continue;
      }
    }

    result.push(normalizedPoint);
  }

  return result;
}

export default class PlanningTrajectory {
  constructor() {
    // 存储当前场景中的路径 Mesh/Line 对象
    this.paths = {};
  }

  /**
   * 每一帧渲染时更新轨迹
   * @param {Object} world - 包含 autoDrivingCar 和实时轨迹信息
   * @param {Object} planningData - 包含来自 Planning 模块的详细路径信息
   * @param {Object} coordinates - 坐标转换工具
   * @param {THREE.Scene} scene - Three.js 场景对象
   */
  update(world, planningData, coordinates, scene) {
    // 1. 确定轨迹宽度（通常以车宽为基准）
    let width = _.get(world, 'autoDrivingCar.width');
    if (!width) {
      width = _.get(PARAMETERS, 'planning.defaults.width', 1.4);
    }

    // 2. 准备新的路径数据
    const newPathsData = {};

    // 主规划轨迹 (Main Trajectory)
    if (world.planningTrajectory) {
      newPathsData.trajectory = world.planningTrajectory.map(
        (point) => ({ x: point.positionX, y: point.positionY })
      );
    }

    // 其他辅助路径 (如 DP Path, QP Path, Speed Decider Path 等)
    if (planningData && planningData.path) {
      planningData.path.forEach((path) => {
        if (path.name && path.pathPoint) {
          newPathsData[path.name] = path.pathPoint;
        }
      });
    }

    // 3. 处理所有已知路径（旧的 + 新的）
    const allPathNames = _.union(Object.keys(this.paths), Object.keys(newPathsData));

    allPathNames.forEach((name) => {
      // 匹配配置项名称
      const optionName = name === 'trajectory' ? 'showPlanningTrajectory' : name;

      // 【MobX 6 适配】检查开关状态
      const isVisible = STORE.options[optionName] || STORE.options.customizedToggles.get(optionName);

      if (!isVisible) {
        // 如果开关关闭，隐藏并清理资源
        if (this.paths[name]) {
          this.paths[name].visible = false;
          this.disposePath(name, scene);
        }
      } else {
        // 如果开关开启且有数据，重绘路径
        if (newPathsData[name] && newPathsData[name].length > 0) {
          // 销毁旧模型（Three.js 0.147 必须手动销毁以防内存泄漏）
          this.disposePath(name, scene);

          // 获取路径渲染属性（颜色、不透明度、层级等）
          let property = _.get(PARAMETERS, 'planning.pathProperties', {})[name];
          if (!property) {
            property = PARAMETERS.planning.pathProperties.default;
          }

          const points = normalizePlanningTrajectory(newPathsData[name], coordinates);

          if (points.length >= 2) {
            if (property.style === 'dash') {
              // 绘制虚线（多用于预测或辅助线）
              this.paths[name] = drawDashedLineFromPoints(
                points,
                property.color,
                width * property.width,
                1.0, // dashSize
                1.0, // gapSize
                property.zOffset,
                property.opacity
              );
            } else {
              // 绘制实线带（主要规划轨迹，使用 Line2/ThickLine 适配）
              this.paths[name] = drawThickBandFromPoints(
                points,
                width * property.width,
                property.color,
                property.opacity,
                property.zOffset
              );
            }

            if (this.paths[name]) {
              this.paths[name].name = `path_${name}`;
              scene.add(this.paths[name]);
            }
          }
        } else {
          // 有开关但没数据，隐藏现有路径
          if (this.paths[name]) {
            this.paths[name].visible = false;
          }
        }
      }
    });
  }

  /**
   * 彻底清理路径资源，防止显存泄漏
   */
  disposePath(name, scene) {
    const oldPath = this.paths[name];
    if (oldPath) {
      scene.remove(oldPath);
      // 销毁几何体
      if (oldPath.geometry) {
        oldPath.geometry.dispose();
      }
      // 销毁材质
      if (oldPath.material) {
        if (Array.isArray(oldPath.material)) {
          oldPath.material.forEach(m => m.dispose());
        } else {
          oldPath.material.dispose();
        }
      }
      this.paths[name] = null;
    }
  }
}
