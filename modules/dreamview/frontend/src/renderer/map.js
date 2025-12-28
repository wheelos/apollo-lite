import _ from 'lodash';
import STORE from 'store';
import { MAP_WS } from 'store/websocket';
import * as THREE from 'three';

import Text3D from 'renderer/text3d';
import TrafficSignals from 'renderer/traffic_controls/traffic_signals';
import TrafficSigns from 'renderer/traffic_controls/traffic_signs';
import {
  drawDashedLineFromPoints,
  drawSegmentsFromPoints,
  drawShapeFromPoints
} from 'utils/draw';

import stopSignMaterial from 'assets/models/stop_sign.mtl';
import stopSignObject from 'assets/models/stop_sign.obj';
import yieldSignMaterial from 'assets/models/yield_sign.mtl';
import yieldSignObject from 'assets/models/yield_sign.obj';

const STOP_SIGN_SCALE = 0.01;
const YIELD_SIGN_SCALE = 1.5;

const colorMapping = {
  YELLOW: 0XDAA520,
  WHITE: 0xCCCCCC,
  CORAL: 0xFF7F50,
  RED: 0xFF6666,
  GREEN: 0x006400,
  BLUE: 0x30A5FF,
  PURE_WHITE: 0xFFFFFF,
  DEFAULT: 0xC0C0C0,
};

export default class Map {
  constructor() {
    this.textRender = new Text3D();
    this.hash = -1;
    this.data = {};
    this.initialized = false;
    this.elementKindsDrawn = '';

    // 交通标识管理类
    this.trafficSignals = new TrafficSignals();
    this.stopSigns = new TrafficSigns(
      stopSignMaterial, stopSignObject, STOP_SIGN_SCALE,
    );
    this.yieldSigns = new TrafficSigns(
      yieldSignMaterial, yieldSignObject, YIELD_SIGN_SCALE,
    );

    // Z轴偏移因子，用于解决多层覆盖时的重影（Z-fighting）问题
    this.zOffsetFactor = 1;
  }

  /**
   * 判断当前元素类型是否在显示选项中被激活
   */
  shouldDrawObjectOfThisElementKind(kind) {
    const optionName = `showMap${kind[0].toUpperCase()}${kind.slice(1)}`;
    return STORE.options[optionName] !== false;
  }

  shouldDrawTextOfThisElementKind(kind) {
    return STORE.options.showMapLaneId && ['parkingSpace', 'lane'].includes(kind);
  }

  /**
   * 比较新旧 ID 列表，提取需要向后端请求的增量数据
   */
  diffMapElements(elementIds, data) {
    const result = {};
    let empty = true;

    for (const kind in elementIds) {
      if (!this.shouldDrawObjectOfThisElementKind(kind)) {
        continue;
      }

      result[kind] = [];
      const newIds = elementIds[kind];
      const oldDataOfKind = data[kind] || [];

      newIds.forEach(id => {
        const found = oldDataOfKind.some(old => old.id.id === id);
        if (!found) {
          empty = false;
          result[kind].push(id);
        }
      });
    }

    return empty ? {} : result;
  }

  /**
   * 绘制车道线 Mesh，根据不同类型选择渲染工具
   */
  addLaneMesh(laneType, points) {
    switch (laneType) {
      case 'DOTTED_YELLOW':
        return drawDashedLineFromPoints(points, colorMapping.YELLOW, 4, 3, 3, this.zOffsetFactor, 1);
      case 'DOTTED_WHITE':
        return drawDashedLineFromPoints(points, colorMapping.WHITE, 2, 0.5, 0.25, this.zOffsetFactor, 0.4);
      case 'SOLID_YELLOW':
        return drawSegmentsFromPoints(points, colorMapping.YELLOW, 3, this.zOffsetFactor);
      case 'SOLID_WHITE':
        return drawSegmentsFromPoints(points, colorMapping.WHITE, 3, this.zOffsetFactor);
      case 'DOUBLE_YELLOW':
        const group = new THREE.Group();
        group.add(drawSegmentsFromPoints(points, colorMapping.YELLOW, 2, this.zOffsetFactor));
        const offsetPoints = points.map(p => new THREE.Vector3(p.x + 0.3, p.y + 0.3, p.z));
        group.add(drawSegmentsFromPoints(offsetPoints, colorMapping.YELLOW, 3, this.zOffsetFactor));
        return group;
      case 'CURB':
        return drawSegmentsFromPoints(points, colorMapping.CORAL, 3, this.zOffsetFactor);
      default:
        return drawSegmentsFromPoints(points, colorMapping.DEFAULT, 3, this.zOffsetFactor);
    }
  }

  /**
   * 添加车道数据：绘制中心线和两侧边界线
   */
  addLane(lane, coordinates, scene) {
    const drewObjects = [];

    // 绘制车道中心线（通常显示为绿色）
    lane.centralCurve.segment.forEach((segment) => {
      const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
      const centerLine = drawSegmentsFromPoints(points, colorMapping.GREEN, 1, this.zOffsetFactor);
      centerLine.name = `CentralLine-${lane.id.id}`;
      scene.add(centerLine);
      drewObjects.push(centerLine);
    });

    // 绘制边界线
    const rightType = _.get(lane, 'rightBoundary.boundaryType[0].types[0]');
    lane.rightBoundary.curve.segment.forEach((segment) => {
      const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
      const boundary = this.addLaneMesh(rightType, points);
      scene.add(boundary);
      drewObjects.push(boundary);
    });

    const leftType = _.get(lane, 'leftBoundary.boundaryType[0].types[0]');
    lane.leftBoundary.curve.segment.forEach((segment) => {
      const points = coordinates.applyOffsetToArray(segment.lineSegment.point);
      const boundary = this.addLaneMesh(leftType, points);
      scene.add(boundary);
      drewObjects.push(boundary);
    });

    return drewObjects;
  }

  /**
   * 添加停止线、路边障碍物、车位等区域（Zone）
   */
  addZone(zone, color, coordinates, scene) {
    const drewObjects = [];
    const border = coordinates.applyOffsetToArray(zone.polygon.point);
    border.push(border[0]); // 闭合多边形

    const material = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.15 });
    const shape = drawShapeFromPoints(border, material, false, this.zOffsetFactor * 3);
    scene.add(shape);
    drewObjects.push(shape);

    const outline = drawSegmentsFromPoints(border, color, 2, this.zOffsetFactor, true);
    scene.add(outline);
    drewObjects.push(outline);

    return drewObjects;
  }

  /**
   * 移除销毁对象，释放 GPU 资源
   */
  removeDrewObjects(drewObjects, scene) {
    if (drewObjects) {
      drewObjects.forEach((object) => {
        scene.remove(object);
        if (object.geometry) object.geometry.dispose();
        if (object.material) {
          if (Array.isArray(object.material)) object.material.forEach(m => m.dispose());
          else object.material.dispose();
        }
      });
    }
  }

  removeExpiredElements(elementIds, scene) {
    const newData = {};
    for (const kind in this.data) {
      const isVisible = this.shouldDrawObjectOfThisElementKind(kind);
      newData[kind] = [];
      const currentIds = elementIds[kind] || [];

      this.data[kind].forEach((oldData) => {
        if (isVisible && currentIds.includes(oldData.id.id)) {
          newData[kind].push(oldData);
        } else {
          this.removeDrewObjects(oldData.drewObjects, scene);
          this.removeDrewText(oldData.text, scene);
        }
      });
    }
    this.data = newData;
  }

  /**
   * 地图循环主更新入口
   */
  updateIndex(hash, elementIds, scene) {
    if (STORE.hmi.inNavigationMode) {
      MAP_WS.requestRelativeMapData();
    } else {
      this.updateText(); // 根据缩放和配置更新 ID 标签

      let newKindStr = '';
      Object.keys(elementIds).sort().forEach(k => {
        if (this.shouldDrawObjectOfThisElementKind(k)) newKindStr += k;
      });

      // 当散列值变化或显示配置变化时，重新比对地图块
      if (hash !== this.hash || this.elementKindsDrawn !== newKindStr) {
        this.hash = hash;
        this.elementKindsDrawn = newKindStr;
        const diff = this.diffMapElements(elementIds, this.data);

        if (!_.isEmpty(diff) || !this.initialized) {
          MAP_WS.requestMapData(diff);
          this.initialized = true;
        }

        this.removeExpiredElements(elementIds, scene);
        this.updateTrafficSignals(elementIds, scene);
      }
    }
    // 相机模式下取消偏移以对齐图像
    this.zOffsetFactor = STORE.options.showCameraView ? 0 : 1;
  }

  /**
   * 处理信号灯状态更新
   */
  update(world) {
    this.trafficSignals.updateTrafficLightStatus(world.perceivedSignal);
  }
}
