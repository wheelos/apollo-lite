import _ from 'lodash'; // 显式导入 lodash
import STORE from 'store';
import * as THREE from 'three';
import {
  disposeMeshGroup,
  drawArrow,
  drawCircle,
  drawSegmentsFromPoints,
} from 'utils/draw';

/**
 * 绘制路边停车（Pull Over）的目标框状态
 */
function drawPullOverBox({
  lengthFront, lengthBack, widthLeft, widthRight,
}) {
  const pullOverStatus = new THREE.Group();
  const color = 0x006AFF;

  // 1. 绘制边界线 (使用适配 0.147 的 BufferGeometry 实现)
  const polygonPoints = [
    new THREE.Vector3(lengthFront, -widthLeft, 0),
    new THREE.Vector3(lengthFront, widthRight, 0),
    new THREE.Vector3(-lengthBack, widthRight, 0),
    new THREE.Vector3(-lengthBack, -widthLeft, 0),
    new THREE.Vector3(lengthFront, -widthLeft, 0),
  ];

  const polygon = drawSegmentsFromPoints(
    polygonPoints,
    color,
    2, // linewidth
    5, // zOffset (通过 addOffsetZ 处理)
  );
  pullOverStatus.add(polygon);

  // 2. 绘制中心圆点
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: true, // 开启透明，否则 opacity 不生效
    opacity: 0.5,
    depthWrite: false, // 防止遮挡路面
  });
  const circle = drawCircle(0.2, material);
  pullOverStatus.add(circle);

  // 3. 绘制朝向箭头
  const heading = drawArrow(
    1.5, // length
    2,   // linewidth
    0.5, // conelength
    0.5, // conewidth
    color,
    true // thickBand: 使用我们之前优化的 Line2/ThickLine
  );
  // 调整箭头朝向，使其沿 X 轴正方向（局部坐标系）
  heading.rotation.set(0, 0, -Math.PI / 2);
  pullOverStatus.add(heading);

  return pullOverStatus;
}

export default class PlanningStatus {
  constructor() {
    this.pullOverBox = null;
    this.dimension = {};
  }

  /**
   * 每一帧调用的更新函数
   */
  update(planningData, coordinates, scene) {
    // 【MobX 6 适配】使用 .get() 获取可观察 Map 的值
    const shouldDrawStatus = STORE.options.customizedToggles.get('pullOver');
    const pullOver = _.get(planningData, 'pullOver');

    // 如果没有数据或开关关闭，隐藏并返回
    if (!pullOver || !shouldDrawStatus) {
      if (this.pullOverBox) {
        this.pullOverBox.visible = false;
      }
      return;
    }

    // 检查维度是否发生变化（例如目标停车位更换）
    const isNewDimension = !this.dimension ||
            pullOver.lengthFront !== this.dimension.lengthFront ||
            pullOver.lengthBack !== this.dimension.lengthBack ||
            pullOver.widthLeft !== this.dimension.widthLeft ||
            pullOver.widthRight !== this.dimension.widthRight;

    // 1. 如果维度变了，销毁旧的 Mesh 释放 GPU 资源
    if (this.pullOverBox && isNewDimension) {
      disposeMeshGroup(this.pullOverBox);
      scene.remove(this.pullOverBox);
      this.pullOverBox = null;
    }

    // 2. 创建或显示 Mesh
    if (!this.pullOverBox) {
      this.pullOverBox = drawPullOverBox(pullOver);
      this.dimension = _.cloneDeep(pullOver); // 记录当前维度
      scene.add(this.pullOverBox);
    }

    this.pullOverBox.visible = true;

    // 3. 更新位置和旋转
    // 将全局坐标转换为相对于车速或地图的偏移坐标
    const position = coordinates.applyOffset({
      x: pullOver.position.x,
      y: pullOver.position.y,
      z: 0.3, // 稍微抬高，防止与地面共面导致 Z-fighting（闪烁）
    });

    if (position) {
      this.pullOverBox.position.set(position.x, position.y, position.z);
      // theta 是航向角
      this.pullOverBox.rotation.set(0, 0, pullOver.theta);
    }
  }

  /**
   * 当模块卸载时清理资源
   */
  dispose() {
    if (this.pullOverBox) {
      disposeMeshGroup(this.pullOverBox);
      this.pullOverBox = null;
    }
  }
}
