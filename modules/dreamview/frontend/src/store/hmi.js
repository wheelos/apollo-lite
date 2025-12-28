import _ from 'lodash';
import { action, computed, makeObservable, observable } from 'mobx';

import UTTERANCE from 'store/utterance';
import WS from 'store/websocket';

const TELEOP_MODE = Object.freeze({
  CAR: 'Car Teleop',
  CONSOLE: 'Console Teleop',
});

export default class HMI {
  constructor(studioConnector) {
    this.studioConnector = studioConnector;
    // MobX 6 必须显式调用 makeObservable
    makeObservable(this);
  }

  modes = [];
  vehicles = [];
  maps = [];
  displayName = {};
  utmZoneId = 10;

  @observable currentMode = 'none';
  @observable currentVehicle = 'none';
  @observable currentMap = 'none';

  // 使用 observable.map() 存储状态，确保细粒度更新
  @observable moduleStatus = observable.map();
  @observable componentStatus = observable.map();
  @observable otherComponentStatus = observable.map();

  @observable enableStartAuto = false;
  @observable dockerImage = 'unknown';
  @observable isCoDriver = false;
  @observable isMute = false;
  @observable isPreprocess = false;
  @observable isVehicleCalibrationMode = false;
  @observable isSensorCalibrationMode = false;

  @observable dataCollectionUpdateStatus = observable.map();
  @observable dataCollectionProgress = observable.map();
  @observable lidars = observable.map();
  @observable camera = {};
  @observable mainSensor = 'none';

  @observable updateConfiguration = false;
  @observable preprocessStarted = false;
  @observable preprocessFinished = false;
  @observable unexpectedAborted = false;
  @observable preprocessStatus = 'UNKNOWN';
  @observable logString = '';
  @observable preprocessProgress = 0;
  @observable counter = 0;

  @observable dynamicModels = [];
  @observable currentDynamicModel = '';
  @observable records = {};
  @observable currentRecordId = '';
  @observable currentVehicleType = 0;
  @observable currentCameraSensorChannel = '';
  @observable currentPointCloudChannel = '';

  defaultVehicleSize = { height: 1.48, width: 2.11, length: 4.933 };
  vehicleParam = {
    frontEdgeToCenter: 3.89, backEdgeToCenter: 1.043,
    leftEdgeToCenter: 1.055, rightEdgeToCenter: 1.055,
    height: 1.48, width: 2.11, length: 4.933,
    steerRatio: 16, wheelBase: 2.8448,
  };

  @action toggleCoDriverFlag() { this.isCoDriver = !this.isCoDriver; }

  @action toggleMuteFlag() {
    this.isMute = !this.isMute;
    UTTERANCE.setMute(this.isMute);
  }

  /**
   * 核心状态更新逻辑：适配从 WebSocket 接收到的 HMIStatus
   */
  @action updateStatus(newStatus) {
    if (newStatus.dockerImage) this.dockerImage = newStatus.dockerImage;
    if (newStatus.utmZoneId) this.utmZoneId = newStatus.utmZoneId;

    if (newStatus.modes) this.modes = [...newStatus.modes].sort();
    if (newStatus.maps) this.maps = [...newStatus.maps].sort();
    if (newStatus.vehicles) this.vehicles = [...newStatus.vehicles].sort();

    // 处理当前模式切换逻辑
    if (newStatus.currentMode) {
      const modeLower = newStatus.currentMode.toLowerCase();
      this.isVehicleCalibrationMode = modeLower.includes('vehicle calibration');
      this.isSensorCalibrationMode = modeLower.includes('sensor calibration');

      if (this.currentMode !== newStatus.currentMode) {
        this.resetDataCollectionProgress();
        this.resetSensorCalibrationConfiguration();
        this.resetPreprocessProgress();
        this.currentMode = newStatus.currentMode;
        if (this.isSensorCalibrationMode) this.updateConfiguration = true;
      }
    }

    // 处理模块状态 (Modules) - 优化后的 MobX 6 兼容逻辑
    if (newStatus.modules) {
      const newKeys = Object.keys(newStatus.modules).sort();
      const currentKeys = Array.from(this.moduleStatus.keys()).sort();
      if (!_.isEqual(newKeys, currentKeys)) {
        this.moduleStatus.clear();
      }
      Object.keys(newStatus.modules).forEach(key => {
        this.moduleStatus.set(key, newStatus.modules[key]);
      });
    }

    // 处理监控组件状态 (Monitored Components)
    if (newStatus.monitoredComponents) {
      const newKeys = Object.keys(newStatus.monitoredComponents).sort();
      const currentKeys = Array.from(this.componentStatus.keys()).sort();
      if (!_.isEqual(newKeys, currentKeys)) {
        this.componentStatus.clear();
      }
      Object.keys(newStatus.monitoredComponents).forEach(key => {
        this.componentStatus.set(key, newStatus.monitoredComponents[key]);
      });

      // 如果 Recorder 启动但前置组件失败，则尝试自动触发
      if (this.startMonitorRecorderProcess && !this.allMonitoredComponentSuccess) {
        this.toggleModule(this.preConditionModule);
      }
    }

    // 更新其他动态属性
    this.currentVehicle = newStatus.currentVehicle || this.currentVehicle;
    this.records = newStatus.records || {};
    this.currentRecordId = newStatus.currentRecordId || '';

    if (typeof newStatus.passengerMsg === 'string') {
      UTTERANCE.speakRepeatedly(newStatus.passengerMsg);
    }
  }

  @action toggleModule(id) {
    const nextStatus = !this.moduleStatus.get(id);
    this.moduleStatus.set(id, nextStatus);
    const command = nextStatus ? 'START_MODULE' : 'STOP_MODULE';
    WS.executeModuleCommand(id, command);
  }

  // Computed 属性提供派生状态，减少组件内的判断逻辑
  @computed get inNavigationMode() { return this.currentMode === 'Navigation'; }
  @computed get inTeleopMode() { return Object.values(TELEOP_MODE).includes(this.currentMode); }
  @computed get isCalibrationMode() { return this.isSensorCalibrationMode || this.isVehicleCalibrationMode; }

  @computed get allMonitoredComponentSuccess() {
    if (!this.isCalibrationMode) return true;
    // 使用 Array.from 修复迭代器排序问题
    return _.every(Array.from(this.componentStatus.keys()), (key) => {
      return key === 'Recorder' || _.get(this.componentStatus.get(key), 'status') === 'OK';
    });
  }

  @computed get preConditionModule() {
    return this.isCalibrationMode ? 'Recorder' : 'none';
  }

  @computed get startMonitorRecorderProcess() {
    return this.isSensorCalibrationMode && this.moduleStatus.get('Recorder');
  }

  // 重置方法
  @action resetDataCollectionProgress() {
    this.dataCollectionUpdateStatus.clear();
    this.dataCollectionProgress.clear();
  }

  @action resetSensorCalibrationConfiguration() {
    this.lidars.clear();
    this.camera = {};
  }

  @action resetPreprocessProgress() {
    this.preprocessStarted = false;
    this.preprocessFinished = false;
    this.preprocessProgress = 0;
    this.logString = '';
  }

  /**
   * 几何计算：根据车辆参数计算自车多边形顶点
   */
  calculateCarPolygonPoints(positionX, positionY, headingInRad) {
    const config = this.vehicleParam;
    const polygonPoints = [
      { y: -config.leftEdgeToCenter, x: config.frontEdgeToCenter },
      { y: config.rightEdgeToCenter, x: config.frontEdgeToCenter },
      { y: config.rightEdgeToCenter, x: -config.backEdgeToCenter },
      { y: -config.leftEdgeToCenter, x: -config.backEdgeToCenter },
      { y: -config.leftEdgeToCenter, x: config.frontEdgeToCenter },
    ];

    return polygonPoints.map((point) => {
      // 旋转 2D 点
      const rotatedX = point.x * Math.cos(headingInRad) - point.y * Math.sin(headingInRad);
      const rotatedY = point.x * Math.sin(headingInRad) + point.y * Math.cos(headingInRad);
      return {
        x: positionX + rotatedX,
        y: positionY + rotatedY,
      };
    });
  }
}
