import {
  action,
  computed,
  extendObservable,
  isComputed,
  makeObservable,
  observable
} from 'mobx';

import _ from 'lodash';
import MENU_DATA from 'store/config/MenuData';

export const MONITOR_MENU = Object.freeze({
  PNC_MONITOR: 'showPNCMonitor',
  DATA_COLLECTION_MONITOR: 'showDataCollectionMonitor',
  CONSOLE_TELEOP_MONITOR: 'showConsoleTeleopMonitor',
  CAR_TELEOP_MONITOR: 'showCarTeleopMonitor',
  CAMERA_PARAM: 'showCameraView',
  FUEL_CLIENT: 'showFuelClient',
});

export default class Options {
    // 显式声明 observable 属性
    @observable customizedToggles = observable.map();
    @observable togglesToHide = observable.map();
    @observable cameraAngle = 'Default';

    constructor() {
      // 【核心修复】MobX 6 必须在构造函数中调用此函数以激活装饰器
      makeObservable(this);

      this.cameraAngleNames = null;
      this.mainSideBarOptions = [
        'showTasks',
        'showModuleController',
        'showMenu',
        'showRouteEditingBar',
        'showDataRecorder',
        'showProfile',
      ];
      this.secondarySideBarOptions = ['showPOI'];

      // 初始化设置
      this.resetOptions();

      const initialToggles = {
        perceptionPointCloud: OFFLINE_PLAYBACK,
        perceptionLaneMarker: OFFLINE_PLAYBACK,
        planningCar: OFFLINE_PLAYBACK,
      };
      // 使用 merge 保持响应式链接
      this.togglesToHide.merge(initialToggles);
    }

    @action resetOptions() {
      const dynamicOptions = {};
      const staticOptions = PARAMETERS.options;

      for (const name in staticOptions) {
        let defaultValue = staticOptions[name].default;

        // Apollo 特殊逻辑处理
        if (OFFLINE_PLAYBACK) {
          if (name === 'showTasks') defaultValue = false;
          if (name === 'showPositionShadow') defaultValue = true;
        }

        if (Object.prototype.hasOwnProperty.call(this, name) || this[name] !== undefined) {
          this[name] = defaultValue;
        } else {
          // 如果是 PARAMETERS 中有，但类定义中完全没写的属性，才放入待扩展对象
          dynamicOptions[name] = defaultValue;
        }
      }

      // 仅对真正“额外”的属性进行扩展
      if (Object.keys(dynamicOptions).length > 0) {
        extendObservable(this, dynamicOptions);
      }
    }

    @computed get showTools() {
      return this.showTasks
               || this.showModuleController
               || this.showMenu
               || this.showPOI
               || this.showDataRecorder
               || this.showProfile;
    }

    @computed get showGeo() {
      return this.showRouteEditingBar
               || this.cameraAngle === 'Map'
               || this.cameraAngle === 'Overhead'
               || this.cameraAngle === 'Monitor';
    }

    @computed get showMonitor() {
      return Object.values(MONITOR_MENU).some(option => this[option]);
    }

    @computed get monitorName() {
      if (this.showConsoleTeleopMonitor) return MONITOR_MENU.CONSOLE_TELEOP_MONITOR;
      if (this.showCarTeleopMonitor) return MONITOR_MENU.CAR_TELEOP_MONITOR;
      if (this.showCameraView) return MONITOR_MENU.CAMERA_PARAM;
      if (this.showDataCollectionMonitor) return MONITOR_MENU.DATA_COLLECTION_MONITOR;
      if (this.showPNCMonitor) return MONITOR_MENU.PNC_MONITOR;
      if (this.showFuelClient) return MONITOR_MENU.FUEL_CLIENT;
      return null;
    }

    @computed get showCameraView() {
      return this.cameraAngle === 'CameraView';
    }

    @action toggle(option, isCustomized) {
      if (isCustomized) {
        const currentValue = this.customizedToggles.get(option);
        this.customizedToggles.set(option, !currentValue);
      } else {
        this[option] = !this[option];
      }

      // 侧边栏互斥逻辑
      if (this[option] && this.mainSideBarOptions.includes(option)) {
        this.mainSideBarOptions.forEach(other => {
          if (other !== option) this[other] = false;
        });
      }

      // Monitor 菜单互斥逻辑
      const monitorOptions = Object.values(MONITOR_MENU);
      if (monitorOptions.includes(option)) {
        monitorOptions.forEach(other => {
          if (other !== option && !isComputed(this, other)) {
            this[other] = false;
          }
        });
      }
    }

    @action setCustomizedToggles(toggles) {
      this.customizedToggles.clear();
      if (toggles) {
        this.customizedToggles.merge(toggles);
      }
    }

    // 辅助方法不需要装饰器
    isSideBarButtonDisabled(option, enableHMIButtonsOnly, inNavigationMode) {
      if (!this.mainSideBarOptions.includes(option)
            && !this.secondarySideBarOptions.includes(option)) {
        return false;
      }

      if (['showTasks', 'showModuleController', 'showProfile'].includes(option)) {
        return false;
      }
      if (option === 'showRouteEditingBar') {
        return enableHMIButtonsOnly || inNavigationMode;
      }
      if (option === 'showPOI') {
        return enableHMIButtonsOnly || this.showRouteEditingBar;
      }
      return enableHMIButtonsOnly;
    }

    rotateCameraAngle() {
      if (!this.cameraAngleNames) {
        const cameraData = MENU_DATA.find((data) => data.id === 'camera');
        this.cameraAngleNames = Object.values(cameraData.data);

        const shouldFilter = _.get(PARAMETERS, 'cameraAngle.hasCameraView', false);
        if (shouldFilter) {
          this.cameraAngleNames = this.cameraAngleNames.filter((name) => name !== 'CameraView');
        }
      }

      const currentIndex = this.cameraAngleNames.findIndex((name) => name === this.cameraAngle);
      const nextIndex = (currentIndex + 1) % this.cameraAngleNames.length;
      this.selectCamera(this.cameraAngleNames[nextIndex]);
    }

    @action selectCamera(angleName) {
      this.cameraAngle = angleName;
    }
}
