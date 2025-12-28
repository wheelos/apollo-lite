import MAP_NAVIGATOR from 'components/Navigation/MapNavigator';
import RENDERER from 'renderer';
import STORE_IMPORT from 'store';
import UTTERANCE from 'store/utterance';
import Worker from 'utils/webworker.js';

export default class RealtimeWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.simWorldUpdatePeriodMs = 100;
    this.simWorldLastUpdateTimestamp = 0;
    this.mapUpdatePeriodMs = 1000;
    this.mapLastUpdateTimestamp = 0;
    this.updatePOI = true;
    this.updateDefaultRoutingPoints = true;
    this.routingTime = undefined;
    this.currentMode = null;
    this.worker = new Worker();
    this.pointcloudWS = null;
    this.requestHmiStatus = this.requestHmiStatus.bind(this);
    this.updateParkingRoutingDistance = true;
    this.timer = null;
  }

  /**
   * 安全获取当前的 STORE 实例
   * 优先使用全局 window.STORE，防止 Webpack 循环引用导致的导入为空
   */
  get activeStore() {
    return STORE_IMPORT || window.STORE;
  }

  initialize() {
    try {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        return;
      }
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`RealtimeWS: Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return;
    }

    this.websocket.onmessage = (event) => {
      this.worker.postMessage({
        source: 'realtime',
        data: event.data,
      });
    };

    this.worker.onmessage = (event) => {
      const message = event.data;
      const store = this.activeStore;

      // 防御性拦截：如果 Store 还没有初始化，暂不处理数据消息
      if (!store || !store.hmi) {
        return;
      }

      switch (message.type) {
        case 'HMIStatus':
          store.hmi.updateStatus(message.data);
          if (store.studioConnector) {
            store.studioConnector.updateLocalScenarioInfo(message.data);
          }
          RENDERER.updateGroundImage(store.hmi.currentMap);
          break;

        case 'VehicleParam':
          // 修复之前报错的方法代理
          if (typeof store.hmi.updateVehicleParam === 'function') {
            store.hmi.updateVehicleParam(message.data);
          }
          break;

        case 'SimControlStatus':
          store.setOptionStatus('enableSimControl', message.enabled);
          break;

        case 'SimWorldUpdate':
          this.checkMessage(message);

          const isNewMode = (this.currentMode && this.currentMode !== store.hmi.currentMode);
          const isNavigationModeInvolved = (this.currentMode === 'Navigation'
            || store.hmi.currentMode === 'Navigation');

          this.currentMode = store.hmi.currentMode;

          if (store.hmi.shouldDisplayNavigationMap) {
            if (MAP_NAVIGATOR.isInitialized()) {
              MAP_NAVIGATOR.update(message);
            }

            if (store.hmi.inNavigationMode) {
              // 导航模式下坐标系处理
              message.autoDrivingCar.positionX = 0;
              message.autoDrivingCar.positionY = 0;
              message.autoDrivingCar.heading = 0;
              RENDERER.coordinates.setSystem('FLU');
              this.mapUpdatePeriodMs = 100;
            }
          } else {
            RENDERER.coordinates.setSystem('ENU');
            this.mapUpdatePeriodMs = 1000;
          }

          // 执行根 Store 的更新动作
          store.update(message, isNewMode);

          RENDERER.maybeInitializeOffest(
            message.autoDrivingCar.positionX,
            message.autoDrivingCar.positionY,
            isNewMode && isNavigationModeInvolved,
          );
          RENDERER.updateWorld(message);
          this.updateMapIndex(message);

          if (this.routingTime !== message.routingTime) {
            this.requestRoutePath();
            this.routingTime = message.routingTime;
          }
          break;

        case 'MapElementIds':
          RENDERER.updateMapIndex(message.mapHash, message.mapElementIds, message.mapRadius);
          break;

        case 'DefaultEndPoint':
          store.routeEditingManager.updateDefaultRoutingEndPoint(message);
          break;

        case 'DefaultRoutings':
          store.routeEditingManager.updateDefaultRoutingPoints(message);
          break;

        case 'AddDefaultRoutingPath':
          store.routeEditingManager.addDefaultRoutingPath(message);
          break;

        case 'RoutePath':
          RENDERER.updateRouting(message.routingTime, message.routePath);
          break;

        case 'RoutingPointCheckResult':
          if (message.error) {
            RENDERER.removeInvalidRoutingPoint(message.pointId, message.error);
          }
          break;

        case 'DataCollectionProgress':
          if (message.data) store.hmi.updateDataCollectionProgress(message.data);
          break;

        case 'PreprocessProgress':
          if (message.data) store.hmi.updatePreprocessProgress(message.data);
          break;

        case 'ParkingRoutingDistance':
          store.routeEditingManager.updateParkingRoutingDistance(message.threshold);
          break;
      }
    };

    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed, code: ${event.code}`);
      const now = new Date().getTime();
      const store = this.activeStore;

      if (this.simWorldLastUpdateTimestamp !== 0 && (now - this.simWorldLastUpdateTimestamp) > 10000) {
        const errorMsg = 'Connection to the server has been lost.';
        if (store && store.monitor) {
          store.monitor.insert('FATAL', errorMsg, now);
        }
        if (UTTERANCE.getCurrentText() !== errorMsg || !UTTERANCE.isSpeaking()) {
          UTTERANCE.speakOnce(errorMsg);
        }
      }
      this.initialize();
    };

    // 每一帧的请求循环
    clearInterval(this.timer);
    this.timer = setInterval(() => {
      const store = this.activeStore;
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN && store) {
        if (this.updatePOI) {
          this.requestDefaultRoutingEndPoint();
          this.updatePOI = false;
        }
        if (this.updateDefaultRoutingPoints) {
          this.requestDefaultRoutingPoints();
          this.updateDefaultRoutingPoints = false;
        }
        if (this.pointcloudWS && this.pointcloudWS.isEnabled()) {
          this.pointcloudWS.requestPointCloud();
        }

        this.requestSimulationWorld(store.options ? store.options.showPNCMonitor : false);

        if (this.updateParkingRoutingDistance) {
          this.requestParkingRoutingDistance();
          this.updateParkingRoutingDistance = false;
        }
        if (store.hmi && (store.hmi.isVehicleCalibrationMode || store.hmi.isSensorCalibrationMode)) {
          this.requestDataCollectionProgress();
          this.requestPreprocessProgress();
        }
      }
    }, this.simWorldUpdatePeriodMs);
  }

  // --- 通用发送指令方法 ---
  send(data) {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify(data));
    }
  }

  requestSimulationWorld(requestPlanningData) {
    this.send({
      type: 'RequestSimulationWorld',
      planning: requestPlanningData,
    });
  }

  checkWsConnection() {
    return (this.websocket && this.websocket.readyState === WebSocket.OPEN)
      ? this : this.initialize();
  }

  updateMapIndex(message) {
    const now = new Date();
    if (message.mapHash && (now - this.mapLastUpdateTimestamp) >= this.mapUpdatePeriodMs) {
      RENDERER.updateMapIndex(message.mapHash, message.mapElementIds, message.mapRadius);
      this.mapLastUpdateTimestamp = now;
    }
  }

  checkMessage(world) {
    const now = new Date().getTime();
    this.simWorldLastUpdateTimestamp = now;
  }

  // --- HMI 动作指令 ---
  hmiAction(action, value) {
    this.send({ type: 'HMIAction', action, value });
  }

  changeSetupMode(mode) { this.hmiAction('CHANGE_MODE', mode); }
  changeMap(map) {
    this.hmiAction('CHANGE_MAP', map);
    this.updatePOI = true;
    this.updateDefaultRoutingPoints = true;
  }
  changeVehicle(vehicle) { this.hmiAction('CHANGE_VEHICLE', vehicle); }
  changeScenario(scenarioId) { this.hmiAction('CHANGE_SCENARIO', scenarioId); }

  changeScenarioSet(scenarioSetId) {
    this.hmiAction('CHANGE_SCENARIO_SET', scenarioSetId);
    this.changeScenario(''); // 切换集后清空场景
  }

  executeModeCommand(action) {
    this.send({ type: 'HMIAction', action });
    setTimeout(this.requestHmiStatus, 5000);
  }

  executeModuleCommand(moduleName, command) {
    this.send({ type: 'HMIAction', action: command, value: moduleName });
    setTimeout(this.requestHmiStatus, 5000);
  }

  requestHmiStatus() { this.send({ type: 'HMIStatus' }); }
  requestRoutePath() { this.send({ type: 'RequestRoutePath' }); }
  requestDefaultRoutingEndPoint() { this.send({ type: 'GetDefaultEndPoint' }); }
  requestDefaultRoutingPoints() { this.send({ type: 'GetDefaultRoutings' }); }
  requestParkingRoutingDistance() { this.send({ type: 'GetParkingRoutingDistance' }); }
  requestDataCollectionProgress() { this.send({ type: 'RequestDataCollectionProgress' }); }
  requestPreprocessProgress() { this.send({ type: 'RequestPreprocessProgress' }); }

  setPointCloudWS(pointcloudws) { this.pointcloudWS = pointcloudws; }
}
