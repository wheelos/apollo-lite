import STORE_IMPORT from 'store';
import Worker from 'utils/webworker.js';

export default class PluginWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.worker = new Worker();
  }

  /**
   * 安全获取 STORE 实例
   */
  get activeStore() {
    return STORE_IMPORT || window.STORE;
  }

  checkWsConnection() {
    if (this.websocket && this.websocket.readyState === this.websocket.OPEN) {
      return this;
    }
    return this.initialize();
  }

  initialize() {
    try {
      if (this.websocket && this.websocket.readyState === this.websocket.OPEN) {
        return this;
      }
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`PluginWS: Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return this;
    }

    this.websocket.addEventListener('message', (event) => {
      this.worker.postMessage({
        source: 'point_cloud', // 注意：此处逻辑沿用原代码，通常插件数据通过此 worker 处理
        data: event.data,
      });
    });

    this.websocket.onclose = (event) => {
      console.log(`Plugin WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };

    // 处理来自 Worker 的解析数据
    this.worker.addEventListener('message', (event) => {
      if (event.data.type === 'PluginMsg') {
        const message = event.data;
        const store = this.activeStore;

        // 防御性校验：确保 StudioConnector 模块已就绪
        if (!store || !store.studioConnector) {
          console.warn('PluginWS: Store.studioConnector is not ready.');
          return;
        }

        const connector = store.studioConnector;
        const info = JSON.parse(message.data.info ?? '{}');

        switch (message.data.name) {
          case 'StudioConnectorCertStatus':
            const status = info.status;
            if (status === 'OK') {
              this.getScenarioSetList();
              this.getDynamicsModelList();
              this.getRecordList();
            }
            connector.checkCertificate(status);
            break;

          case 'GetScenarioSetListSuccess':
          case 'GetScenarioSetListFail':
            connector.updateRemoteScenarioSetList(info);
            break;

          case 'DownloadScenarioSetSuccess':
            connector.updateRemoteScenarioSetStatus(info?.scenario_set_id, info?.status);
            break;

          case 'DownloadScenarioSetFail':
            connector.updateRemoteScenarioSetStatus(info?.scenario_set_id, 'fail', info?.error_msg);
            break;

          case 'GetDynamicModelListSuccess':
          case 'GetDynamicModelListFail':
            connector.updateRemoteDynamicsModelList(info);
            break;

          case 'DownloadDynamicModelSuccess':
            connector.updateRemoteDynamicsModelStatus(info?.dynamic_model_name, info?.status);
            break;

          case 'DownloadDynamicModelFail':
            connector.updateRemoteDynamicsModelStatus(info?.dynamic_model_name, 'fail', info?.error_msg);
            break;

          case 'GetRecordsListSuccess':
          case 'GetRecordListFail':
            connector.updateRemoteRecordsList(info);
            break;

          case 'UpdateRecordToStatus':
            connector.updateRemoteRecordStatus(info?.record_id, info?.status);
            break;

          case 'DownloadRecordFail':
            connector.updateRemoteRecordStatus(info?.record_id, 'fail', info?.error_msg);
            break;

          case 'GetVehicleInfoSuccess':
            connector.updateVehicleInfo(info, 2);
            break;

          case 'GetVehicleInfoFail':
            connector.updateVehicleInfo(info, 3);
            break;

          case 'RefreshVehicleConfigSuccess':
            connector.refreshVehicleConfig(info, 2);
            break;
        }
      }
    });

    return this;
  }

  // --- 通用发送指令封装 ---
  sendPluginRequest(name, info = '') {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: 'PluginRequest',
        data: {
          name,
          info,
          source: 'dreamview',
          target: 'studio_connector',
          source_type: 'module',
          target_type: 'plugins',
        }
      }));
    }
  }

  checkCertificate() {
    this.sendPluginRequest('CheckCertStatus');
    return this;
  }

  getScenarioSetList() {
    this.sendPluginRequest('GetScenarioSetList');
    return this;
  }

  downloadScenarioSetById(scenarioSetId) {
    this.sendPluginRequest('DownloadScenarioSet', scenarioSetId);
    return this;
  }

  downloadRecord(id) {
    this.sendPluginRequest('DownloadRecord', id);
    return this;
  }

  getDynamicsModelList() {
    this.sendPluginRequest('GetDynamicModelList');
    return this;
  }

  downloadDynamicsModel(modelName) {
    this.sendPluginRequest('DownloadDynamicModel', modelName);
    return this;
  }

  getRecordList() {
    this.sendPluginRequest('GetRecordsList');
    return this;
  }

  getVehicleInfo() {
    this.sendPluginRequest('GetVehicleInfo');
    return this;
  }

  // --- 带有 Promise 回调的方法 (用于 UI 交互确认) ---

  createPluginPromise(successMsg, failMsg, sendAction) {
    sendAction();
    return new Promise((resolve, reject) => {
      const listener = (event) => {
        if (event.data.type === 'PluginMsg') {
          const name = event.data.data.name;
          if (name === successMsg) {
            this.worker.removeEventListener('message', listener);
            resolve(event.data.data.info ? JSON.parse(event.data.data.info) : null);
          } else if (name === failMsg) {
            this.worker.removeEventListener('message', listener);
            reject();
          }
        }
      };
      this.worker.addEventListener('message', listener);
    });
  }

  refreshVehicleConfig(vehicle_id) {
    return this.createPluginPromise(
      'RefreshVehicleConfigSuccess',
      'RefreshVehicleConfigFail',
      () => this.sendPluginRequest('RefreshVehicleConfig', vehicle_id)
    );
  }

  resetVehicleConfig(vehicle_id) {
    return this.createPluginPromise(
      'ResetVehicleConfigSuccess',
      'ResetVehicleConfigFail',
      () => this.sendPluginRequest('ResetVehicleConfig', vehicle_id)
    );
  }

  uploadVehicleConfig(vehicle_id) {
    return this.createPluginPromise(
      'UploadConfigSuccess',
      'UploadConfigFail',
      () => this.sendPluginRequest('UploadVehicleConfig', vehicle_id)
    );
  }

  getV2xInfo() {
    return this.createPluginPromise(
      'GetV2xInfoSuccess',
      'GetV2xInfoFail',
      () => this.sendPluginRequest('GetV2xInfo')
    );
  }

  refreshV2xConf(v2xId) {
    return this.createPluginPromise(
      'RefreshV2xConfSuccess',
      'RefreshV2xConfFail',
      () => this.sendPluginRequest('RefreshV2xConf', v2xId)
    );
  }

  resetV2xConf(v2xId) {
    return this.createPluginPromise(
      'ResetV2xConfSuccess',
      'ResetV2xConfFail',
      () => this.sendPluginRequest('ResetV2xConf', v2xId)
    );
  }

  uploadV2xConf(v2xId) {
    return this.createPluginPromise(
      'UploadV2xSuccess',
      'UploadV2xFail',
      () => this.sendPluginRequest('UploadV2xConf', v2xId)
    );
  }
}
