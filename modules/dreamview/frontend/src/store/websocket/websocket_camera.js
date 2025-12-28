import RENDERER from 'renderer';
import STORE_IMPORT from 'store'; // 重命名导入以避免冲突
import { safeParseJSON } from 'utils/JSON';
import Worker from 'utils/webworker.js';

export default class CameraDataWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.cameraDataUpdatePeriodMs = 100;
    this.worker = new Worker();
    this.timer = null;
  }

  /**
   * 安全获取 STORE 实例的辅助方法
   * 解决模块循环引用导致的 STORE_IMPORT 为空或残缺的问题
   */
  get activeStore() {
    return STORE_IMPORT || window.STORE;
  }

  initialize() {
    try {
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return;
    }

    this.websocket.onmessage = (event) => {
      this.worker.postMessage({
        source: 'camera',
        data: event.data,
      });
    };

    this.worker.onmessage = (event) => {
      const message = event.data;
      const store = this.activeStore; // 获取当前有效的 store 实例

      // 防御性校验：确保 store 及其子模块已准备就绪
      if (!store || !store.cameraData) {
        console.warn('Store.cameraData is not ready yet.');
        return;
      }

      switch (message.type) {
        case 'CameraData':
          if (message.data) {
            // 调用 init 方法前再次确认类型，防止原型链丢失
            if (typeof store.cameraData.init === 'function') {
              store.cameraData.init(message, RENDERER.coordinates);
            } else {
              console.error('STORE.cameraData.init is not a function. Instance might be corrupted.');
            }
          }
          break;
        default:
          console.warn('Camera WebSocket received unknown message:', message);
          break;
      }
    };

    this.websocket.onclose = (event) => {
      console.log(`Camera WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };
  }

  startCamera() {
    clearInterval(this.timer);
    this.timer = setInterval(() => {
      const store = this.activeStore;
      // 安全访问 options
      const canRequest = store && store.options &&
                         (store.options.showCameraView || store.options.showVideo);

      if (this.websocket.readyState === this.websocket.OPEN && canRequest) {
        this.requestCameraData();
      }
    }, this.cameraDataUpdatePeriodMs);
    return this;
  }

  stopCamera() {
    clearInterval(this.timer);
    return this;
  }

  close() {
    clearInterval(this.timer);
    if (this.websocket) {
      this.websocket.close();
    }
    return this;
  }

  requestCameraData() {
    if (this.websocket.readyState === this.websocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: 'RequestCameraData',
      }));
    }
    return this;
  }

  getCameraChannel() {
    this.websocket.send(JSON.stringify({
      type: 'GetCameraChannel',
    }));
    return new Promise(
      (resolve, reject) => {
        const listener = (event) => {
          if (event.data instanceof ArrayBuffer) {
            return;
          }
          const message = safeParseJSON(event?.data);
          if (message?.data?.name === 'GetCameraChannelListSuccess') {
            this.websocket.removeEventListener('message', listener);
            resolve(message?.data?.info?.channel);
          } else if (message?.data?.name === 'GetCameraChannelListFail') {
            this.websocket.removeEventListener('message', listener);
            reject(message?.data);
          }
        };
        this.websocket.addEventListener('message', listener);
      }
    );
  }

  changeCameraChannel(channel) {
    if (this.websocket.readyState === this.websocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: 'ChangeCameraChannel',
        data: channel,
      }));
    }
    return this;
  }
}
