import RENDERER from 'renderer';
import STORE_IMPORT from 'store';
import Worker from 'utils/webworker.js';

export default class MapDataWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.currentMode = null;
    this.worker = new Worker();
  }

  /**
   * 安全获取当前的 STORE 实例
   */
  get activeStore() {
    return STORE_IMPORT || window.STORE;
  }

  initialize() {
    try {
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`MapDataWS: Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return;
    }

    this.websocket.onmessage = (event) => {
      this.worker.postMessage({
        source: 'map',
        data: event.data,
      });
    };

    this.worker.onmessage = (event) => {
      const store = this.activeStore;

      // 防御性检查：确保 hmi 模块已加载
      if (!store || !store.hmi) {
        console.warn('MapDataWS: Store or HMI not initialized yet.');
        return;
      }

      // 获取当前模式并判断是否需要清除旧地图缓存
      const inNavigationMode = store.hmi.inNavigationMode || false;
      const removeOldMap = inNavigationMode || this.currentMode !== store.hmi.currentMode;

      this.currentMode = store.hmi.currentMode;

      // 更新渲染器中的地图数据
      if (RENDERER && typeof RENDERER.updateMap === 'function') {
        RENDERER.updateMap(event.data, removeOldMap);
      }

      // 标记系统已初始化
      if (typeof store.setInitializationStatus === 'function') {
        store.setInitializationStatus(true);
      }
    };

    this.websocket.onclose = (event) => {
      console.log(`MapData WebSocket closed (code: ${event.code}), reconnecting...`);
      this.initialize();
    };
  }

  requestMapData(elements) {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: 'RetrieveMapData',
        elements,
      }));
    }
  }

  requestRelativeMapData(elements) {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: 'RetrieveRelativeMapData',
        elements,
      }));
    }
  }
}
