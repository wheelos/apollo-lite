import RENDERER from 'renderer';
import STORE_IMPORT from 'store';
import { safeParseJSON } from 'utils/JSON';
import Worker from 'utils/webworker.js';

export default class PointCloudWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.worker = new Worker();
    this.enabled = false;
  }

  /**
   * 安全获取 STORE 实例
   * 解决模块循环引用导致的 STORE_IMPORT 为空的问题
   */
  get activeStore() {
    return STORE_IMPORT || window.STORE;
  }

  initialize() {
    try {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        return this;
      }
      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`PointCloudWS: Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return;
    }

    this.websocket.onmessage = (event) => {
      // 将高频点云二进制数据直接交给 Worker 处理，避免阻塞主线程
      this.worker.postMessage({
        source: 'point_cloud',
        data: event.data,
      });
    };

    this.websocket.onclose = (event) => {
      console.log(`PointCloud WebSocket connection closed with code: ${event.code}`);
      this.initialize();
    };

    this.worker.onmessage = (event) => {
      const store = this.activeStore;
      if (!store || !store.options) {
        return; // Store 尚未就绪，忽略此帧
      }

      if (event.data.type === 'PointCloudStatus') {
        // 同步后端反馈的点云开启状态到前端 Store
        store.setOptionStatus('showPointCloud', event.data.enabled);
        if (store.options.showPointCloud === false) {
          RENDERER.updatePointCloud({ num: [] }); // 清空 3D 视图中的点云
        }
      } else if (store.options.showPointCloud === true && event.data.num !== undefined) {
        // 只有在前端开关开启且数据有效时，才更新渲染器
        RENDERER.updatePointCloud(event.data);
      }
    };

    return this;
  }

  requestPointCloud() {
    const store = this.activeStore;
    // 只有当 WebSocket 连接正常且前端勾选了显示点云时，才向后端发送请求
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN
            && store && store.options && store.options.showPointCloud === true) {
      this.websocket.send(JSON.stringify({
        type: 'RequestPointCloud',
      }));
    }
  }

  isEnabled() {
    return this.enabled;
  }

  togglePointCloud(enable) {
    this.enabled = enable;
    const store = this.activeStore;

    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: 'TogglePointCloud',
        enable,
      }));
    }

    if (store && store.options && store.options.showPointCloud === false) {
      RENDERER.updatePointCloud({ num: [] });
    }
  }

  getPointCloudChannel() {
    return new Promise((resolve, reject) => {
      if (!this.websocket || this.websocket.readyState !== WebSocket.OPEN) {
        reject("WebSocket is not connected");
        return;
      }

      this.websocket.send(JSON.stringify({
        type: 'GetPointCloudChannel',
      }));

      // 使用一次性监听器获取频道列表
      const onChannelMessage = (event) => {
        if (event.data instanceof ArrayBuffer) return;

        const message = safeParseJSON(event.data);
        if (message?.data?.name === 'GetPointCloudChannelListSuccess') {
          this.websocket.removeEventListener('message', onChannelMessage);
          if (message?.data?.info?.channel) {
            resolve(message.data.info.channel);
          } else {
            reject(message.data.info);
          }
        } else if (message?.data?.name === 'GetPointCloudChannelListFail') {
          this.websocket.removeEventListener('message', onChannelMessage);
          reject(message.data.info);
        }
      };

      this.websocket.addEventListener('message', onChannelMessage);
    });
  }

  changePointCloudChannel(channel) {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: 'ChangePointCloudChannel',
        data: channel,
      }));
    }
  }
}
