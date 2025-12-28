import STORE_IMPORT from 'store';
import Worker from 'utils/webworker.js';

export default class TeleopWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.worker = new Worker();
    this.timer = null;
  }

  /**
   * 安全获取 STORE 实例
   * 解决模块循环引用导致的导出对象为 undefined 或原型丢失的问题
   */
  get activeStore() {
    return STORE_IMPORT || window.STORE;
  }

  initialize() {
    try {
      // 避免重复初始化已开启的连接
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        return;
      }

      this.websocket = new WebSocket(this.serverAddr);
      this.websocket.binaryType = 'arraybuffer';
    } catch (error) {
      console.error(`TeleopWS: Failed to establish a connection: ${error}`);
      setTimeout(() => {
        this.initialize();
      }, 1000);
      return;
    }

    this.websocket.onmessage = (event) => {
      this.worker.postMessage({
        source: 'teleop',
        data: event.data,
      });
    };

    this.websocket.onclose = (event) => {
      console.log(`Teleop WebSocket connection closed with code: ${event.code}`);
      // 只有在非主动关闭的情况下才尝试重连
      if (this.timer) {
        this.initialize();
      }
    };

    this.worker.onmessage = (event) => {
      const message = event.data;
      const store = this.activeStore;

      // 防御性校验：确保 teleop 子模块已加载
      if (store && store.teleop && typeof store.teleop.update === 'function') {
        store.teleop.update(message);
      } else {
        console.warn('TeleopWS: Store.teleop is not ready to receive updates.');
      }
    };

    // 状态请求循环：每 200ms 获取一次远程作业状态
    clearInterval(this.timer);
    this.timer = setInterval(() => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
        this.websocket.send(JSON.stringify({
          type: 'RequestTeleopStatus',
        }));
      }
    }, 200);
  }

  /**
   * 彻底关闭连接并清理定时器
   */
  close() {
    if (this.timer) {
      clearInterval(this.timer);
      this.timer = null;
    }

    if (this.websocket) {
      // 移除监听器防止触发 onclose 中的重连逻辑
      this.websocket.onclose = null;
      this.websocket.close();
      this.websocket = null;
    }
  }

  /**
   * 执行远程指令（如切换模式、紧急制动等）
   * @param {string} command 指令名称
   */
  executeCommand(command) {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify({
        type: command,
      }));
    } else {
      console.error(`TeleopWS: Cannot execute command "${command}", WebSocket is not open.`);
    }
  }
}
