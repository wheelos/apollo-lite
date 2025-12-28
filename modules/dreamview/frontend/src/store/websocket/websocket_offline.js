import RENDERER from 'renderer';
import STORE_IMPORT from 'store';

export default class OfflinePlaybackWebSocketEndpoint {
  constructor(serverAddr) {
    this.serverAddr = serverAddr;
    this.websocket = null;
    this.lastUpdateTimestamp = 0;
    this.lastSeqNum = -1;
    this.requestTimer = null;
    this.processTimer = null;
    this.frameData = {}; // 帧缓存
    this.routingTime2Path = {};
  }

  /**
   * 安全获取 STORE 实例。
   * 如果 import 的 STORE 是空的，则尝试从全局 window.STORE 获取
   */
  get activeStore() {
    return STORE_IMPORT || window.STORE;
  }

  initialize(params) {
    const store = this.activeStore;

    // 防御性检查：确保回放模块所需的状态已准备好
    if (!store || !store.playback) {
      console.warn('OfflinePlaybackWS: Store.playback is not initialized yet. Retrying...');
      setTimeout(() => this.initialize(params), 500);
      return;
    }

    if (params && params.id && params.map) {
      store.playback.setRecordId(params.id);
      store.playback.setMapId(params.map);
    } else {
      console.error('ERROR: missing required parameter(s)');
      store.setOfflineViewErrorMsg('Missing required parameter(s).');
      return;
    }

    if (params.url) {
      this.serverUrl = `${location.protocol}//${params.url}`;
    }

    try {
      this.websocket = new WebSocket(this.serverAddr);
    } catch (error) {
      console.error(`Failed to establish a connection: ${error}`);
      setTimeout(() => this.initialize(params), 1000);
      return;
    }

    this.websocket.onopen = () => {
      this.requestGroundMeta(store.playback.mapId);
    };

    this.websocket.onmessage = (event) => {
      const currentStore = this.activeStore;
      const message = JSON.parse(event.data);

      if (message.load_error) {
        currentStore.setOfflineViewErrorMsg(message.load_error);
        return;
      }

      switch (message.type) {
        case 'GroundMetadata':
          RENDERER.updateGroundMetadata(message.data);
          this.requestFrameCount(currentStore.playback.recordId);
          break;
        case 'FrameCount':
          currentStore.playback.setNumFrames(message.data);
          if (currentStore.playback.hasNext()) {
            this.requestSimulationWorld(currentStore.playback.recordId, currentStore.playback.next());
            this.requestCheckPoints(currentStore.playback.recordId, currentStore.playback.mapId);
          }
          break;
        case 'RoutePath':
          this.routingTime2Path[message.routingTime] = message.routePath;
          break;
        case 'CheckPoints':
          if (RENDERER.checkPoints) {
            RENDERER.checkPoints.update(message.data);
          }
          break;
        case 'SimWorldUpdate':
          this.checkMessage(message);
          currentStore.setInitializationStatus(true);

          const world = (typeof message.world) === 'string'
            ? JSON.parse(message.world) : message.world;

          if (world.routePath) {
            this.routingTime2Path[world.routingTime] = world.routePath;
          } else if (!(world.routingTime in this.routingTime2Path)) {
            this.requestRoutePath(currentStore.playback.recordId, world.sequenceNum);
          }

          if (currentStore.playback.isSeeking) {
            this.processSimWorld(world);
          }

          if (world.sequenceNum && !(world.sequenceNum in this.frameData)) {
            this.frameData[world.sequenceNum] = world;
            currentStore.playback.setLoadingMarker(world.sequenceNum);
          }
          break;
      }
    };

    this.websocket.onclose = (event) => {
      console.log(`WebSocket connection closed, code: ${event.code}`);
      this.pausePlayback(); // 关闭时清除定时器
      setTimeout(() => this.initialize(params), 2000);
    };
  }

  checkMessage(message) {
    const seq = message.world ? message.world.sequenceNum : message.sequenceNum;
    if (this.lastUpdateTimestamp !== 0 && message.timestamp - this.lastUpdateTimestamp > 150) {
      console.warn(`SimWorldUpdate delay: ${message.timestamp - this.lastUpdateTimestamp}ms`);
    }
    this.lastUpdateTimestamp = message.timestamp;
    this.lastSeqNum = seq;
  }

  startPlayback(msPerFrame) {
    const store = this.activeStore;
    clearInterval(this.requestTimer);
    this.requestTimer = setInterval(() => {
      if (this.websocket && this.websocket.readyState === WebSocket.OPEN && store.playback.initialized()) {
        if (!store.playback.hasNext()) {
          clearInterval(this.requestTimer);
          this.requestTimer = null;
          return;
        }
        this.requestSimulationWorld(store.playback.recordId, store.playback.next());
      }
    }, msPerFrame / 2);

    clearInterval(this.processTimer);
    this.processTimer = setInterval(() => {
      if (store.playback.initialized()) {
        const frameId = store.playback.seekingFrame;
        if (frameId in this.frameData) {
          this.processSimWorld(this.frameData[frameId]);
        }
        if (store.playback.replayComplete) {
          this.pausePlayback();
        }
      }
    }, msPerFrame);
  }

  pausePlayback() {
    clearInterval(this.requestTimer);
    clearInterval(this.processTimer);
    this.requestTimer = null;
    this.processTimer = null;
  }

  processSimWorld(world) {
    const store = this.activeStore;
    // 关键修复：确保调用前 store 内的方法依然存在
    if (store && store.playback && store.playback.shouldProcessFrame(world)) {
      if (!world.routePath) {
        world.routePath = this.routingTime2Path[world.routingTime];
      }

      // 执行状态分发，需确保这些方法在 Store 中定义为 @action
      store.updateTimestamp(world.timestamp);

      if (RENDERER) {
        RENDERER.maybeInitializeOffest(
          world.autoDrivingCar.positionX,
          world.autoDrivingCar.positionY,
        );
        RENDERER.updateWorld(world);
      }

      // 批量分发到子模块
      if (store.meters) store.meters.update(world);
      if (store.monitor) store.monitor.update(world);
      if (store.trafficSignal) store.trafficSignal.update(world);
    }
  }

  requestGroundMeta(mapId) {
    this.send({ type: 'RetrieveGroundMeta', mapId });
  }

  requestFrameCount(recordId) {
    this.send({ type: 'RetrieveFrameCount', recordId });
  }

  requestCheckPoints(recordId, mapId) {
    this.send({ type: 'RequestCheckPoints', recordId, mapId });
  }

  requestSimulationWorld(recordId, frameId) {
    const store = this.activeStore;
    if (!(frameId in this.frameData)) {
      this.send({ type: 'RequestSimulationWorld', recordId, frameId });
    } else {
      if (store.playback.isSeeking) {
        this.processSimWorld(this.frameData[frameId]);
      }
      let loadingMarker = frameId;
      while (loadingMarker in this.frameData) {
        loadingMarker++;
      }
      store.playback.setLoadingMarker(loadingMarker - 1);
    }
  }

  requestRoutePath(recordId, frameId) {
    this.send({ type: 'requestRoutePath', recordId, frameId });
  }

  // 通用发送指令方法，增加状态检查
  send(data) {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      this.websocket.send(JSON.stringify(data));
    }
  }
}
