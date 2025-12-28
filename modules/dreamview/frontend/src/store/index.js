import _ from 'lodash';
import { action, computed, makeObservable, observable } from 'mobx';

// 子 Store 导入
import CameraData from 'store/camera_data';
import ControlData from 'store/control_data';
import Dimension from 'store/dimension';
import HMI from 'store/hmi';
import Latency from 'store/latency';
import Meters from 'store/meters';
import Monitor from 'store/monitor';
import Options from 'store/options';
import PlanningData from 'store/planning_data';
import Playback from 'store/playback';
import RouteEditingManager from 'store/route_editing_manager';
import StoryTellers from 'store/story_tellers';
import StudioConnector from 'store/studio_connector';
import Teleop from 'store/teleop';
import TrafficSignal from 'store/traffic_signal';

class DreamviewStore {
    // --- 响应式状态定义 ---
    @observable timestamp = 0;
    @observable isInitialized = false;
    @observable studioConnector = new StudioConnector();
    @observable hmi = null; // 构造函数中初始化
    @observable planningData = new PlanningData();
    @observable controlData = new ControlData();
    @observable latency = new Latency();
    @observable playback = (typeof OFFLINE_PLAYBACK !== 'undefined' && OFFLINE_PLAYBACK) ? new Playback() : null;
    @observable trafficSignal = new TrafficSignal();
    @observable meters = new Meters();
    @observable monitor = new Monitor();
    @observable options = null; // 构造函数中初始化
    @observable routeEditingManager = new RouteEditingManager();
    @observable geolocation = {};
    @observable moduleDelay = observable.map();
    @observable cameraData = new CameraData();
    @observable storyTellers = new StoryTellers();
    @observable teleop = new Teleop();
    @observable dimension = null; // 依赖 HMI 和 Options
    @observable newDisengagementReminder = false;
    @observable offlineViewErrorMsg = null;

    constructor() {
        /**
         * 关键点 1：显式实例化具有依赖关系的子模块
         * 按照依赖链初始化，防止 Dimension 访问 Options 时出现 undefined
         */
        this.hmi = new HMI();
        this.options = new Options();
        this.dimension = new Dimension(this.hmi, this.options);

        /**
         * 关键点 2：MobX 6 必须调用 makeObservable(this)
         * 确保装饰器（@observable, @action）生效，否则方法将无法触发 UI 更新
         */
        makeObservable(this);
    }

    @computed get enableHMIButtonsOnly() {
        return !this.isInitialized;
    }

    // --- Actions (业务逻辑原子更新) ---

    @action updateTimestamp(newTimestamp) {
        this.timestamp = newTimestamp;
    }

    @action setInitializationStatus(status) {
        this.isInitialized = status;
    }

    @action setOfflineViewErrorMsg(msg) {
        this.offlineViewErrorMsg = msg;
    }

    /**
     * 更新各模块延迟状态 (显示在顶部工具栏)
     */
    @action updateModuleDelay(world) {
        if (world && world.delay) {
            Object.entries(world.delay).forEach(([moduleName, delayValue]) => {
                const hasNotUpdated = (delayValue < 0);
                const delayStr = hasNotUpdated ? '-' : delayValue.toFixed(2);

                if (this.moduleDelay.has(moduleName)) {
                    const item = this.moduleDelay.get(moduleName);
                    item.delay = delayStr;
                } else {
                    this.moduleDelay.set(moduleName, {
                        delay: delayStr,
                        name: _.capitalize(moduleName),
                    });
                }
            });
        }
    }

    /**
     * 全局开关切换逻辑，处理 3D 画布和布局的关联更新
     */
    @action handleOptionToggle(option) {
        if (!this.options) return;

        const oldShowMonitor = this.options.showMonitor;
        const oldShowRouteEditingBar = this.options.showRouteEditingBar;

        this.options.toggle(option);

        // 侧边栏布局联动
        if (oldShowMonitor && !this.options.showMonitor) {
            this.dimension.disableMonitor();
        } else if (!oldShowMonitor && this.options.showMonitor) {
            this.dimension.enableMonitor();
        }

        if (oldShowRouteEditingBar && !this.options.showRouteEditingBar) {
            this.routeEditingManager.disableRouteEditing();
        }

        // 触发布局重新计算
        this.dimension.update();

        if (option === 'showRouteEditingBar' && this.options.showRouteEditingBar) {
            this.options.showPOI = false;
            this.routeEditingManager.enableRouteEditing();
        }
    }

    @action setOptionStatus(option, enabled) {
        if (this.options && this.options[option] !== enabled) {
            this.handleOptionToggle(option);
        }
    }

    /**
     * 自动驾驶接管逻辑：当系统退出 Auto 模式时给出反馈
     */
    @action handleDrivingModeChange(wasAutoMode, isAutoMode) {
        if (this.options.enableSimControl) return;

        const hasDisengagement = wasAutoMode && !isAutoMode;
        const hasAutoEntry = !wasAutoMode && isAutoMode;

        this.newDisengagementReminder = this.hmi.isCoDriver && hasDisengagement;

        // 如果发生人工接管，自动弹出数据记录面板
        if (this.newDisengagementReminder && !this.options.showDataRecorder) {
            this.handleOptionToggle('showDataRecorder');
        }

        // 模式切换时自动锁定/解锁任务面板
        if (hasAutoEntry && !this.options.lockTaskPanel) {
            this.handleOptionToggle('lockTaskPanel');
        } else if (hasDisengagement && this.options.lockTaskPanel) {
            this.handleOptionToggle('lockTaskPanel');
        }
    }

    /**
     * SimulationWorld 主更新入口 (每 100ms 执行一次)
     * 此方法被 RealtimeWebSocketEndpoint 频繁调用
     */
    @action update(world, isNewMode) {
        if (!world) return;

        // 模式切换（如从自动驾驶切换到地图采集）时重置 UI 状态
        if (isNewMode) {
            this.options.resetOptions();
            this.dimension.disableMonitor();
            this.routeEditingManager.disableRouteEditing();
        }

        this.updateTimestamp(world.timestamp);
        this.updateModuleDelay(world);

        const wasAutoMode = this.meters.isAutoMode;
        this.meters.update(world);
        this.handleDrivingModeChange(wasAutoMode, this.meters.isAutoMode);

        // 状态分发至子模块
        this.monitor.update(world);
        this.trafficSignal.update(world);
        this.hmi.update(world);

        // 性能优化：仅在 PncMonitor 开启时计算高耗能规划数据
        if (this.options.showPNCMonitor) {
            this.storyTellers.update(world);
            this.planningData.update(world);
            this.controlData.update(world, this.hmi.vehicleParam);
            this.latency.update(world);
        }

        // 远程控制状态自动同步
        if (this.hmi.inCarTeleopMode) {
            this.setOptionStatus('showCarTeleopMonitor', true);
        } else if (this.hmi.inConsoleTeleopMode) {
            this.setOptionStatus('showConsoleTeleopMonitor', true);
        }
    }

    /**
     * 解决 HMI 方法丢失报错的补丁方法
     */
    @action updateVehicleParam(param) {
        if (this.hmi && typeof this.hmi.updateVehicleParam === 'function') {
            this.hmi.updateVehicleParam(param);
        }
    }
}

// 关键点 3：单例导出与全局挂载
const STORE = new DreamviewStore();


if (typeof window !== 'undefined') {
    window.STORE = STORE;
}

export default STORE;
