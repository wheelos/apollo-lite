# 统一定位

定位运行时将连续局部运动与全局配准分离：

- `LocalLocalizationComponent` 独占 `odom -> base_link`。IMU 和车轮传播在
  室内、室外、隧道及 GNSS 不可用期间保持连续；全局观测不得重置或校正
  局部状态。
- `GlobalLocalizationComponent` 独占 `map -> odom`。GNSS、地图匹配和重定位
  观测结合各自的时间、参考系和不确定度更新全局对齐，不发布竞争性的局部
  位姿。
- `LocalizationHealthComponent` 与健康桥发布独立的方向性评估。部分可观测
  约束不会被提升为完整位姿或 TF 声明。

运行时接口定义在 `proto/unified_localization.proto`，包括局部里程计、全局
定位及方向性健康评估。默认 launch 启动局部、全局和健康进程；
`localization_odom_only.launch` 与 `localization_lane_keeping.launch` 提供
精简配置。无法安全推断的标定和测量预算必须由部署配置提供。

旧 MSF 定位器已退役。独立 NDT 地图结构与点云 IO 分别归属
`modules/ndt_localization/map_support` 和
`modules/localization/common/pointcloud_io`，不再作为另一套全局定位权威。
历史 `/apollo/localization/msf_status` 通道名作为兼容接口保留，供现有 RTK、
独立 NDT 和监控消费者使用。

实现和单元测试基线不等于车辆资格认证。剩余模型、回放和车辆验收门槛见
`wheelos-service/context/modules/localization/knowledge/directional-localization-implementation-plan.md`。
