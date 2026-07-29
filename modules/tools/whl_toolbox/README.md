# whl-toolbox

单端口工具宿主，放在 `modules/tools/whl_toolbox`。

当前实现包含：

- `perception_lidar`
  - 只保留“运行并可视化”入口
  - record 自动探测 topic
  - record 到 `pcd/pose` 抽取
  - 调 `offline_lidar_obstacle_perception`
  - toolbox 自动管理输出目录
  - 可选 GT 对比，完成后会在 viewer 页展示结果目录和 summary
- `endpoint_static`
  - 内部启动 `endpoint_static_visualizer_exporter`
  - 内部启动 `cyber_recorder play`
  - 基于 `expected_exports` 自动退出
  - toolbox 自动管理输出目录
- `slam_visualization`
  - 单端口 live 页面
  - 通过容器内 `slam_stream_proxy.py` 订阅 cyber 数据
- `live_pointcloud`
  - 只需要填写点云通道
  - toolbox 启动 `modules/tools/whl_toolbox/live_pointcloud_viewer`
  - 前端页面由 toolbox 提供，点云流直接走 C++ CivetWeb WebSocket

## Run

只支持在 Apollo 容器内运行，并且代码仓路径固定为 `/apollo`。

先进入容器并准备好 Apollo 环境，然后在仓库根目录执行：

```bash
python3 modules/tools/whl_toolbox/server.py
```

默认地址：

```text
http://127.0.0.1:8080
```

`data_package` 支持三种输入形式：

- 单个 record 文件
- 包含多段 record 的目录
- 通配符模式，例如 `data/bag/good/20260324035342.record.0000*`

环境变量：

- `WHL_TOOLBOX_HOST`
- `WHL_TOOLBOX_PORT`

## Plugin Usage

### Perception LiDAR

1. 在左侧选择 `Perception LiDAR`。
2. 在右侧填写：
   - `Data Package`
   - `Sensor Name`
   - 可选的 `Detection Config`
   - 可选的 `Tracking Config`
   - 可选的 `Max Points`
3. 如果只是单次查看结果，直接点 `Run`。
4. 如果要和已有结果做对比：
   - 勾选 `Compare With GT`
   - 填写 `Baseline Result Dir`
   - 按需填写 `Benchmark Reserve`
   - 再点 `Run`
5. 任务运行后，在下方 `Jobs` 里查看进度和日志。
6. 完成后点击 `Open Viewer` 查看逐帧可视化。
7. 如果后续要把这次结果当作 GT，直接使用任务详情里显示的 `Result Dir`。

### Endpoint Static

1. 在左侧选择 `Endpoint Static`。
2. 在右侧填写：
   - `Data Package`
   - `DAG Config`
   - 可选的 `Max Full Points`
   - 可选的 `Max Filtered Points`
   - 可选的 `Export Every N`
   - 可选的 `Max Exports`
3. 点 `Run`。
4. 任务运行后，在 `Jobs` 里查看当前导出进度和日志。
5. 完成后点击 `Open Viewer` 查看静态可视化页面。

### SLAM Visualization

1. 在左侧选择 `SLAM Visualization`。
2. 点 `Open Live Viewer`。
3. 新页面打开后，直接查看实时建图或定位效果。
4. 如果页面没有数据，先确认容器内相关 SLAM 数据流已经在发布。

### Live PointCloud

1. 在左侧选择 `Live PointCloud`。
2. 填写 `PointCloud Channel`。
3. 如果希望统一到 IMU 坐标系显示，可选填写 `IMU Frame`。
4. 点 `Run` 启动实时点云后端。
5. 任务完成后，点击 `Open Viewer` 打开实时点云页面。
6. 如果填写了 `IMU Frame`，但静态 TF 里找不到对应坐标变换，任务会直接报错。

## Common Workflow

1. 启动 `whl-toolbox`。
2. 在左侧选择插件。
3. 在右侧填写参数并执行。
4. 在下方 `Jobs` 查看进度、日志和结果入口。
5. 完成后通过 `Open Viewer` 或结果目录继续使用产物。

## Required Binaries

toolbox 不会自动编译目标。

如果源码存在但二进制不存在，页面或任务会直接提示你先编译。当前至少需要这些目标中的相应子集：

- `//modules/tools/whl_toolbox:record_tool`
- `//modules/perception/lidar/tools:offline_lidar_obstacle_perception`
- `//modules/tools/whl_toolbox:live_pointcloud_viewer`
- `//modules/perception/tool/benchmark/lidar:lidar_web_visualizer_exporter`
- `//modules/perception/tool/benchmark/lidar:lidar_benchmark`
- `//modules/localization/endpoint/tools:endpoint_static_visualizer_exporter`
- `@core//cyber/tools/cyber_recorder:cyber_recorder`
