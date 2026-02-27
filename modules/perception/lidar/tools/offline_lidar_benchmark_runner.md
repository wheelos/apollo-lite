# 离线 LiDAR：跑模型结果 + 计算 Precision/Recall/AP（支持回归对比）

这个文档介绍如何把两个现成工具串起来：

- 产出离线推理结果（每帧一个 `*.txt`）：`//modules/perception/lidar/tools:offline_lidar_obstacle_perception`
- 读取 `PCD + result_txt + gt_txt` 计算 `precision/recall/AP`：`//modules/perception/tool/benchmark/lidar:lidar_benchmark`

并提供一键脚本：

- `modules/perception/lidar/tools/run_offline_lidar_benchmark.sh`

## 1. 工具做的事情分别是什么？

### 1.1 offline_lidar_obstacle_perception（产出 result）

它加载 `--lidar_detection_config_file`（`PipelineConfig`）并执行对应的 stage pipeline；不同 pipeline / stage 决定走什么模型、什么 inference。

常见配置：

- `modules/perception/pipeline/config/lidar_detection_pipeline.pb.txt`
  - stage 含 `CNN_SEGMENTATION`
  - `cnnseg64_param.conf` 里 `model_type: "RTNet"`，因此会走 TensorRT 的 `RTNet`（`rt_net`）
- `modules/perception/pipeline/config/lidar_detection_pipeline_trt.pb.txt`
  - stage 含 `CENTER_POINT_TRT_DETECTION`
  - 走 `onnx_multi_batch` 的 TensorRT（engine cache 通常是 `*.trt10.engine`）

### 1.2 lidar_benchmark（只做评估，不跑推理）

它不跑模型，只读三组数据：

1) `--cloud`：PCD 文件（或其 list 文件）
2) `--result`：你的检测输出 txt（或其 list）
3) `--groundtruth`：GT txt（或其 list）；也可以用“旧版本 result 目录”当伪 GT 来做回归一致性检查

评估核心是按点集 overlap/Jaccard 做匹配，从而算 PR/AP。

## 2. 一键脚本：run_offline_lidar_benchmark.sh

脚本位置：`modules/perception/lidar/tools/run_offline_lidar_benchmark.sh`

它做两件事：

1) 先跑离线推理，把每帧结果写到 `--result_dir/*.txt`
2) 如果传了 `--gt_dir`，再自动生成 list 文件并跑 evaluator 输出指标（log）

注意：脚本默认直接执行 `bazel-bin/...` 下的二进制，不会自动 build；请确保二进制已在你的环境里构建好，或用 `--offline_bin/--benchmark_bin` 指定它们的位置。

### 2.1 只生成 result（不评估）

```bash
./modules/perception/lidar/tools/run_offline_lidar_benchmark.sh \
  --pcd_dir=/data/pcd \
  --result_dir=/tmp/result_run1 \
  --lidar_detection_config_file=modules/perception/pipeline/config/lidar_detection_pipeline.pb.txt \
  --lidar_tracking_config_file=modules/perception/pipeline/config/lidar_tracking_pipeline.pb.txt \
  --enable_tracking=false
```

输出：

- `/tmp/result_run1/*.txt`（逐帧 result）
- `/tmp/result_run1/logs/offline_lidar_obstacle_perception.*.log`

### 2.2 有 GT：直接评估 PR/AP

```bash
./modules/perception/lidar/tools/run_offline_lidar_benchmark.sh \
  --pcd_dir=/data/pcd \
  --result_dir=/tmp/result_run1 \
  --gt_dir=/data/gt_txt \
  --lidar_detection_config_file=modules/perception/pipeline/config/lidar_detection_pipeline.pb.txt \
  --lidar_tracking_config_file=modules/perception/pipeline/config/lidar_tracking_pipeline.pb.txt
```

输出：

- `/tmp/result_run1/logs/lidar_benchmark.*.log`（评估指标打印）

### 2.3 无 GT：回归一致性（旧版本当伪 GT）

这适合验证 “TRT7/8 vs TRT10” 或 “改动前 vs 改动后” 是否输出一致：

1) 旧版本跑出 `result_old/`
2) 新版本跑出 `result_new/`
3) 用 `result_old` 作为 `--gt_dir`：

```bash
./modules/perception/lidar/tools/run_offline_lidar_benchmark.sh \
  --pcd_dir=/data/pcd \
  --result_dir=/tmp/result_new \
  --gt_dir=/tmp/result_old \
  --lidar_detection_config_file=modules/perception/pipeline/config/lidar_detection_pipeline_trt.pb.txt \
  --lidar_tracking_config_file=modules/perception/pipeline/config/lidar_tracking_pipeline.pb.txt \
  --reserve="JACCARD:0.9"
```

说明：

- 指标接近 100% 代表新旧输出高度一致，但不代表“绝对正确”（旧版本也可能错）。
- `--reserve` 会传给 evaluator，用于调阈值（例如提高 `JACCARD` 让匹配更严格）；格式是 `KEY:VALUE|KEY:VALUE`（例如 `JACCARD:0.9|RANGE:distance`）。

## 3. 常见坑

1) 文件名必须能对齐
脚本按 PCD 的 `stem`（`xxx.pcd` -> `xxx.txt`）去找 result/gt；缺文件会直接中止评估。

2) tracking config 仍然必填
`offline_lidar_obstacle_perception` 在 `setup()` 里一定会初始化 tracking pipeline，所以即使 `--enable_tracking=false` 也需要给 `--lidar_tracking_config_file` 一个有效路径。

3) 运行目录/配置管理器
离线工具内部会设置 `FLAGS_config_manager_path="./conf"`，因此需要你的运行环境有对应的 `conf/`（通常 Apollo 运行环境有）。如果你的环境没有，请先补齐或调整运行方式。

## 4. 从通道导出 PCD（给离线工具喂数据）

如果你只有在线/record 的点云通道，想先导出一批 `.pcd` 再做离线测试，可以用：

```bash
  /apollo/bazel-bin/modules/perception/lidar/tools/exporter/pcd_exporter \
  /apollo/sensor/lidar128/PointCloud2 \
  /tmp/pcd_dump
```

它会在输出目录持续写入 `*.pcd`（文件名包含时间戳，方便后续与 `*.txt` 对齐）。

