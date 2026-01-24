# offline_extract.py - 快速入门指南

## 概述

`offline_extract.py` 是 Apollo Cyber RT 录制文件的数据提取工具。它从 `.record` 文件中读取数据并输出指定字段，支持多通道和多字段组合。

## 前置条件

- Python 3 环境
- Cyber RT python 模块可用
- Apollo 录制文件（`.record` 格式）

## 安装

无需安装，脚本位于：
```
modules/tools/whl-debug/offline_extract.py
```

## 基本用法

### 1. 列出所有支持的通道

```bash
python modules/tools/whl-debug/offline_extract.py --list-channels
```

输出：
```
Supported channels (9):
  /apollo/canbus/chassis -> Chassis
  /apollo/canbus/chassis_detail -> ChassisDetail
  /apollo/localization/pose -> LocalizationEstimate
  /apollo/planning -> ADCTrajectory
  /apollo/hmi/status -> HMIStatus
  /apollo/control -> ControlCommand
  /apollo/prediction -> PredictionObstacles
  /apollo/perception/obstacles -> PerceptionObstacles
  /apollo/routing_request -> RoutingRequest
  /apollo/routing_response -> RoutingResponse
```

### 2. 从单个通道提取单个字段

从录制文件提取底盘速度：
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f speed_ms
```

输出：
```
[/apollo/canbus/chassis][1769155734637735350] 2.45
[/apollo/canbus/chassis][1769155734722863806] 2.52
[/apollo/canbus/chassis][1769155734822936175] 2.48
```

### 3. 提取多个字段（CSV 格式）

提取底盘速度和转向百分比：
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f speed_ms \
  -f steering_percentage \
  --output-format csv
```

输出：
```
channel,ts,speed_ms,steering_percentage
/apollo/canbus/chassis,1769155734637735350,2.45,-5.2
/apollo/canbus/chassis,1769155734722863806,2.52,-5.1
```

### 4. 从多个通道提取数据

提取底盘速度和规划决策：
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  -c /apollo/planning -f decision
```

### 5. 输出完整消息

使用 `-f .` 输出完整消息：
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f .
```

### 6. 限制消息数量

只提取 100 条消息：
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  -n 100
```

### 7. Porcelain 模式（仅输出数据）

抑制日志输出，只打印数据：
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  --porcelain
```

## 输出格式

### 文本格式（默认）
```
[/apollo/canbus/chassis][timestamp] value1 | value2
```

### CSV 格式
```bash
--output-format csv
```
输出：
```
channel,ts,field1,field2
/apollo/channel,1234567890,1.0,2.0
```

### JSON 格式
```bash
--output-format json
```
输出：
```json
{"channel":"/apollo/canbus/chassis","ts":1234567890,"speed_ms":2.45}
```

## 常见使用场景

### 分析底盘数据
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f speed_ms -f steering_percentage -f throttle -f brake \
  --output-format csv > chassis_analysis.csv
```

### 调试规划输出
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/planning \
  -f decision -f trajectory_type
```

### 提取定位数据
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/localization/pose \
  -f pose.position.x -f pose.position.y -f pose.heading
```

### 多通道 CSV 导出
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms -f throttle \
  -c /apollo/planning -f decision \
  --output-format csv --porcelain > output.csv
```

## 输入目录

`-i` 选项指定包含录制文件的目录：
```
/path/to/record_dir/
├── record.00000
├── record.00001
└── record.00002
```

文件按字母顺序处理。

## 使用技巧

1. 将输出重定向到文件时使用 `--porcelain` 模式
2. 使用 CSV 格式便于在 Excel/pandas 中分析
3. 工具处理输入目录中的所有 `.record` 文件
4. 使用 `-n` 限制处理数量以快速测试
5. 每个 `-c` 开始新的通道组，后续的 `-f` 应用于该通道

## 与 online_extract.py 的区别

| 特性 | online_extract.py | offline_extract.py |
|------|-------------------|-------------------|
| 数据源 | Cyber RT（实时） | 录制文件 |
| 需要 Cyber RT 运行 | 是 | 否 |
| 使用场景 | 实时监控 | 离线分析 |
| 输入选项 | 无 | `-i`（目录） |
