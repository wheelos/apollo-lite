# online_extract.py - 快速入门指南

## 概述

`online_extract.py` 是 Apollo Cyber RT 的实时数据提取工具。它订阅 Cyber RT 通道并实时输出指定字段，支持多通道和多字段组合。

## 前置条件

- Apollo Cyber RT 环境正在运行
- Python 3 环境已安装 cyber_py3 模块
- 相关通道正在发布数据

## 安装

无需安装，脚本位于：
```
modules/tools/whl-debug/online_extract.py
```

## 基本用法

### 1. 列出所有支持的通道

```bash
python modules/tools/whl-debug/online_extract.py --list-channels
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

提取底盘速度：
```bash
python modules/tools/whl-debug/online_extract.py -c /apollo/canbus/chassis -f speed_ms
```

输出：
```
[/apollo/canbus/chassis][1769155734.63773] 2.45
[/apollo/canbus/chassis][1769155734.72860] 2.52
[/apollo/canbus/chassis][1769155734.81947] 2.48
```

### 3. 提取多个字段（CSV 格式）

提取底盘速度和转向百分比：
```bash
python modules/tools/whl-debug/online_extract.py \
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
/apollo/canbus/chassis,1769155734822936175,2.48,-5.3
```

### 4. 从多个通道提取数据

提取底盘速度和规划决策：
```bash
python modules/tools/whl-debug/online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms \
  -c /apollo/planning -f decision
```

### 5. 输出完整消息

使用 `-f .` 或省略 `-f` 来输出完整消息：
```bash
python modules/tools/whl-debug/online_extract.py \
  -c /apollo/canbus/chassis \
  -f .
```

### 6. 限制消息数量

只提取 10 条消息：
```bash
python modules/tools/whl-debug/online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms \
  -n 10
```

### 7. Porcelain 模式（仅输出数据）

抑制日志输出，只打印数据：
```bash
python modules/tools/whl-debug/online_extract.py \
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

### 监控底盘状态
```bash
python modules/tools/whl-debug/online_extract.py \
  -c /apollo/canbus/chassis \
  -f speed_ms -f steering_percentage -f throttle -f brake
```

### 监控规划输出
```bash
python modules/tools/whl-debug/online_extract.py \
  -c /apollo/planning \
  -f decision -f trajectory_type
```

### 监控定位信息
```bash
python modules/tools/whl-debug/online_extract.py \
  -c /apollo/localization/pose \
  -f pose.position.x -f pose.position.y -f pose.heading
```

### 多通道 CSV 导出
```bash
python modules/tools/whl-debug/online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f throttle \
  -c /apollo/planning -f decision \
  --output-format csv --porcelain > output.csv
```

## 使用技巧

1. 将输出重定向到文件时使用 `--porcelain` 模式
2. 使用 CSV 格式便于在 Excel/pandas 中分析
3. 按 `Ctrl+C` 停止提取器
4. 每个 `-c` 开始新的通道组，后续的 `-f` 应用于该通道
