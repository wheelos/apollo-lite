# online_extract.py - 用户指南

## 目录

1. [简介](#简介)
2. [命令行选项](#命令行选项)
3. [通道和字段语法](#通道和字段语法)
4. [输出格式](#输出格式)
5. [高级用法](#高级用法)
6. [示例](#示例)
7. [故障排除](#故障排除)

## 简介

`online_extract.py` 是 Apollo Cyber RT 的实时数据提取工具。它同时订阅多个通道并提取指定字段，支持多种输出格式，用于数据分析和调试。

### 核心特性

- **多通道支持**：一条命令订阅多个通道
- **多字段提取**：每个通道可提取多个字段
- **实时输出**：支持文本、CSV 或 JSON 格式
- **灵活的字段语法**：支持点号表示法访问嵌套字段
- **完整消息输出**：需要时可输出完整消息

## 命令行选项

### 基本选项

| 选项 | 简写 | 描述 |
|------|------|------|
| `--channel` | `-c` | Cyber RT 通道名称（可多次指定） |
| `--field` | `-f` | 要提取的字段路径（可多次指定） |
| `--count` | `-n` | 要处理的消息数量（默认：-1 表示无限） |

### 输出选项

| 选项 | 描述 |
|------|------|
| `--output-format` | 输出格式：`text`、`csv` 或 `json`（默认：text） |
| `--separator` | CSV 分隔符（默认：","） |
| `--porcelain` | 抑制所有日志输出，仅打印数据 |

### 工具选项

| 选项 | 描述 |
|------|------|
| `--list-channels` | 列出所有支持的通道并退出 |
| `--log-level` | 日志级别：`DEBUG`、`INFO`、`WARNING`、`ERROR`（默认：INFO） |

## 通道和字段语法

### 通道指定

每个 `-c` 选项开始一个新的通道组。后续的 `-f` 选项应用于最近的 `-c`。

```
-c <通道1> -f <字段1> -f <字段2> -c <通道2> -f <字段3>
```

含义：
- `<通道1>` 提取 `<字段1>` 和 `<字段2>`
- `<通道2>` 提取 `<字段3>`

### 字段路径语法

使用点号表示法访问嵌套字段：

| 字段路径 | 描述 |
|----------|------|
| `speed_ms` | 顶层字段 |
| `pose.position.x` | 嵌套字段（3 层） |
| `header.sequence_num` | 带下划线的嵌套字段 |
| `.` | 根层级（整个消息） |

### 特殊字段值

- `.` 或空字符串：输出整个消息
- 省略 `-f`：等同于指定 `-f .`

## 输出格式

### 文本格式（默认）

```
[/apollo/canbus/chassis][1769155734.63773] 2.45 | -5.2 | 12.3
```

格式：`[通道][时间戳] 值1 | 值2 | 值3`

### CSV 格式

```bash
--output-format csv
```

输出：
```
channel,ts,speed_ms,steering_percentage,throttle
/apollo/canbus/chassis,1769155734637735350,2.45,-5.2,12.3
/apollo/canbus/chassis,1769155734722863806,2.52,-5.1,12.5
```

特性：
- 统一的表头，包含所有通道的所有字段
- 不属于当前通道的字段为空值
- 所有通道保持一致的列顺序

### JSON 格式

```bash
--output-format json
```

输出：
```json
{"channel":"/apollo/canbus/chassis","ts":1769155734637735350,"speed_ms":2.45,"steering_percentage":-5.2}
{"channel":"/apollo/canbus/chassis","ts":1769155734722863806,"speed_ms":2.52,"steering_percentage":-5.1}
```

每行是一个完整的 JSON 对象。

## 高级用法

### 多通道不同字段

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f throttle \
  -c /apollo/planning -f decision \
  -c /apollo/localization/pose -f pose.position.x
```

### 混合：完整消息 + 特定字段

```bash
python online_extract.py \
  -c /apollo/canbus/chassis \
  -c /apollo/planning -f decision
```

输出：
- `/apollo/canbus/chassis` 的完整消息
- `/apollo/planning` 只有 `decision` 字段

### 自定义 CSV 分隔符

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms \
  --output-format csv --separator "\t"
```

### 数据采集用于分析

```bash
# 收集数据到 CSV 文件
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f throttle -f brake \
  --output-format csv --porcelain \
  -n 1000 > chassis_data.csv
```

### 调试模式

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms \
  --log-level DEBUG
```

## 示例

### 示例 1：监控车辆速度

```bash
python online_extract.py -c /apollo/canbus/chassis -f speed_ms
```

输出：
```
[/apollo/canbus/chassis][1769155734.63773] 2.45
[/apollo/canbus/chassis][1769155734.72860] 2.52
[/apollo/canbus/chassis][1769155734.81947] 2.48
```

### 示例 2：采集控制数据

```bash
python online_extract.py \
  -c /apollo/control \
  -f throttle -f brake -f steering_rate \
  --output-format csv --porcelain \
  -n 500 > control_data.csv
```

### 示例 3：多通道系统调试

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f gear_location \
  -c /apollo/planning -f decision \
  -c /apollo/control -f throttle -f brake
```

### 示例 4：定位位置跟踪

```bash
python online_extract.py \
  -c /apollo/localization/pose \
  -f pose.position.x \
  -f pose.position.y \
  -f pose.heading
```

### 示例 5：提取嵌套规划数据

```bash
python online_extract.py \
  -c /apollo/planning \
  -f trajectory_point.0.path_point.x \
  -f trajectory_point.0.path_point.y
```

## 故障排除

### 问题："Channel not in predefined mapping"

**原因**：通道不在 `CHANNEL_MESSAGE_TYPE_MAP` 中。

**解决方案**：
1. 使用 `--list-channels` 查看可用通道
2. 在脚本的 `CHANNEL_MESSAGE_TYPE_MAP` 中添加通道和消息类型

### 问题：没有数据输出

**可能原因**：
1. 通道未发布数据
2. Cyber RT 未运行
3. 通道名称错误

**解决方案**：
1. 检查 Cyber RT 是否运行：`cyber_monitor`
2. 使用 `--list-channels` 验证通道名称
3. 启用调试日志：`--log-level DEBUG`

### 问题：字段返回 "N/A"

**原因**：字段路径不正确或字段在消息中不存在。

**解决方案**：
1. 使用 `-f .` 输出完整消息并检查可用字段
2. 验证字段路径语法（嵌套字段使用点号）

### 问题：CSV 有空列

**原因**：从多个通道提取时，每个通道只有自己的字段。

**解决方案**：这是预期行为。不属于某个通道的字段将为空。

## 支持的通道

| 通道 | 消息类型 | 常用字段 |
|------|----------|----------|
| `/apollo/canbus/chassis` | Chassis | speed_ms, throttle, brake, steering_percentage, gear_location |
| `/apollo/localization/pose` | LocalizationEstimate | pose.position.x, pose.position.y, pose.heading |
| `/apollo/planning` | ADCTrajectory | decision, trajectory_type, trajectory_point |
| `/apollo/control` | ControlCommand | throttle, brake, steering_rate |
| `/apollo/perception/obstacles` | PerceptionObstacles | obstacle, timestamp |
| `/apollo/prediction` | PredictionObstacles | obstacle, timestamp |
