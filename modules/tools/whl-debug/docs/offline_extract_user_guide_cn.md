# offline_extract.py - 用户指南

## 目录

1. [简介](#简介)
2. [命令行选项](#命令行选项)
3. [通道和字段语法](#通道和字段语法)
4. [输出格式](#输出格式)
5. [高级用法](#高级用法)
6. [示例](#示例)
7. [故障排除](#故障排除)

## 简介

`offline_extract.py` 是 Apollo Cyber RT 录制文件的数据提取工具。它从 `.record` 文件中读取数据并提取指定字段，支持多种输出格式用于事后分析和调试。

### 核心特性

- **多通道支持**：一条命令从多个通道提取数据
- **多字段提取**：每个通道可提取多个字段
- **批量处理**：处理整个目录的录制文件
- **灵活的输出格式**：文本、CSV 或 JSON
- **灵活的字段语法**：支持点号表示法访问嵌套字段

## 命令行选项

### 必需选项

| 选项 | 简写 | 描述 |
|------|------|------|
| `--input-dir` | `-i` | 包含录制文件的目录路径（必需） |

### 基本选项

| 选项 | 简写 | 描述 |
|------|------|------|
| `--channel` | `-c` | Cyber RT 通道名称（可多次指定） |
| `--field` | `-f` | 要提取的字段路径（可多次指定） |
| `--count` | `-n` | 要处理的消息数量（默认：-1 表示全部） |

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

通道和字段语法与 `online_extract.py` 相同。详细信息请参考 [online_extract_user_guide_cn.md](online_extract_user_guide_cn.md)。

### 通道指定

每个 `-c` 选项开始一个新的通道组。后续的 `-f` 选项应用于最近的 `-c`。

### 字段路径语法

使用点号表示法访问嵌套字段：

| 字段路径 | 描述 |
|----------|------|
| `speed_ms` | 顶层字段 |
| `pose.position.x` | 嵌套字段（3 层） |
| `header.sequence_num` | 带下划线的嵌套字段 |
| `.` | 根层级（整个消息） |

## 输出格式

输出格式与 `online_extract.py` 相同。详细信息请参考 [online_extract_user_guide_cn.md](online_extract_user_guide_cn.md)。

### 文本格式（默认）
```
[/apollo/canbus/chassis][1769155734637735350] 2.45 | -5.2 | 12.3
```

### CSV 格式
```
channel,ts,speed_ms,steering_percentage,throttle
/apollo/canbus/chassis,1769155734637735350,2.45,-5.2,12.3
```

### JSON 格式
```json
{"channel":"/apollo/canbus/chassis","ts":1769155734637735350,"speed_ms":2.45}
```

## 高级用法

### 处理多个录制文件

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms
```

目录中的所有 `.record` 文件按字母顺序处理。

### 多通道分析

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms -f throttle \
  -c /apollo/planning -f decision \
  --output-format csv
```

### 限制处理以快速测试

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  -n 10
```

### 导出用于数据分析

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms -f steering_percentage \
  -c /apollo/control -f throttle -f brake \
  --output-format csv --porcelain \
  > analysis_data.csv
```

### 自定义 CSV 分隔符

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  --output-format csv --separator "\t"
```

## 示例

### 示例 1：提取车辆速度

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/canbus/chassis -f speed_ms
```

### 示例 2：分析控制命令

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/control \
  -f throttle -f brake -f steering_rate \
  --output-format csv --porcelain \
  > control_analysis.csv
```

### 示例 3：多通道调试

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/canbus/chassis -f speed_ms -f gear_location \
  -c /apollo/planning -f decision \
  -c /apollo/control -f throttle -f brake
```

### 示例 4：定位轨迹

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/localization/pose \
  -f pose.position.x \
  -f pose.position.y \
  -f pose.heading \
  --output-format csv
```

### 示例 5：提取嵌套规划数据

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/planning \
  -f trajectory_point.0.path_point.x \
  -f trajectory_point.0.path_point.y \
  -f trajectory_point.0.path_point.theta
```

## 故障排除

### 问题："No record files found in directory"

**原因**：指定目录中没有 `.record` 文件。

**解决方案**：
1. 验证目录路径正确
2. 检查文件是否存在：`ls -la /path/to/record_dir/`
3. 确保文件扩展名为 `.record`

### 问题："Channel not in predefined mapping"

**原因**：通道不在 `CHANNEL_MESSAGE_TYPE_MAP` 中。

**解决方案**：
1. 使用 `--list-channels` 查看可用通道
2. 在脚本的 `CHANNEL_MESSAGE_TYPE_MAP` 中添加通道和消息类型

### 问题：字段返回 "N/A" 或为空

**原因**：字段路径不正确或字段在消息中不存在。

**解决方案**：
1. 使用 `-f .` 输出完整消息并检查可用字段
2. 验证字段路径语法（嵌套字段使用点号）
3. 检查录制文件中该通道是否有数据

### 问题："Failed to parse message"

**原因**：消息类型不匹配或数据损坏。

**解决方案**：
1. 验证 `MESSAGE_TYPE_MAP` 中的通道映射
2. 检查录制文件是否有效

### 问题：处理速度慢

**原因**：录制文件过大或通道太多。

**解决方案**：
1. 使用 `-n` 限制处理数量进行测试
2. 只处理特定通道而不是全部
3. 使用 `--output-format csv` 比 JSON 处理更快

## 录制文件格式

工具期望 Apollo Cyber RT `.record` 文件。这些是包含序列化 protobuf 消息的二进制文件，带有通道头部。

```
/path/to/record_dir/
├── record.00000
├── record.00001
└── record.00002
```

文件按字母顺序处理（`record.00000`、`record.00001`...）。

## 支持的通道

完整支持通道列表请参考 [online_extract_user_guide_cn.md](online_extract_user_guide_cn.md#支持的通道)。

## 与 online_extract.py 的对比

| 特性 | online_extract.py | offline_extract.py |
|------|-------------------|-------------------|
| **数据源** | 实时 Cyber RT | 录制文件 |
| **需要 Cyber RT** | 是（运行中） | 否（仅 python 模块） |
| **使用场景** | 实时监控 | 事后分析 |
| **输入** | 无 | `-i`（目录） |
| **处理** | 持续进行 | 有限（处理完成后结束） |
| **消息数量** | `-n` 后停止 | `-n` 或全部消息 |
| **依赖** | cyber.PyNode | RecordReader |

## 典型工作流程

1. **录制数据**：使用 `cyber_recorder` 录制数据
   ```bash
   cyber_recorder record -a -o record_dir
   ```

2. **提取数据**：使用 `offline_extract.py` 提取字段
   ```bash
   python offline_extract.py -i record_dir -c /apollo/canbus/chassis -f speed_ms
   ```

3. **分析数据**：导入到分析工具
   ```bash
   python offline_extract.py -i record_dir -c /apollo/canbus/chassis -f speed_ms \
     --output-format csv --porcelain > data.csv
   # 在 pandas、Excel 等中分析
   ```
