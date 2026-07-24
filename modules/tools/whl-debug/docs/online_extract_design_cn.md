# online_extract.py - 设计文档

## 目录

1. [架构概述](#架构概述)
2. [组件设计](#组件设计)
3. [数据流](#数据流)
4. [算法详解](#算法详解)
5. [配置](#配置)
6. [扩展点](#扩展点)

## 架构概述

```
┌─────────────────────────────────────────────────────────────┐
│                        online_extract.py                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │   CLI       │───▶│    Online    │───▶│  Channel     │   │
│  │   Parser    │    │   Extractor  │    │  Extractors  │   │
│  └─────────────┘    └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            ▼                    ▼            │
│                     ┌──────────────┐    ┌──────────────┐   │
│                     │   Callback   │    │  Message     │   │
│                     │   Handler    │    │  Processor   │   │
│                     └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            └────────┬───────────┘            │
│                                     ▼                        │
│                            ┌──────────────┐                 │
│                            │   Output     │                 │
│                            │   Formatter  │                 │
│                            └──────────────┘                 │
└─────────────────────────────────────────────────────────────┘
                           │         │
                           ▼         ▼
                    ┌──────────┐ ┌──────────┐
                    │  Cyber  │ │  stdout/ │
                    │    RT   │ │   file   │
                    └──────────┘ └──────────┘
```

## 组件设计

### 1. CLI 解析器

**职责**：解析命令行参数并构建通道配置。

**核心逻辑**：
```python
# 解析 sys.argv 构建通道到字段的映射
for each -c option:
    创建新的通道条目，字段列表为空
for each -f option:
    追加到最近一个通道的字段列表
```

**输出**：`List[Tuple[str, List[str]]]` - (channel, fields) 元组列表

### 2. Online Extractor

**职责**：主控制器，管理 Cyber RT 生命周期并协调提取器。

**关键属性**：
- `channel_configs`：(channel, fields) 元组列表
- `all_csv_fields`：CSV 表头的统一字段列表
- `extractors`：ChannelExtractor 实例列表

**关键方法**：
- `_calculate_all_fields()`：计算所有通道的唯一字段名
- `_print_header()`：如需要则打印 CSV 表头
- `start()`：初始化 Cyber RT 并创建读者
- `stop()`：清理资源

### 3. Channel Extractor

**职责**：处理单个通道的消息。

**关键属性**：
- `channel`：通道名称
- `fields`：要提取的字段路径列表
- `all_csv_fields`：统一字段列表（用于 CSV 对齐）
- `msg_type_name`：用于显示的消息类型名

**关键方法**：
- `callback(data)`：处理接收到的消息
- `_should_output_root()`：检查是否应输出完整消息
- `get_display_fields()`：获取人类可读的字段描述

### 4. Message Processor

**职责**：从 protobuf 消息中提取嵌套字段。

**函数**：`get_nested_field(msg, field_path)`

**算法**：
```python
def get_nested_field(msg, field_path):
    data = MessageToDict(msg, preserving_proto_field_name=True)
    for part in field_path.split('.'):
        if part in data:
            data = data[part]
        else:
            return None
    return data
```

### 5. Output Formatter

**职责**：根据输出格式格式化提取的数据。

**格式**：
- **文本**：`[channel][ts] value1 | value2`
- **CSV**：`channel,ts,field1,field2`
- **JSON**：`{"channel": "...", "ts": ..., "field": ...}`

## 数据流

```
用户输入 (CLI)
       │
       ▼
┌─────────────────┐
│  解析参数        │
│  构建配置        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 创建提取器      │
│ 创建读者        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  轮询循环        │◀─────────────────┐
│  (Poll RT)      │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 接收消息        │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 查找提取器      │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 提取字段        │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 格式化输出      │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 打印到标准输出  │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 检查计数        │                  │
│ (如果指定了 -n) │─────────────────┘
└────────┬────────┘
         │
         ▼
    [继续/停止]
```

## 算法详解

### 通道-字段关联算法

主要设计挑战是在使用多个 `-c` 和 `-f` 选项时，将字段与正确的通道关联。

**算法**：
```python
def parse_channel_field_config(channels, fields, sys_argv):
    configs = []  # [(channel, [fields])]
    channel_idx = 0
    field_idx = 0

    i = 1  # 跳过脚本名称
    while i < len(sys_argv):
        arg = sys_argv[i]
        if arg in ("-c", "--channel"):
            # 新通道
            if channel_idx < len(channels):
                configs.append([channels[channel_idx], []])
                channel_idx += 1
            i += 2  # 跳过选项和值
        elif arg in ("-f", "--field"):
            # 最近一个通道的字段
            if configs and field_idx < len(fields):
                configs[-1][1].append(fields[field_idx])
                field_idx += 1
            i += 2  # 跳过选项和值
        else:
            i += 1

    # 空字段默认为 '.'
    return [(ch, flds if flds else ["."]) for ch, flds in configs]
```

### CSV 表头统一算法

对于 CSV 输出，所有通道必须共享相同的表头，包含所有通道的所有字段。

**算法**：
```python
def calculate_all_fields(channel_configs):
    all_fields = []
    for channel, fields in channel_configs:
        msg_type = CHANNEL_MESSAGE_TYPE_MAP.get(channel)
        type_name = msg_type.__name__ if msg_type else "Unknown"

        for f in fields:
            if f in ("", "."):
                # 根级别输出使用消息类型名
                field_name = type_name
            else:
                field_name = f

            if field_name not in all_fields:
                all_fields.append(field_name)

    return all_fields
```

### 字段值映射算法

输出 CSV 时，每个通道必须将其字段映射到统一表头。

**算法**：
```python
def map_fields_to_header(fields, msg_type_name, all_csv_fields):
    # 构建显示名到原始字段路径的映射
    field_map = {}
    for f in fields:
        if f in ("", "."):
            field_map[msg_type_name] = f
        else:
            field_map[f] = f

    # 按表头顺序输出值
    out_values = []
    for display_name in all_csv_fields:
        if display_name in field_map:
            original_field = field_map[display_name]
            if original_field in ("", "."):
                # 输出完整消息为 JSON
                out_values.append(json.dumps(msg_dict))
            else:
                # 提取嵌套字段
                value = get_nested_field(msg, original_field)
                out_values.append(format_value(value))
        else:
            # 字段属于其他通道
            out_values.append("")

    return out_values
```

## 配置

### 通道-消息类型映射

```python
CHANNEL_MESSAGE_TYPE_MAP = {
    "/apollo/canbus/chassis": Chassis,
    "/apollo/canbus/chassis_detail": ChassisDetail,
    "/apollo/localization/pose": LocalizationEstimate,
    "/apollo/planning": ADCTrajectory,
    "/apollo/hmi/status": HMIStatus,
    "/apollo/control": ControlCommand,
    "/apollo/prediction": PredictionObstacles,
    "/apollo/perception/obstacles": PerceptionObstacles,
    "/apollo/routing_request": RoutingRequest,
    "/apollo/routing_response": RoutingResponse,
}
```

### 默认值

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `output_format` | `text` | 输出格式 |
| `separator` | `,` | CSV 分隔符 |
| `count` | `-1` | 处理所有消息 |
| `log_level` | `INFO` | 日志级别 |

## 扩展点

### 添加新通道

添加对新通道的支持：

1. 导入消息类型：
```python
from xxx_msgs/xxx_pb2 import XxxMessage
```

2. 添加到 `CHANNEL_MESSAGE_TYPE_MAP`：
```python
CHANNEL_MESSAGE_TYPE_MAP = {
    ...
    "/apollo/new/channel": XxxMessage,
}
```

### 添加新输出格式

1. 在 `--output-format` 选项中添加格式
2. 在 `ChannelExtractor.callback()` 中实现格式化逻辑

### 添加自定义字段处理器

扩展 `ChannelExtractor` 添加自定义字段处理逻辑：

```python
class CustomChannelExtractor(ChannelExtractor):
    def process_field(self, value, field_path):
        # 自定义处理逻辑
        return transformed_value
```

## 性能考虑

1. **消息转换**：每条消息都调用 `MessageToDict`。对于高频通道，考虑缓存。
2. **字段提取**：嵌套字段访问每次都要遍历字典。扁平字段更快。
3. **输出格式化**：JSON 由于序列化开销，比 text/CSV 慢。

## 线程模型

- 单线程设计
- Cyber RT 回调在 Cyber RT 的线程池中执行
- 提取器之间无共享可变状态
- 通过 `click.echo()` 保证线程安全输出
