# offline_extract.py - 设计文档

## 目录

1. [架构概述](#架构概述)
2. [组件设计](#组件设计)
3. [数据流](#数据流)
4. [算法详解](#算法详解)
5. [录制文件格式](#录制文件格式)
6. [性能考虑](#性能考虑)

## 架构概述

```
┌─────────────────────────────────────────────────────────────┐
│                        offline_extract.py                    │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │   CLI       │───▶│    Offline   │───▶│  Channel     │   │
│  │   Parser    │    │   Extractor  │    │  Extractors  │   │
│  └─────────────┘    └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            ▼                    ▼            │
│                     ┌──────────────┐    ┌──────────────┐   │
│                     │  File Scanner│    │  Message     │   │
│                     │              │    │  Processor   │   │
│                     └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            ▼                    ▼            │
│                     ┌──────────────┐    ┌──────────────┐   │
│                     │   Record     │    │  Field       │   │
│                     │   Reader     │    │  Extractor   │   │
│                     └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            └────────┬───────────┘            │
│                                     ▼                        │
│                            ┌──────────────┐                 │
│                            │   Output     │                 │
│                            │   Formatter  │                 │
│                            └──────────────┘                 │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
                    ┌──────────┐
                    │  stdout/ │
                    │   file   │
                    └──────────┘
```

## 组件设计

### 1. CLI 解析器

**职责**：解析命令行参数并构建通道配置。

**核心逻辑**：与 `online_extract.py` 相同 - 解析 sys.argv 构建通道到字段的映射。

**输出**：`List[Tuple[str, List[str]]]` - (channel, fields) 元组列表

### 2. Offline Extractor

**职责**：主控制器，管理文件处理并协调提取器。

**关键属性**：
- `input_dir`：包含录制文件的目录
- `channel_configs`：(channel, fields) 元组列表
- `all_csv_fields`：CSV 表头的统一字段列表
- `extractors`：ChannelExtractor 实例列表

**关键方法**：
- `_get_record_files()`：扫描目录查找录制文件
- `_calculate_all_fields()`：计算所有通道的唯一字段名
- `_print_header()`：如需要则打印 CSV 表头
- `start()`：处理所有录制文件

### 3. Record Reader 封装

**职责**：从 Apollo Cyber RT 录制文件读取消息。

**关键属性**：
- 使用 `cyber.python.cyber_py3.record.RecordReader`
- 使用 `read_messages()` 迭代消息

**消息格式**：
```python
# RecordReader.read_messages() 返回的每条消息：
PyBagMessage(
    topic=str,        # 通道名称
    message=bytes,    # 原始 protobuf 字节
    data_type=str,    # 消息类型名称
    timestamp=int     # 纳秒级时间戳
)
```

### 4. Message Parser

**职责**：使用正确的消息类型解析原始消息字节。

**核心逻辑**：
```python
def parse_message(raw_bytes, msg_type):
    msg = msg_type()
    msg.ParseFromString(raw_bytes)
    return msg
```

### 5. Channel Extractor

**职责**：处理单个通道的消息。

与 `online_extract.py` 相同，但具有额外的消息解析能力。

**与 online_extract.py 的主要区别**：
- 接收原始字节而不是已解析的消息
- 必须使用 `MESSAGE_TYPE_MAP` 解析消息

## 数据流

```
用户输入 (CLI + -i 目录)
       │
       ▼
┌─────────────────┐
│  解析参数        │
│  扫描目录        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 创建提取器      │
│ 获取录制文件    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 遍历每个文件    │◀─────────────────┐
│  创建读者        │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 读取消息        │                  │
│ (RecordReader)  │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ 解析原始字节    │                  │
│ (msg.ParseFrom) │                  │
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
    [下一条消息/下一个文件/完成]
```

## 算法详解

### 文件扫描算法

```python
def get_record_files(input_dir):
    files = [
        os.path.join(input_dir, x)
        for x in os.listdir(input_dir)
        if os.path.isfile(os.path.join(input_dir, x))
    ]
    files.sort()  # 字母顺序
    return files
```

### 消息处理循环

```python
def process_files(record_files, channel_extractor_map):
    for file in record_files:
        reader = RecordReader(file)
        available_channels = reader.get_channellist()

        for msg in reader.read_messages():
            topic, raw_data, data_type, timestamp = msg

            # 只处理我们感兴趣的通道
            if topic not in channels_to_read:
                continue

            extractor = channel_extractor_map[topic]
            extractor.process_message(raw_data, timestamp)

            # 检查计数限制
            if target_count > 0 and total_processed >= target_count:
                return
```

### 通道-提取器映射

```python
def build_channel_extractor_map(extractors):
    return {
        extractor.channel: extractor
        for extractor in extractors
    }
```

## 录制文件格式

Apollo Cyber RT 录制文件使用以下格式：

### 文件结构

```
┌──────────────────────────────────────┐
│           文件头部                    │
├──────────────────────────────────────┤
│  ┌────────────────────────────────┐  │
│  │  通道头部                       │  │
│  ├────────────────────────────────┤  │
│  │  消息 1                         │  │
│  ├────────────────────────────────┤  │
│  │  消息 2                         │  │
│  └────────────────────────────────┘  │
│  ┌────────────────────────────────┐  │
│  │  通道头部                       │  │
│  ├────────────────────────────────┤  │
│  │  消息 1                         │  │
│  └────────────────────────────────┘  │
│            ...                       │
└──────────────────────────────────────┘
```

### RecordReader API

```python
# 创建读者
reader = RecordReader(file_path)

# 获取可用通道
channels = reader.get_channellist()
# 返回: ['/apollo/canbus/chassis', '/apollo/planning', ...]

# 读取消息
for msg in reader.read_messages():
    # msg.topic: 通道名称 (str)
    # msg.message: 原始 protobuf 字节 (bytes)
    # msg.data_type: 消息类型名称 (str)
    # msg.timestamp: 纳秒 (int)
    pass

# 获取通道的消息数量
count = reader.get_messagenumber(channel_name)

# 获取通道的消息类型
msg_type = reader.get_messagetype(channel_name)
```

## 性能考虑

### 内存使用

1. **消息解析**：每条消息单独解析并丢弃
2. **无消息缓存**：消息不存储在内存中
3. **流式输出**：输出立即写入，不缓冲

### 处理速度

1. **文件 I/O**：顺序文件读取效率高
2. **Protobuf 解析**：`ParseFromString()` 对大多数消息很快
3. **JSON 序列化**：比 text/CSV 慢（用于复杂消息）

### 优化建议

1. **使用 `-n` 测试**：先处理消息子集
2. **限制通道**：只提取需要的通道
3. **使用 CSV 格式**：大数据集比 JSON 更快
4. **使用 `--porcelain`**：跳过日志开销

## 与 online_extract.py 的设计差异

| 方面 | online_extract.py | offline_extract.py |
|------|-------------------|-------------------|
| **消息来源** | Cyber RT 回调 | RecordReader 迭代器 |
| **消息格式** | 已解析的 protobuf | 原始字节 |
| **解析** | 自动 | 手动 `ParseFromString()` |
| **生命周期** | 持续（直到 Ctrl+C） | 有限（处理完成后结束） |
| **线程** | 多线程（Cyber RT） | 单线程 |
| **状态管理** | 节点管理 | 文件管理 |

## 错误处理

### 文件级错误

- 目录缺失：错误消息并退出
- 无录制文件：警告消息并退出
- 录制文件损坏：记录错误，跳到下一个文件

### 消息级错误

- 解析错误：记录错误，跳过消息
- 缺少消息类型：记录警告，跳过消息
- 通道不在映射中：记录警告，跳过消息

### 恢复策略

工具在非致命错误后继续处理：
- 跳过损坏的消息
- 文件级错误时继续下一个文件
- 记录所有错误用于事后分析

## 扩展点

与 `online_extract.py` 相同：

1. **添加新通道**：更新 `MESSAGE_TYPE_MAP`
2. **添加输出格式**：扩展 `ChannelExtractor.process_message()`
3. **自定义字段处理**：子类化 `ChannelExtractor`
