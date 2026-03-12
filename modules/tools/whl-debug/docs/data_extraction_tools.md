# 数据提取工具介绍

## 概述

Apollo 自动驾驶系统运行时会产生大量的实时数据，包括底盘状态、规划轨迹、定位信息、感知障碍物等。这些数据通过 Cyber RT 框架在各个模块之间传递，对于系统调试、性能分析和问题诊断至关重要。

本数据提取工具包含两个核心组件：

- **online_extract.py** - 在线数据提取工具，从运行的 Cyber RT 系统中实时提取数据
- **offline_extract.py** - 离线数据提取工具，从录制文件中提取历史数据

## 应用场景

### 1. 高频数据异常检测

**问题**：使用 `cyber_monitor` 观察 Cyber RT 通道时，对于高频发布的数据（如 100Hz 的底盘控制信号），肉眼无法捕捉瞬时异常值。

**解决方案**：使用数据提取工具将数据导出为 CSV 文件，通过数据分析工具（如 Excel、pandas）进行统计分析：
- 识别数据跳变
- 检测异常值
- 分析数据分布

### 2. 关键参数记录与追踪

**问题**：复现问题时需要知道特定时刻的车辆状态、规划决策等关键参数。

**解决方案**：
- **在线模式**：实时监控并记录关键参数到日志
- **离线模式**：从录制文件中提取完整时间序列数据

### 3. 多通道关联分析

**问题**：某些问题涉及多个模块的协同，需要同时查看多个通道的数据。

**解决方案**：工具支持同时提取多个通道的数据，在 CSV 输出中自动对齐时间戳，方便进行关联分析。

### 4. 算法性能评估

**问题**：评估控制算法、规划算法的性能需要定量分析。

**解决方案**：导出控制命令、跟踪误差等数据，进行定量分析（RMSE、最大误差、响应时间等）。

## 核心功能

### 多通道支持

支持同时从多个 Cyber RT 通道提取数据：

```bash
# 同时提取底盘、规划和控制数据
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f steering_percentage \
  -c /apollo/planning -f decision \
  -c /apollo/control -f throttle -f brake
```

### 嵌套字段提取

使用点号表示法提取嵌套字段：

```bash
# 提取嵌套的位置信息
python online_extract.py \
  -c /apollo/localization/pose \
  -f pose.position.x -f pose.position.y -f pose.heading
```

### 多种输出格式

- **文本格式**：适合终端实时查看
- **CSV 格式**：适合导入 Excel、pandas 进行分析
- **JSON 格式**：适合程序化处理

### 统一表头输出

CSV 输出时，所有通道共享统一的表头，不属于当前通道的字段自动填充空值，便于数据对齐和分析。

## 工具对比

| 特性 | online_extract.py | offline_extract.py |
|------|-------------------|-------------------|
| **数据源** | Cyber RT（实时） | 录制文件 |
| **使用场景** | 实时监控、调试 | 事后分析、回放 |
| **需要 Cyber RT 运行** | 是 | 否 |
| **处理模式** | 持续输出（Ctrl+C 停止） | 处理完成后自动退出 |
| **输入** | 通道列表 | 目录 + 通道列表 |
| **典型工作流** | 启动工具 -> 观察输出 | 选择录制 -> 提取 -> 分析 |

## 使用示例

### 场景 1：实时监控底盘状态

```bash
# 实时查看速度、油门、刹车
python online_extract.py \
  -c /apollo/canbus/chassis \
  -f speed_ms -f throttle -f brake
```

### 场景 2：导出控制数据进行分析

```bash
# 导出为 CSV，用于 Excel 分析
python online_extract.py \
  -c /apollo/control \
  -f throttle -f brake -f steering_rate \
  --output-format csv --porcelain \
  -n 1000 > control_data.csv
```

### 场景 3：从录制文件提取定位轨迹

```bash
# 从历史录制中提取定位数据
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/localization/pose \
  -f pose.position.x -f pose.position.y -f pose.heading \
  --output-format csv
```

### 场景 4：多通道关联分析

```bash
# 同时提取多个通道，分析关联关系
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/canbus/chassis -f speed_ms \
  -c /apollo/control -f throttle -f brake \
  -c /apollo/planning -f decision \
  --output-format csv --porcelain > multi_channel.csv
```

## 数据分析工作流

### 典型的数据分析流程

```
┌─────────────────┐
│  1. 数据采集     │
│  (cyber_recorder)│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  2. 数据提取     │
│  (offline_extract)│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  3. 数据分析     │
│  (pandas/Excel) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  4. 问题诊断     │
│  /性能优化       │
└─────────────────┘
```

### 示例：使用 pandas 分析数据

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取导出的数据
df = pd.read_csv('control_data.csv')

# 绘制油门和刹车曲线
plt.figure(figsize=(12, 6))
plt.plot(df['throttle'], label='Throttle')
plt.plot(df['brake'], label='Brake')
plt.legend()
plt.savefig('control_analysis.png')

# 统计分析
print(f"油门平均值: {df['throttle'].mean()}")
print(f"油门最大值: {df['throttle'].max()}")
print(f"刹车使用次数: {(df['brake'] > 0).sum()}")
```

## 高级特性

### Porcelain 模式

`--porcelain` 选项抑制所有日志输出，只输出数据，适合数据采集：

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms \
  --porcelain > raw_data.txt
```

### 消息数量限制

使用 `-n` 选项限制提取的消息数量，适合快速测试：

```bash
# 只提取前 100 条消息
python offline_extract.py \
  -i record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  -n 100
```

### 自定义分隔符

自定义 CSV 分隔符：

```bash
# 使用制表符分隔
python offline_extract.py \
  -i record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  --output-format csv --separator "\t"
```

## 文档索引

详细使用文档请参考：

### 快速入门
- [online_extract.py 快速入门](online_extract_quick_start_cn.md)
- [offline_extract.py 快速入门](offline_extract_quick_start_cn.md)

### 用户指南
- [online_extract.py 用户指南](online_extract_user_guide_cn.md)
- [offline_extract.py 用户指南](offline_extract_user_guide_cn.md)

### 设计文档
- [online_extract.py 设计文档](online_extract_design_cn.md)
- [offline_extract.py 设计文档](offline_extract_design_cn.md)

## 常见问题

### Q: 为什么不直接使用 cyber_monitor？

**A**: `cyber_monitor` 适合查看消息内容和发布频率，但存在以下限制：
- 高频数据刷新太快，无法看清具体数值
- 无法导出数据用于分析
- 无法进行多通道数据对齐
- 无法进行定量分析

### Q: 提取工具的性能如何？

**A**: 工具经过优化，可以处理高频数据：
- 使用流式处理，内存占用低
- CSV 格式输出速度快
- 支持处理大量录制文件

### Q: 支持哪些通道？

**A**: 目前支持常用的控制相关通道：
- `/apollo/canbus/chassis` - 底盘状态
- `/apollo/control` - 控制命令
- `/apollo/planning` - 规划轨迹
- `/apollo/localization/pose` - 定位信息
- `/apollo/perception/obstacles` - 感知障碍物
- 更多通道可通过修改代码添加

### Q: 如何添加新的通道支持？

**A**: 参考设计文档中的扩展点说明，主要步骤：
1. 导入消息类型
2. 添加到 `CHANNEL_MESSAGE_TYPE_MAP`
3. 重新运行即可

## 技术支持

如有问题或建议，请参考相应的设计文档或联系开发团队。
