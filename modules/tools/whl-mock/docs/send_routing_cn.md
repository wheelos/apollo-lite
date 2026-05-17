# send_routing.py 使用指南

## 概述

`send_routing.py` 是一个用于通过 Cyber RT 发送路由请求（RoutingRequest）的命令行工具。它可以帮助你在仿真和实车测试场景中，以编程方式精确控制路由请求的发送，避免使用 Dreamview 手动操作带来的误差。

## 应用场景

### 1. 仿真场景

在仿真测试中，可以固定起点和终点，方便进行多组测试对比结果：

- **精确可重复**：每次发送的航点完全相同，避免了在 Dreamview 上使用鼠标点击操作带来的位置误差
- **批量测试**：支持循环发送和定时发送，便于进行多组控制算法测试
- **自动化集成**：可以作为自动化测试脚本的一部分，无需人工干预

### 2. 实车场景

在实际车辆测试中，可以每次获取当前车辆实际位置作为起点：

- **动态起点**：使用 `--add-pose` 选项，自动从定位通道获取当前车辆位置作为第一个航点
- **固定终点**：在配置文件中预设固定的目标点或路径
- **测试对比**：适合进行相同路线的多轮测试对比，评估算法一致性

## 功能特性

- 从文件加载 RoutingRequest 配置
- 自动添加当前车辆位姿作为起点
- 支持单次发送、定时循环发送、交互式发送
- 两种位姿刷新模式：
  - `once`: 使用启动时获取的位姿（适用于仿真）
  - `fresh`: 每次发送时获取最新位姿（适用于实车）

## 安装与依赖

工具位于 Apollo 项目中，无需额外安装：

```bash
cd /apollo/modules/tools/whl-mock
./send_routing.py --help
```

## RoutingRequest 文件格式

配置文件使用 Protocol Buffer Text Format 格式：

```protobuf
waypoint {
  pose {
    x: 272114.24
    y: 4020864.14
    z: 0.0
  }
  heading: 1.57
}
waypoint {
  pose {
    x: 272091.94
    y: 4020872.85
    z: 0.0
  }
  heading: 3.1040
}

parking_info {
  parking_space_id: "test_parking"
  parking_point {
    x: 272091.94
    y: 4020872.85
    z: 0.0
  }
  parking_space_type: VERTICAL_PLOT
  heading: 3.1040
}
```

### 字段说明

| 字段 | 说明 | 单位 |
|------|------|------|
| `waypoint[].pose.x/y/z` | 航点位置坐标 | 米 |
| `waypoint[].heading` | 航点航向角 | 弧度 |
| `parking_info.parking_space_id` | 停车位 ID | 字符串 |
| `parking_info.parking_point` | 停车位中心点 | - |
| `parking_info.parking_space_type` | 停车位类型 | 枚举 |
| `parking_info.heading` | 停车位航向 | 弧度 |

## 使用方法

### 基本用法

#### 单次发送（固定路径）

适用于仿真场景，发送预设的固定路径：

```bash
./send_routing.py -i RoutingRequest_template.txt
```

#### 添加当前位姿作为起点

适用于实车场景，自动获取当前车辆位置作为起点：

```bash
./send_routing.py --add-pose -i RoutingRequest_template.txt
```

### 高级用法

#### 定时循环发送

每隔指定时间发送一次路由请求：

```bash
# 每 1 秒发送一次
./send_routing.py --loop --interval 1.0

# 每 5 秒发送一次，使用缓存的起点位姿
./send_routing.py --add-pose --loop --interval 5.0 --pose-mode once
```

#### 交互式发送

按 `c` 键发送，按 `q` 键退出：

```bash
# 基本交互模式
./send_routing.py --interactive

# 交互模式 + 每次获取最新位姿
./send_routing.py --add-pose --interactive --pose-mode fresh
```

#### 使用自定义通道

```bash
./send_routing.py -c /apollo/custom_routing_request
```

## 命令行选项

| 选项 | 说明 | 默认值 |
|------|------|--------|
| `-i, --input` | 输入文件路径 | `RoutingRequest.txt` |
| `-c, --channel` | 目标 Cyber RT 通道 | `/apollo/routing_request` |
| `--add-pose` | 添加当前位姿作为第一个航点 | 关闭 |
| `--localization-channel` | 定位数据通道 | `/apollo/localization/pose` |
| `--pose-timeout` | 等待位姿超时时间（秒） | 10.0 |
| `--loop` | 循环模式 | 关闭 |
| `--interval` | 发送间隔时间（秒） | 1.0 |
| `--interactive` | 交互模式（按键发送） | 关闭 |
| `--pose-mode` | 位姿刷新模式：once/fresh | `fresh` |
| `--wait` | 发送前等待时间，用于服务发现（秒） | 1.0 |
| `--count` | 发送次数（0=无限，仅 loop 模式） | 0 |
| `--log-level` | 日志级别 | `INFO` |

## 位姿模式详解

### `--pose-mode once`

使用启动时获取的位姿，后续所有发送都使用该缓存的位姿。

**适用场景**：仿真测试，需要固定起点

```bash
./send_routing.py --add-pose --pose-mode once --loop
```

**输出示例**：
```
Pose: (272114.24, 4020864.14)
Using cached pose for all iterations
Sending every 1.0s (Ctrl+C to stop)
[INFO] Sent RoutingRequest (seq=1): 3 waypoints
[INFO] Sent RoutingRequest (seq=2): 3 waypoints
...
```

### `--pose-mode fresh`

每次发送时获取最新的车辆位姿。

**适用场景**：实车测试，需要动态起点

```bash
./send_routing.py --add-pose --pose-mode fresh --loop
```

**输出示例**：
```
Pose: (272114.24, 4020864.14)
Getting fresh pose each iteration
Sending every 1.0s (Ctrl+C to stop)
[INFO] Sent RoutingRequest (seq=1): 3 waypoints
[INFO] Sent RoutingRequest (seq=2): 3 waypoints
...
```

## 使用示例

### 示例 1：仿真场景 - 固定路径多轮测试

```bash
# 准备配置文件
cat > test_route.txt << EOF
waypoint {
  pose { x: 272114.24 y: 4020864.14 z: 0.0 }
  heading: 1.57
}
waypoint {
  pose { x: 272091.94 y: 4020872.85 z: 0.0 }
  heading: 3.1040
}
EOF

# 每 2 秒发送一次，进行多轮测试
./send_routing.py -i test_route.txt --loop --interval 2.0
```

### 示例 2：实车场景 - 动态起点固定终点

```bash
# 配置文件只包含终点
cat > destination.txt << EOF
waypoint {
  pose { x: 272091.94 y: 4020872.85 z: 0.0 }
  heading: 3.1040
}
EOF

# 自动添加当前车辆位置作为起点，每 5 秒发送一次
./send_routing.py --add-pose -i destination.txt --loop --interval 5.0
```

### 示例 3：交互式测试

```bash
# 交互模式，按 c 发送，按 q 退出
./send_routing.py --interactive --add-pose

# 输出：
# === Interactive Mode ===
# c - send, q - quit
# > c
# [INFO] Sent RoutingRequest (seq=1): 2 waypoints
# > c
# [INFO] Sent RoutingRequest (seq=2): 2 waypoints
# > q
```

### 示例 4：批量自动化测试

```bash
# 结合 bash 脚本进行自动化测试
for i in {1..10}; do
  echo "Running test $i..."
  ./send_routing.py -i test_route.txt
  sleep 30  # 等待测试完成
done
```

## 常见问题

### Q: 为什么我的请求没有生效？

A: 请检查以下事项：
1. 确保 Cyber RT 正在运行：`cyber_launch status`
2. 确保 Routing 模块正在运行
3. 检查通道名称是否正确
4. 使用 `--log-level DEBUG` 查看详细日志

### Q: 如何获取当前车辆位置？

A: 使用以下方法之一：
```bash
# 方法 1: 使用 cyber_monitor
cyber_monitor /apollo/localization/pose

# 方法 2: 使用本工具
./send_routing.py --add-pose --log-level DEBUG
```

### Q: `--pose-mode once` 和 `fresh` 有什么区别？

A:
- `once`: 启动时获取一次位姿，后续都使用该位姿
- `fresh`: 每次发送前都获取最新的位姿

### Q: 如何调试 RoutingRequest 配置？

A:
1. 使用 Dreamview 发送一次请求
2. 在 Dreamview 的 PnC Monitor 中查看 Routing Response
3. 复制有效的 waypoint 配置到你的文件

## 技术细节

### 消息格式

工具发送的是 Apollo `RoutingRequest` 消息，定义位于：
`modules/common_msgs/routing_msgs/routing.proto`

### 通道信息

- **默认通道**: `/apollo/routing_request`
- **定位通道**: `/apollo/localization/pose`

### 序列号

每次发送会自动递增序列号（sequence_num），可用于追踪请求顺序。

## 相关工具

- **Dreamview**: 可视化路由请求和响应
- **cyber_monitor**: 监控 Cyber RT 通道消息
- **cyber_recorder**: 录制和回放消息
