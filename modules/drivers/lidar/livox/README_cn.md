# Livox 激光雷达驱动

[English](README.md) | 中文

## 概述

Apollo-Lite 的 Livox 激光雷达驱动，基于 [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)。本驱动使用广播码（broadcast code）设备发现机制连接 Livox 雷达。

## 支持型号

| 型号 | 设备类型 |
|------|----------|
| Livox Mid-40 | kDeviceTypeLidarMid40 |
| Livox Tele | kDeviceTypeLidarTele |
| Livox Horizon | kDeviceTypeLidarHorizon |
| Livox Mid-70 | kDeviceTypeLidarMid70 |
| Livox Avia | kDeviceTypeLidarAvia |

> **注意**：本驱动使用的是 [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)，而非 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)。SDK2 虽然定义了 Avia 的枚举值，但并没有真正的 Avia 代码实现。

## 配置

编辑 `conf/livox.pb.txt`：

```protobuf
config_base {
  scan_channel: "/apollo/sensor/livox/front/Scan"
  point_cloud_channel: "/apollo/sensor/livox/front/PointCloud2"
  frame_id: "livox_front"
  source_type: ONLINE_LIDAR
}

# 设备序列号（15位广播码）
# 为空时连接所有发现的设备
broadcast_code: ""
enable_sdk_console_log: false
integral_time: 0.1
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `broadcast_code` | string | "" | 设备序列号。为空时连接所有发现的设备。 |
| `integral_time` | double | 0.1 | 点云积分时间窗口，单位秒。 |
| `enable_sdk_console_log` | bool | false | 是否启用 Livox SDK 控制台日志输出。 |

## 使用方法

### 编译

```bash
./apollo.sh build drivers/lidar/livox
```

### 启动

```bash
# 使用 cyber_launch
cyber_launch start modules/drivers/lidar/livox/launch/livox.launch

# 使用 mainboard
mainboard -d modules/drivers/lidar/livox/dag/livox.dag
```

### 输出通道

- **Scan**：原始雷达扫描数据（`livox::LivoxScan` 消息）
- **PointCloud2**：聚合后的点云数据（`PointCloud` 消息）

## 目录结构

```
livox/
├── component/
│   └── livox_component.h/cpp
├── conf/
│   └── livox.pb.txt
├── dag/
│   └── livox.dag
├── launch/
│   └── livox.launch
└── proto/
    └── livox.proto
```
