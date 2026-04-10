# Livox LiDAR Driver

English | [中文](README_cn.md)

## Overview

Livox LiDAR driver for Apollo-Lite, based on [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK). This driver uses the broadcast code based device discovery mechanism to connect Livox LiDARs.

## Supported Models

| Model | Device Type |
|-------|-------------|
| Livox Mid-40 | kDeviceTypeLidarMid40 |
| Livox Tele | kDeviceTypeLidarTele |
| Livox Horizon | kDeviceTypeLidarHorizon |
| Livox Mid-70 | kDeviceTypeLidarMid70 |
| Livox Avia | kDeviceTypeLidarAvia |

> **Note**: This driver uses [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK), NOT [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2). SDK2 does not have real Avia support despite having the enum value defined.

## Configuration

Edit `conf/livox.pb.txt`:

```protobuf
config_base {
  scan_channel: "/apollo/sensor/livox/front/Scan"
  point_cloud_channel: "/apollo/sensor/livox/front/PointCloud2"
  frame_id: "livox_front"
  source_type: ONLINE_LIDAR
}

# Device serial number (15-digit broadcast code)
# Empty means connect to all discovered devices
broadcast_code: ""
enable_sdk_console_log: false
integral_time: 0.1
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `broadcast_code` | string | "" | Device serial number. When empty, connects to all discovered devices. |
| `integral_time` | double | 0.1 | Point cloud integration time window in seconds. |
| `enable_sdk_console_log` | bool | false | Enable Livox SDK console log output. |

## Usage

### Build

```bash
./apollo.sh build drivers/lidar/livox
```

### Launch

```bash
# Using cyber_launch
cyber_launch start modules/drivers/lidar/livox/launch/livox.launch

# Using mainboard
mainboard -d modules/drivers/lidar/livox/dag/livox.dag
```

### Output Channels

- **Scan**: Raw lidar scan data (`livox::LivoxScan` message)
- **PointCloud2**: Aggregated point cloud (`PointCloud` message)

## Directory Structure

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
