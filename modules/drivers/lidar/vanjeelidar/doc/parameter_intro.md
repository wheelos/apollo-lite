# Vanjee LiDAR Driver Configuration Guide

This document provides a comprehensive introduction to all parameters in the Vanjee LiDAR driver configuration file. The configuration is based on the vanjee_driver v2.2.9 SDK.

## Table of Contents

- [Overview](#overview)
- [Configuration Structure](#configuration-structure)
- [Driver Parameters](#driver-parameters)
- [Input Parameters](#input-parameters)
- [Decoder Parameters](#decoder-parameters)
- [Transform Parameters](#transform-parameters)
- [MEMS Parameters](#mems-parameters)
- [Configuration Examples](#configuration-examples)

---

## Overview

The Vanjee LiDAR driver configuration follows a hierarchical structure defined in `vanjeelidar_config.proto`. The main configuration message is `Config`, which contains:

- **LidarConfigBase**: Base LiDAR configuration (frame_id, scan_channel, etc.)
- **VanjeeDriverParam**: Main driver parameters including input and decoder settings
- **VanjeeMemsParam**: MEMS-specific parameters for MEMS LiDAR models

---

## Configuration Structure

```
Config
├── config_base (LidarConfigBase)        # Base configuration (required)
├── driver_param (VanjeeDriverParam)     # Driver parameters (recommended)
│   ├── lidar_type                       # LiDAR model
│   ├── input_type                       # Input source type
│   ├── input_param (VanjeeInputParam)   # Input configuration
│   └── decoder_param (VanjeeDecoderParam) # Decoder configuration
└── mems_param (VanjeeMemsParam)         # MEMS parameters (optional)
```

---

## Driver Parameters

### VanjeeDriverParam

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar_type` | string | `"vanjee_720"` | LiDAR model identifier |
| `input_type` | int32 | `2` | Input source type |
| `input_param` | VanjeeInputParam | - | Input configuration parameters |
| `decoder_param` | VanjeeDecoderParam | - | Decoder configuration parameters |

#### lidar_type

Supported LiDAR models:

| Value | Description |
|-------|-------------|
| `"vanjee_716mini"` | Vanjee 716mini |
| `"vanjee_718h"` | Vanjee 718H |
| `"vanjee_719"` | Vanjee 719 |
| `"vanjee_719c"` | Vanjee 719C |
| `"vanjee_719e"` | Vanjee 719E |
| `"vanjee_720"` or `"vanjee_720_16"` | Vanjee 720 (16-line) |
| `"vanjee_720_32"` | Vanjee 720 (32-line) |
| `"vanjee_721"` | Vanjee 721 |
| `"vanjee_722"` | Vanjee 722 |
| `"vanjee_722f"` | Vanjee 722F |
| `"vanjee_722h"` | Vanjee 722H |
| `"vanjee_722z"` | Vanjee 722Z |
| `"vanjee_733"` | Vanjee 733 |
| `"vanjee_738"` | Vanjee 738 |
| `"vanjee_750"` | Vanjee 750 |
| `"vanjee_760"` | Vanjee 760 |

#### input_type

| Value | Constant | Description |
|-------|----------|-------------|
| `1` | `ONLINE_LIDAR` | Connect to real LiDAR device via network |
| `2` | `PCAP_FILE` | Read from PCAP file (offline) |
| `3` | `RAW_PACKET` | Receive raw packets from Cyber RT |

---

## Input Parameters

### VanjeeInputParam

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `connect_type` | uint32 | `1` | Network connection type |
| `host_msop_port` | uint32 | `3333` | Host port for MSOP data packets |
| `lidar_msop_port` | uint32 | `3001` | LiDAR port for MSOP data packets |
| `difop_port` | uint32 | `0` | Port for device info packets |
| `host_address` | string | `"0.0.0.0"` | Host IP address |
| `lidar_address` | string | `"0.0.0.0"` | LiDAR IP address |
| `group_address` | string | `"0.0.0.0"` | Multicast group address |
| `pcap_path` | string | `""` | Path to PCAP file |
| `pcap_repeat` | bool | `true` | Loop PCAP file playback |
| `pcap_rate` | float | `1.0` | PCAP playback rate multiplier |
| `use_vlan` | bool | `false` | Enable VLAN support |
| `user_layer_bytes` | uint32 | `0` | User layer bytes in packet header |
| `tail_layer_bytes` | uint32 | `0` | Tail layer bytes in packet header |
| `port_name` | string | `""` | Serial port name (for serial connection) |
| `baud_rate` | uint32 | `115200` | Serial baud rate |
| `network_interface` | string | `""` | Network interface name |

#### connect_type

| Value | Connection Type | Description |
|-------|----------------|-------------|
| `1` | UDP | UDP socket connection (default) |
| `2` | TCP | TCP socket connection |
| `3` | Serial Port | Serial port connection |

#### Network Configuration Example

For **ONLINE_LIDAR** mode with UDP:

```yaml
input_param {
  connect_type: 1
  host_msop_port: 3333
  lidar_msop_port: 3001
  host_address: "192.168.2.88"
  lidar_address: "192.168.2.86"
  group_address: "0.0.0.0"
}
```

#### PCAP Configuration Example

For **PCAP_FILE** mode:

```yaml
input_param {
  connect_type: 1
  host_msop_port: 3333
  pcap_path: "/path/to/lidar.pcap"
  pcap_repeat: true
  pcap_rate: 1.0
}
```

---

## Decoder Parameters

### VanjeeDecoderParam

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `config_from_file` | bool | `true` | Read angle calibration from file |
| `wait_for_difop` | bool | `true` | Wait for device info before decoding |
| `min_distance` | float | `0.0` | Minimum point distance (meters) |
| `max_distance` | float | `0.0` | Maximum point distance (meters) |
| `start_angle` | float | `0.0` | Start angle for FOV filter (degrees) |
| `end_angle` | float | `360.0` | End angle for FOV filter (degrees) |
| `use_lidar_clock` | bool | `false` | Use LiDAR clock for timestamps |
| `dense_points` | bool | `false` | Set invalid points to 0 instead of NaN |
| `ts_first_point` | bool | `false` | Use first point timestamp for frame |
| `use_offset_timestamp` | bool | `true` | Use offset time for point timestamps |
| `publish_mode` | uint32 | `2` | Point cloud publish mode |
| `rpm` | uint32 | `0` | LiDAR rotation speed (0 = auto) |
| `angle_path_ver` | string | `""` | Vertical angle calibration file path |
| `angle_path_hor` | string | `""` | Horizontal angle calibration file path |
| `imu_param_path` | string | `""` | IMU parameter file path |

#### Feature Enable Flags

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `point_cloud_enable` | bool | `false` | Enable point cloud output |
| `imu_enable` | int32 | `-1` | IMU enable mode |
| `imu_orientation_enable` | bool | `true` | Enable IMU orientation transform |
| `laser_scan_enable` | bool | `false` | Enable laser scan output |
| `device_ctrl_state_enable` | bool | `false` | Enable device state messages |
| `device_ctrl_cmd_enable` | bool | `false` | Enable device command messages |
| `send_packet_enable` | bool | `false` | Enable packet sending |
| `recv_packet_enable` | bool | `false` | Enable packet receiving |
| `send_lidar_param_enable` | bool | `false` | Enable LiDAR parameter sending |
| `recv_lidar_param_cmd_enable` | bool | `false` | Enable LiDAR parameter command receiving |
| `query_via_external_interface_enable` | bool | `false` | Query parameters via external interface |
| `tail_filter_enable` | bool | `false` | Enable tail filter for point cloud |
| `hide_points_range` | string | `""` | Points hiding range specification |

#### Advanced Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `transform_param` | VanjeeTransformParam | - | Coordinate transformation parameters |

---

### Detailed Decoder Parameter Descriptions

#### Distance Filtering

- **min_distance**: Minimum valid point distance in meters. Points closer than this value are filtered out.
- **max_distance**: Maximum valid point distance in meters. Points farther than this value are filtered out. A value of 0 means no maximum limit.

#### Angle Filtering (FOV)

- **start_angle**: Start angle for the field of view filter in degrees (0-360).
- **end_angle**: End angle for the field of view filter in degrees (0-360).

Only points within the angle range [start_angle, end_angle] will be published.

#### Timestamp Modes

- **use_lidar_clock**:
  - `true`: Use LiDAR internal clock for message timestamps
  - `false`: Use host system clock for message timestamps (default)

- **ts_first_point**:
  - `true`: Point cloud timestamp uses the first point's time
  - `false`: Point cloud timestamp uses the last point's time (default)

- **use_offset_timestamp**:
  - `true`: Each point's timestamp is an offset relative to the frame timestamp
  - `false`: Each point uses absolute UTC timestamp

#### Point Cloud Mode

- **dense_points**:
  - `true`: Invalid points are set to (0, 0, 0)
  - `false`: Invalid points are set to NaN (default)

#### publish_mode

Controls which echo (return) data to publish:

| Value | Description |
|-------|-------------|
| `0` | Publish first return only |
| `1` | Publish second return only |
| `2` | Publish both returns (default) |

#### imu_enable

Controls IMU data processing:

| Value | Description |
|-------|-------------|
| `-1` | Disable IMU (default) |
| `0` | Enable IMU with default calibration parameters |
| `1` | Enable IMU with configured parameters |

#### hide_points_range

Specifies point ranges to hide/exclude from the output. Format:

```
<line_range>,<angle_range>,<distance_range>[;<group2>;<group3>...]
```

Where:
- `<line_range>`: `L1-L2` (line numbers, e.g., `1-16`)
- `<angle_range>`: `A1-A2/A3-A4/...` (angles in degrees, e.g., `0-0.5/100-105`)
- `<distance_range>`: `D1-D2/D3-D4/...` (distances in meters, e.g., `0-1.5/10-20`)

Examples:
- `1-3,0-0.5/100-105,0-1.5/10-20`: Hides lines 1-3; angles 0-0.5° and 100-105°; distances 0-1.5m and 10-20m
- Multiple groups can be separated by semicolons (`;`)
- Non-contiguous ranges within a group are separated by slashes (`/`)

#### rpm

LiDAR rotation speed in RPM. A value of 0 means auto-detect from the device.

---

## Transform Parameters

### VanjeeTransformParam

Coordinate transformation parameters for the LiDAR sensor.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `x` | float | `0.0` | Translation along X axis (meters) |
| `y` | float | `0.0` | Translation along Y axis (meters) |
| `z` | float | `0.0` | Translation along Z axis (meters) |
| `roll` | float | `0.0` | Rotation around X axis (degrees) |
| `pitch` | float | `0.0` | Rotation around Y axis (degrees) |
| `yaw` | float | `0.0` | Rotation around Z axis (degrees) |
| `x_imu` | float | `0.0` | IMU translation along X axis |
| `y_imu` | float | `0.0` | IMU translation along Y axis |
| `z_imu` | float | `0.0` | IMU translation along Z axis |
| `roll_imu` | float | `0.0` | IMU rotation around X axis |
| `pitch_imu` | float | `0.0` | IMU rotation around Y axis |
| `yaw_imu` | float | `0.0` | IMU rotation around Z axis |

---

## MEMS Parameters

### VanjeeMemsParam

Parameters for MEMS-type LiDAR models (e.g., Vanjee 722, 733).

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rotate_mirror_pitch` | float | `0.0` | Rotate mirror pitch angle |
| `rotate_mirror_offset` | float[] | - | Rotate mirror offset values |
| `view_center_yaws` | float[] | - | View center yaw angles |
| `rwadata_yaw_resolution` | float | `0.0` | Raw data yaw resolution (degrees) |
| `rwadata_pitch_resolution` | float | `0.0` | Raw data pitch resolution (degrees) |
| `start_pitch` | float | `0.0` | Start pitch angle (degrees) |
| `end_pitch` | float | `0.0` | End pitch angle (degrees) |
| `start_yaw` | float | `0.0` | Start yaw angle (degrees) |
| `end_yaw` | float | `0.0` | End yaw angle (degrees) |
| `beta` | float | `0.0` | Beta correction parameter |
| `gama_z` | float | `0.0` | Gamma Z correction parameter |
| `gama_x` | float | `0.0` | Gamma X correction parameter |
| `reversal_horizontal` | bool | `false` | Horizontal reversal flag |
| `reversal_vertical` | bool | `false` | Vertical reversal flag |
| `scan_echo_type` | uint32 | `1` | Scan echo type |

---

## Configuration Examples

### Example 1: Online LiDAR (Vanjee 720-16)

```protobuf
config_base {
  frame_id: "velodyne"
  scan_channel: "/apollo/sensor/vanjee/scan"
  point_cloud_channel: "/apollo/sensor/vanjee/PointCloud2"
}

driver_param {
  lidar_type: "vanjee_720_16"
  input_type: 1  # ONLINE_LIDAR

  input_param {
    connect_type: 1  # UDP
    host_msop_port: 3333
    lidar_msop_port: 3001
    host_address: "192.168.2.88"
    lidar_address: "192.168.2.86"
  }

  decoder_param {
    min_distance: 0.2
    max_distance: 120.0
    start_angle: 0.0
    end_angle: 360.0
    use_lidar_clock: false
    dense_points: false
    wait_for_difop: true
    config_from_file: false
    publish_mode: 2  # Both returns
    point_cloud_enable: true
    imu_enable: -1  # Disabled
  }
}
```

### Example 2: PCAP File Playback

```protobuf
config_base {
  frame_id: "velodyne"
  scan_channel: "/apollo/sensor/vanjee/scan"
  point_cloud_channel: "/apollo/sensor/vanjee/PointCloud2"
}

driver_param {
  lidar_type: "vanjee_720_16"
  input_type: 2  # PCAP_FILE

  input_param {
    connect_type: 1
    host_msop_port: 3333
    pcap_path: "/apollo/modules/drivers/lidar/vanjeelidar/data/vanjeelidar-720.pcap"
    pcap_repeat: true
    pcap_rate: 1.0
  }

  decoder_param {
    min_distance: 0.2
    max_distance: 120.0
    start_angle: 0.0
    end_angle: 360.0
    use_lidar_clock: false
    dense_points: false
    wait_for_difop: false
    config_from_file: true
    angle_path_ver: "/path/to/Vejee_720_16_VA.csv"
    angle_path_hor: "/path/to/Vejee_720_HA.csv"
    publish_mode: 2
    point_cloud_enable: true
  }
}
```

### Example 3: With Transform and IMU

```protobuf
config_base {
  frame_id: "velodyne"
  scan_channel: "/apollo/sensor/vanjee/scan"
  point_cloud_channel: "/apollo/sensor/vanjee/PointCloud2"
}

driver_param {
  lidar_type: "vanjee_720_16"
  input_type: 1

  input_param {
    connect_type: 1
    host_msop_port: 3333
    host_address: "192.168.2.88"
    lidar_address: "192.168.2.86"
  }

  decoder_param {
    min_distance: 0.2
    max_distance: 120.0
    start_angle: 0.0
    end_angle: 360.0
    publish_mode: 2
    point_cloud_enable: true
    imu_enable: 1  # Enable with configured params
    imu_orientation_enable: true

    transform_param {
      x: 1.2
      y: 0.0
      z: 1.8
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

      x_imu: 1.15
      y_imu: 0.0
      z_imu: 1.75
      roll_imu: 0.0
      pitch_imu: 0.0
      yaw_imu: 0.0
    }
  }
}
```

---

## Parameter Mapping Between SDK and Proto

The proto definitions in `vanjeelidar_config.proto` are designed to match the C++ structures in the vanjee_driver SDK:

| Proto Message | SDK Structure |
|---------------|---------------|
| `VanjeeDriverParam` | `WJDriverParam` |
| `VanjeeInputParam` | `WJInputParam` |
| `VanjeeDecoderParam` | `WJDecoderParam` |
| `VanjeeTransformParam` | `WJTransfromParam` |
| `VanjeeMemsParam` | `WJMemsParam` |

---

## Notes

1. **Deprecated Parameters**: The top-level parameters in `Config` (lines 1-27 in the proto file) are deprecated. Use `driver_param` and `mems_param` instead.

2. **Wait for DIFOP**: When `wait_for_difop` is `true`, the driver will wait for device information packets from the LiDAR before starting point cloud decoding. This is recommended for online LiDAR mode to ensure correct calibration.

3. **Config from File**: When `config_from_file` is `true`, angle calibration data is read from the specified files instead of being requested from the LiDAR. This is useful for PCAP playback or when the LiDAR doesn't support difop packets.

4. **PCAP Rate**: The `pcap_rate` parameter controls playback speed. A value of `1.0` plays at normal speed, `2.0` plays at 2x speed, and `0.5` plays at half speed.

5. **Network Interface**: For multi-NIC systems, specify the network interface name (e.g., `eth0`) to ensure packets are received from the correct interface.
