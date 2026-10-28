# **Vanjee LiDAR Driver**

## 1 Introduction

 **vanjee**  is the lidar driver kit under Apollo platform.

### Supported LiDAR Models

- `vanjee_716mini`
- `vanjee_718h`
- `vanjee_719`
- `vanjee_719c`
- `vanjee_719e`
- `vanjee_720` / `vanjee_720_16`
- `vanjee_720_32`
- `vanjee_721`
- `vanjee_722`
- `vanjee_722f`
- `vanjee_722h`
- `vanjee_722z`
- `vanjee_733`
- `vanjee_750`
- `vanjee_760`

## 2 Run

**All the drivers need to be excuted in Apollo docker environment.**

```sh
cyber_launch start /apollo/modules/drivers/lidar/vanjeelidar/launch/vanjeelidar.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/vanjeelidar/dag/vanjeelidar.dag
```

default topic name：

- raw point cloud -- /apollo/sensor/vanjeelidar/up/PointCloud2"
- Scan -- /apollo/sensor/vanjeelidar/up/Scan

## 3 Parameters Intro

[Intro to parameters](doc/parameter_intro.md)

## 4 IMU output and point time

The default 720-16 example publishes `apollo.drivers.vanjee.VanjeeImu` on
`/apollo/sensor/vanjeelidar/up/Imu`. Enable the SDK with `imu_enable: 0`
(default SDK calibration) or `1` (model-specific calibration parameters), and
set both `imu_channel` and `imu_frame_id`. Existing configurations with
`imu_enable: -1` remain disabled. The SDK may withhold measurements during its
stationary gyro calibration; a configured publisher does not prove that the
device supports IMU or that calibration has completed.

The message preserves SDK measurement seconds, sequence, angular velocity
in rad/s and acceleration in m/s^2 including gravity response. The vectors
are already SDK-calibrated/rotated; `imu_frame_id` names that actual output
frame, not an assumed `base_link`. Supply its calibrated LiDAR extrinsic to
the LIO consumer. Orientation and covariance are not published because the
SDK does not consistently initialize them. This is intentionally not
`apollo.drivers.gnss.Imu`, whose measurement time is GPS-epoch seconds.

`use_lidar_clock` in the output records the SDK clock selection, not a
synchronization guarantee. The examples retain `use_lidar_clock: false`
(host-time approximation); enable device time only after checking the device
clock and the IMU/LiDAR common time basis. Publication time is separately
recorded in `header.timestamp_sec`.

Point timestamps are absolute nanoseconds in that selected SDK clock domain.
With `use_offset_timestamp: true`, signed offsets (including negative
frame-end-relative offsets) are added to the SDK frame anchor once. With
`false`, SDK point timestamps are already absolute and are not added again.
`measurement_time` and `header.lidar_timestamp` use the maximum point time,
independent of point ordering; both frame-id fields are set. Invalid/empty
clouds and invalid IMU samples are rejected with errors, not stamped with
arrival time. Ring preservation and the FAST-LIO2 consumer adapter remain
separate work; this change does not add ring to the shared PointCloud schema.

Software regression targets:

```sh
bazel test --config=dev \
  //modules/drivers/lidar/vanjeelidar:measurement_conversion_test \
  //modules/drivers/lidar/vanjeelidar:vanjeelidar_component_test
```

Run them as the mapped non-root user in the managed development container.
These tests exercise conversion and SDK-callback-to-Cyber publication, not
physical-device timing, SDK packet decoding, or complete FAST-LIO2 operation.
