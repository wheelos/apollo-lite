# **Robosense LiDAR Driver**

## 1 工程简介

**robosense** 为速腾聚创在阿波罗平台上的雷达驱动集成包。 目前支持以下型号的雷达。

- `RS-LiDAR-16`: `RS16`
- `RS-LiDAR-32`: `RS32`
- `RS-Bpearl`: `RSBP`
- `RS-Helios`: `RSHELIOS`
- `RS-Helios-16P`: `RSHELIOS_16P`
- `RS-Ruby-128`: `RS128`
- `RS-Ruby-80`: `RS80`
- `RS-Ruby-48`: `RS48`
- `RS-Ruby-Plus-128`: `RSP128`
- `RS-Ruby-Plus-80`: `RSP80`
- `RS-Ruby-Plus-48`: `RSP48`
- `RS-LiDAR-M1`: `RSM1`
- `RS-LiDAR-M2`: `RSM2`
- `RS-LiDAR-M3`: `RSM3`
- `RS-LiDAR-E1`: `RSE1`
- `RS-LiDAR-MX`: `RSMX`
- `RS-LiDAR-AIRY`: `RSAIRY`

## 2 运行

**所有驱动均需要在Apollo Docker环境下运行**

```sh
cyber_launch start /apollo/modules/drivers/lidar/rslidar/launch/rslidar.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/rslidar/dag/rslidar.dag
```

默认话题名：

- 原始点云 -- `/apollo/sensor/rslidar/up/PointCloud2`
- Scan -- `/apollo/sensor/rslidar/up/Scan`

## 3 参数介绍

[参数介绍](doc/parameter_intro.md)
