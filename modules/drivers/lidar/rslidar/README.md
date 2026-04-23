# **Robosense LiDAR Driver**

## 1 Introduction

**robosense** is the lidar driver kit under Apollo platform. Now support the following models:

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

## 2 Run

**All the drivers need to be excuted in Apollo docker environment.**

```sh
cyber_launch start /apollo/modules/drivers/lidar/rslidar/launch/rslidar.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/rslidar/dag/rslidar.dag
```

default channel：

- PointCloud -- `/apollo/sensor/rslidar/up/PointCloud2`
- Scan -- `/apollo/sensor/rslidar/up/Scan`

## 3 Parameters Intro

[Intro to parameters](doc/parameter_intro.md)
