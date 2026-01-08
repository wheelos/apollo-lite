# **Seyond Lidar Driver**

## 1 Introduction

 **seyond** Seyond-Apollo Lidar Driver.

### Supported lidar

- `Falcon-K1`
- `Falcon-K2`
- `Falcon-k24`
- `Robin-W`
- `Robin-ELITE(Robin-E1X)`
- `Robin-E2`
- `HB-D1`

## 2 Run

**All the drivers need to be excuted in Apollo docker environment.**

```sh
cyber_launch start /apollo/modules/drivers/lidar/seyond/launch/seyond.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/seyond/dag/seyond.dag
```

Default Topic：

- PointCloud -- /apollo/sensor/seyond/up/PointCloud2"
- Scan -- /apollo/sensor/seyond/up/Scan

## 3 Parameters Intro
| Parameter          | Default Value | description   |
| :--------:         | :---------:   | :---------:   |
| device_ip          | 172.168.1.10 | lidar ip   |
| port               | 8010         | tcp port   |
| udp_port           | 8010         | udp port, if < 0, use tcp for transmission   |
| reflectance_mode   | true         | false:intensity mode true:reflectance_mode mode   |
| multiple_return    | 1            | lidar detection echo mode   |
| coordinate_mode    | 3            | convert the xyz direction of a point cloud, 0: lidar-default, 3:WGS-84   |
| max_range          | 2000.0       | point cloud display maximum distance (unit:m)   |
| min_range          | 0.4          | point cloud display minimum distance (unit:m)   |
| log_level          | "info"       | limit log from lidar, can choose from (info warn error)    |
