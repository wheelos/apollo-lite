# Apollo Configuration Inventory

This document maps every configuration loading point discovered in the
codebase to the file it loads, the loader function used, and the protobuf
message type it populates.  It forms the baseline for the layered override
migration tracked in the [configuration override design doc](configuration_override.md).

---

## Legend

| Column | Meaning |
|--------|---------|
| **Module** | Apollo module name |
| **Source file** | C++ file that loads the config |
| **Config file (default path)** | Path used via gflag or hardcoded string |
| **Loader** | C++ function called |
| **Proto type** | Protobuf message type populated |

---

## Core / Common

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| common | `modules/common/configs/vehicle_config_helper.cc` | `FLAGS_vehicle_config_path` → `/apollo/modules/common/data/vehicle_param.pb.txt` | `GetProtoFromFile` | `apollo.common.VehicleConfig` |
| common | `modules/common/vehicle_model/vehicle_model.cc` | `FLAGS_vehicle_model_config_filename` → `/apollo/modules/common/vehicle_model/conf/vehicle_model_config.pb.txt` | `GetProtoFromFile` | `apollo.common.VehicleModelConfig` |

---

## Control

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| control | `modules/control/control_component.cc` | `FLAGS_control_conf_file` → `/apollo/modules/control/conf/control_conf.pb.txt` | `GetProtoFromFileWithOverride` ✅ | `apollo.control.ControlConf` |
| control | `modules/control/submodules/mpc_controller_submodule.cc` | `FLAGS_mpc_controller_conf_file` → `/apollo/modules/control/conf/mpc_controller_conf.pb.txt` | `GetProtoFromFile` | `apollo.control.MPCControllerConf` |
| control | `modules/control/submodules/lat_lon_controller_submodule.cc` | `FLAGS_lateral_controller_conf_file` | `GetProtoFromFile` | `apollo.control.LatControllerConf` |
| control | `modules/control/submodules/preprocessor_submodule.cc` | `FLAGS_control_common_conf_file` | `GetProtoFromFile` | `apollo.control.ControlCommonConf` |

---

## Planning

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| planning | `modules/planning/planning_component.cc` | Cyber component framework (`GetProtoConfig`) | `ComponentBase::GetProtoConfig` | `apollo.planning.PlanningConfig` |
| planning | `modules/planning/on_lane_planning.cc` | Multiple scenario conf files via `GetProtoFromFile` | `GetProtoFromFile` | Various `ScenarioConfig` subtypes |
| planning | `modules/planning/reference_line/reference_line_provider.cc` | Smoother config | `GetProtoFromFile` | `apollo.planning.ReferenceLineSmootherConfig` |

---

## Routing

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| routing | `modules/routing/core/navigator.cc` | `FLAGS_routing_conf_file` → `/apollo/modules/routing/conf/routing_conf.pb.txt` | `GetProtoFromFile` | `apollo.routing.RoutingConfig` |

---

## Localization

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| localization | `modules/localization/rtk/rtk_localization_component.cc` | RTK localization conf | `GetProtoFromFile` | `apollo.localization.rtk.RTKLocalizationConfig` |

---

## Prediction

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| prediction | `modules/prediction/common/message_process.cc` | `FLAGS_prediction_conf_file` | `GetProtoFromFile` | `apollo.prediction.PredictionConf` |

---

## Canbus

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| canbus | `modules/canbus/canbus_component.cc` | `FLAGS_canbus_conf_file` | `GetProtoFromFile` | `apollo.canbus.CanbusConf` |

---

## Drivers

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| drivers/gnss | `modules/drivers/gnss/gnss_component.cc` | GNSS sensor conf | `GetProtoFromFile` | `apollo.drivers.gnss.config.Config` |
| drivers/camera | `modules/drivers/camera/camera_component.cc` | Camera sensor conf | `GetProtoFromFile` | `apollo.drivers.CameraConf` |
| drivers/radar | `modules/drivers/radar/racobit_radar/racobit_radar_canbus_component.cc` | Radar sensor conf | `GetProtoFromFile` | `apollo.drivers.RadarConf` |

---

## Perception

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| perception/lidar | `modules/perception/onboard/component/lidar_detection_component.cc` | Lidar detection pipeline conf | `GetProtoFromFile` | `apollo.perception.onboard.LidarDetectionComponentConfig` |
| perception/camera | `modules/perception/onboard/component/fusion_camera_detection_component.cc` | Camera detection pipeline conf | `GetProtoFromFile` | `apollo.perception.onboard.FusionCameraDetectionConfig` |
| perception/fusion | `modules/perception/fusion/lib/fusion_system/probabilistic_fusion/probabilistic_fusion.cc` | Fusion conf | `GetProtoFromFile` | `apollo.perception.fusion.ProbabilisticFusionConfig` |

---

## Dreamview

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| dreamview | `modules/dreamview/backend/hmi/hmi_worker.cc` | HMI conf | `GetProtoFromFile` | `apollo.dreamview.HMIConfig` |
| dreamview | `modules/dreamview/backend/hmi/vehicle_manager.cc` | Vehicle manager conf | `GetProtoFromFile` | `apollo.dreamview.VehicleData` |

---

## Cyber Runtime

| Module | Source file | Config file (default path) | Loader | Proto type |
|--------|-------------|---------------------------|--------|------------|
| cyber | `cyber/common/global_data.cc` | `cyber/conf/cyber.pb.conf` | `GetProtoFromFile` | `apollo.cyber.proto.CyberConfig` |

---

## Migration Status

| Status | Meaning |
|--------|---------|
| ✅ | Already using `GetProtoFromFileWithOverride` |
| ⬜ | Still using `GetProtoFromFile` (pending migration) |

Only `modules/control/control_component.cc` has been migrated so far as the
reference implementation.  All other loaders listed as ⬜ are candidates for
migration in follow-up work.
