// Copyright 2026 WheelOS. All Rights Reserved.
#include <iostream>

#include "modules/common/vehicle_calibration/lidar_height.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/transform/calibration/static_transform_loader.h"

int main() {
  namespace vc = apollo::common::vehicle_calibration;
  vc::Registry registry;
  if (!registry.LoadFromEnvironment()) {
    return 1;
  }
  apollo::transform::TransformStampeds transforms;
  if (!apollo::transform::LoadStaticTransforms(registry, &transforms)) {
    return 1;
  }
  if (!registry.resolved_lidar_height_file().empty()) {
    vc::LidarHeight height;
    if (!vc::LoadLidarHeight(registry, "", &height)) {
      return 1;
    }
  }
  for (const auto& sensor : registry.sensors()) {
    if (!sensor.resolved_lidar_height_file.empty()) {
      vc::LidarHeight height;
      if (!vc::LoadLidarHeight(registry, sensor.calibration.sensor_id(),
                               &height)) {
        return 1;
      }
    }
  }
  apollo::perception::common::SensorManager sensors(registry);
  if (!sensors.Init()) {
    return 1;
  }
  std::cout << "vehicle=" << registry.profile().vehicle_id()
            << " version=" << registry.profile().calibration_version()
            << " manifest_sha256=" << registry.manifest_sha256()
            << " sensors=" << registry.sensors().size()
            << " static_edges=" << transforms.transforms_size() << '\n';
  return 0;
}
