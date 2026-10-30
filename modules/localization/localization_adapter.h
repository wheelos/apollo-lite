// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <optional>
#include <string>

#include "modules/localization/proto/localization_config.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"
#include "wheelos_msgs/sensor_msgs/imu.pb.h"

#include "modules/openloc_localization/native_global_worker.h"
#include "modules/openloc_localization/native_lio_config.h"

namespace apollo::localization::openloc {

namespace host = ::apollo::localization::openloc_diagnostic;
namespace native = ::openloc::localization::unified;

void ValidateConfiguration(const LocalizationConfig& config);
native::ImuSample DecodeImu(const drivers::gnss::Imu& input,
                            const LocalizationConfig& config, double receive,
                            uint64_t sequence);
native::LidarScan DecodeCloud(const drivers::PointCloud& input,
                              const LocalizationConfig& config,
                              uint32_t maximum_points, double receive,
                              uint64_t sequence);

struct Publication {
  LocalizationStatus status;
  std::optional<LocalizationEstimate> pose;
  std::optional<transform::TransformStamped> transform;
};

Publication MakePublication(const LocalizationConfig& config,
                            const host::NativeGlobalDiagnostic& global,
                            const native::LocalState* local, double now,
                            uint32_t sequence);

}  // namespace apollo::localization::openloc
