// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <string>

#include "modules/localization/proto/localization_config.pb.h"

#include "modules/common/vehicle_calibration/registry.h"
#include "modules/transform/transform_query.h"

namespace apollo::localization::openloc {

LocalizationConfig LoadDeploymentConfiguration(const std::string& path);
LocalizationConfig ResolveConfiguration(
    const LocalizationConfig& deployment,
    const common::vehicle_calibration::Registry& calibration,
    const transform::TransformQuery& query);

}  // namespace apollo::localization::openloc
