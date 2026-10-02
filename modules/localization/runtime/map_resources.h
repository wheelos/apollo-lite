// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <memory>
#include <vector>

#include "modules/localization/map/map_matcher.h"
#include "modules/localization/runtime/messages.h"

namespace apollo {
namespace localization {
namespace unified {

Result LoadGlobalMap(const GlobalEstimatorConfig& config,
                     std::shared_ptr<MapMatcher>* matcher,
                     std::vector<Eigen::Isometry3d>* seeds);
Result ValidateMapSwitch(const MapSwitchRequest& request,
                         const GlobalEstimatorConfig& active,
                         const Epoch& epoch, uint64_t last_request,
                         double now);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
