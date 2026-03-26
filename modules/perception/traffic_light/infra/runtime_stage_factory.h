/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <memory>

#include "modules/perception/traffic_light/application/ports/traffic_light_detector_port.h"
#include "modules/perception/traffic_light/application/ports/traffic_light_tracker_port.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace infra {

std::unique_ptr<application::TrafficLightDetectorPort>
CreateRuntimeTrafficLightDetector(const pipeline::StageConfig& stage_config);

std::unique_ptr<application::TrafficLightTrackerPort>
CreateRuntimeTrafficLightTracker(const pipeline::StageConfig& stage_config);

}  // namespace infra
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
