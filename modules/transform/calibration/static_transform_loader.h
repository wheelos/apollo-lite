// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include "wheelos_msgs/transform_msgs/transform.pb.h"

#include "modules/common/vehicle_calibration/registry.h"

namespace apollo {
namespace transform {

bool LoadStaticTransforms(const common::vehicle_calibration::Registry& registry,
                          TransformStampeds* transforms);

}  // namespace transform
}  // namespace apollo
