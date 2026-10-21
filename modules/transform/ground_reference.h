// Copyright 2026 WheelOS. All Rights Reserved.
#pragma once

#include <string>

#include "Eigen/Geometry"

#include "modules/transform/transform_query.h"

namespace apollo {
namespace transform {

// The calibration ground plane is normal to base_link +Z. Its origin is
// directly below base_link; this is not a runtime terrain/slope estimator.
bool QueryGroundTransform(const TransformQuery& query,
                          const std::string& reference_frame,
                          double reference_height,
                          const std::string& source_frame,
                          Eigen::Affine3d* ground_from_source,
                          float timeout_sec = 0.0f);

}  // namespace transform
}  // namespace apollo
