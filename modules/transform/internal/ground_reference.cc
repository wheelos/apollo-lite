// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/ground_reference.h"

#include <cmath>
#include <string>

#include "cyber/common/log.h"

namespace apollo {
namespace transform {

bool QueryGroundTransform(const TransformQuery& query,
                          const std::string& reference_frame,
                          double reference_height,
                          const std::string& source_frame,
                          Eigen::Affine3d* ground_from_source,
                          float timeout_sec) {
  if (ground_from_source == nullptr || reference_frame.empty() ||
      source_frame.empty() || !std::isfinite(reference_height) ||
      reference_height <= 0.0) {
    AERROR << "Ground transform requires frames, positive height and output.";
    return false;
  }
  Eigen::Affine3d base_from_reference;
  Eigen::Affine3d base_from_source;
  if (!query.GetLatestStaticTransformToAffine(
          "base_link", reference_frame, &base_from_reference, timeout_sec) ||
      !query.GetLatestStaticTransformToAffine("base_link", source_frame,
                                              &base_from_source, timeout_sec)) {
    AERROR << "Missing static calibration for ground transform: "
           << reference_frame << ", " << source_frame;
    return false;
  }
  Eigen::Affine3d candidate = base_from_source;
  candidate.translation().z() +=
      reference_height - base_from_reference.translation().z();
  if (!candidate.matrix().allFinite()) {
    AERROR << "Non-finite ground transform: " << source_frame;
    return false;
  }
  *ground_from_source = candidate;
  return true;
}

}  // namespace transform
}  // namespace apollo
