/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/localization/common/rigid_transform_helper.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/transform/transform_query.h"

namespace {
constexpr float kDefaultRigidTransformTimeoutSec = 1.0f;
}  // namespace

namespace apollo {
namespace localization {
namespace common {

bool LookupStaticTransform(const std::string& target_frame_id,
                           const std::string& source_frame_id,
                           Eigen::Affine3d* transform,
                           float timeout_sec) {
  CHECK_NOTNULL(transform);

  if (target_frame_id.empty() || source_frame_id.empty()) {
    AERROR << "Static TF lookup requires non-empty frame ids. target frame: "
           << target_frame_id << ", source frame: " << source_frame_id;
    return false;
  }

  const float effective_timeout =
      timeout_sec > 0.0f ? timeout_sec : kDefaultRigidTransformTimeoutSec;
  const apollo::cyber::Time query_time(0);

  apollo::transform::TransformQuery transform_query;
  std::string err_msg;
  if (!transform_query.LookupTransformToAffine(
          target_frame_id, source_frame_id, query_time, transform,
          effective_timeout, &err_msg)) {
    AERROR << "Failed to resolve rigid TF from " << source_frame_id << " to "
           << target_frame_id << ": " << err_msg;
    return false;
  }

  return true;
}

std::string GetPointCloudFrameId(const drivers::PointCloud& point_cloud) {
  if (!point_cloud.frame_id().empty()) {
    return point_cloud.frame_id();
  }
  if (point_cloud.has_header() && point_cloud.header().has_frame_id()) {
    return point_cloud.header().frame_id();
  }
  return "";
}

}  // namespace common
}  // namespace localization
}  // namespace apollo