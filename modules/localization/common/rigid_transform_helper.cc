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

#include <chrono>
#include <thread>

#include "cyber/cyber.h"
#include "cyber/common/log.h"
#include "modules/transform/transform_query.h"

namespace {

constexpr float kDefaultStaticTransformTimeoutSec = 1.0f;
constexpr auto kStaticTransformRetryInterval = std::chrono::milliseconds(10);

}  // namespace

namespace apollo {
namespace localization {
namespace common {

bool LookupStaticTransform(const std::string& target_frame_id,
                           const std::string& source_frame_id,
                           Eigen::Affine3d* transform, float timeout_sec) {
  CHECK_NOTNULL(transform);

  if (target_frame_id.empty() || source_frame_id.empty()) {
    AERROR << "Static TF lookup requires non-empty frame ids. target frame: "
           << target_frame_id << ", source frame: " << source_frame_id;
    return false;
  }

  apollo::transform::TransformQuery transform_query;
  const float effective_timeout = timeout_sec > 0.0f
                                      ? timeout_sec
                                      : kDefaultStaticTransformTimeoutSec;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(effective_timeout);

  do {
    if (transform_query.GetLatestStaticTransformToAffine(
            target_frame_id, source_frame_id, transform)) {
      return true;
    }
    if (apollo::cyber::IsShutdown() ||
        std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(kStaticTransformRetryInterval);
  } while (std::chrono::steady_clock::now() < deadline);

  if (transform_query.GetLatestStaticTransformToAffine(
          target_frame_id, source_frame_id, transform)) {
    return true;
  }

  AERROR << "Failed to resolve rigid static TF from " << source_frame_id
         << " to " << target_frame_id << " within " << effective_timeout
         << "s.";
  return false;
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
