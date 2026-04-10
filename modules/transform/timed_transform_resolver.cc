// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2026-04-10
//  Author: daohu527

#include "modules/transform/timed_transform_resolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace transform {

Eigen::Quaterniond Slerp(const Eigen::Quaterniond& source, const double& t,
                         const Eigen::Quaterniond& other);

namespace {

std::string BuildTransformCacheKey(const std::string& frame_id,
                                   const std::string& child_frame_id) {
  return frame_id + "\n" + child_frame_id;
}

void InterpolateStampedTransform(const StampedTransform& first,
                                 const StampedTransform& second,
                                 double timestamp,
                                 StampedTransform* transform) {
  CHECK_NOTNULL(transform);

  const double duration = second.timestamp - first.timestamp;
  if (duration <= std::numeric_limits<double>::epsilon()) {
    *transform = second;
    transform->timestamp = timestamp;
    return;
  }

  const double ratio = (timestamp - first.timestamp) / duration;
  transform->timestamp = timestamp;
  transform->rotation = Slerp(first.rotation, ratio, second.rotation);
  transform->translation.x() =
      first.translation.x() * (1 - ratio) + second.translation.x() * ratio;
  transform->translation.y() =
      first.translation.y() * (1 - ratio) + second.translation.y() * ratio;
  transform->translation.z() =
      first.translation.z() * (1 - ratio) + second.translation.z() * ratio;
}

}  // namespace

void TransformCache::AddTransform(const StampedTransform& transform) {
  if (transforms_.empty()) {
    transforms_.push_back(transform);
    return;
  }
  double delt = transform.timestamp - transforms_.back().timestamp;
  if (delt < 0.0) {
    ADEBUG << "Ignore out-of-order transform at " << transform.timestamp
           << " for transform cache.";
    return;
  }

  if (delt <= std::numeric_limits<double>::epsilon()) {
    transforms_.back() = transform;
    return;
  }

  do {
    delt = transform.timestamp - transforms_.front().timestamp;
    if (delt < cache_duration_) {
      break;
    }
    transforms_.pop_front();
  } while (!transforms_.empty());

  transforms_.push_back(transform);
}

Eigen::Quaterniond Slerp(const Eigen::Quaterniond& source, const double& t,
                         const Eigen::Quaterniond& other) {
  const double one = 1.0 - std::numeric_limits<double>::epsilon();
  double d = source.x() * other.x() + source.y() * other.y() +
             source.z() * other.z() + source.w() * other.w();
  double abs_d = std::abs(d);

  double scale0;
  double scale1;

  if (abs_d >= one) {
    scale0 = 1.0 - t;
    scale1 = t;
  } else {
    double theta = std::acos(abs_d);
    double sin_theta = std::sin(theta);

    scale0 = std::sin((1.0 - t) * theta) / sin_theta;
    scale1 = std::sin((t * theta)) / sin_theta;
  }
  if (d < 0) {
    scale1 = -scale1;
  }

  return Eigen::Quaterniond(scale0 * source.w() + scale1 * other.w(),
                            scale0 * source.x() + scale1 * other.x(),
                            scale0 * source.y() + scale1 * other.y(),
                            scale0 * source.z() + scale1 * other.z());
}

bool TransformCache::QueryTransform(double timestamp,
                                    StampedTransform* transform,
                                    double max_duration) {
  if (transforms_.empty() || transform == nullptr) {
    return false;
  }

  if (timestamp < transforms_.front().timestamp) {
    ADEBUG << "Transform cache miss: requested timestamp " << timestamp
           << " is earlier than cached range.";
    return false;
  }

  const double delt = timestamp - transforms_.back().timestamp;
  if (delt > max_duration) {
    ADEBUG << "Transform cache miss: requested timestamp is " << delt
           << "s newer than the latest cached sample.";
    return false;
  }

  const int size = static_cast<int>(transforms_.size());
  if (size == 1) {
    *transform = transforms_.back();
    transform->timestamp = timestamp;
    ADEBUG << "Reuse latest cached transform at "
           << transforms_.back().timestamp << " for " << timestamp << ".";
    return true;
  }

  if (timestamp >= transforms_.back().timestamp) {
    InterpolateStampedTransform(transforms_[size - 2], transforms_[size - 1],
                                timestamp, transform);
    ADEBUG << "Extrapolate cached transform at " << timestamp
           << " using samples at " << transforms_[size - 2].timestamp
           << " and " << transforms_[size - 1].timestamp << ".";
    return true;
  }

  for (int index = 1; index < size; ++index) {
    const auto& previous = transforms_[index - 1];
    const auto& current = transforms_[index];
    if (timestamp <= current.timestamp) {
      if (std::abs(timestamp - current.timestamp) <=
          std::numeric_limits<double>::epsilon()) {
        *transform = current;
        transform->timestamp = timestamp;
      } else {
        InterpolateStampedTransform(previous, current, timestamp, transform);
      }
      return true;
    }
  }

  return false;
}

void TimedTransformResolver::SetOptions(
    const TimedTransformResolverOptions& options) {
  options_ = options;
  options_.tf2_buffer_size_sec = std::max(0.0f, options_.tf2_buffer_size_sec);
  options_.cache_duration_sec = std::max(0.0, options_.cache_duration_sec);
  options_.max_extrapolation_latency_sec =
      std::max(0.0, options_.max_extrapolation_latency_sec);
  options_.latest_lookup_fallback_tolerance_sec =
      std::max(0.0, options_.latest_lookup_fallback_tolerance_sec);
  for (auto& entry : transform_caches_) {
    entry.second.SetCacheDuration(options_.cache_duration_sec);
  }
}

TransformCache* TimedTransformResolver::GetTransformCache(
    const std::string& frame_id, const std::string& child_frame_id) {
  const std::string cache_key =
      BuildTransformCacheKey(frame_id, child_frame_id);
  auto [iter, inserted] = transform_caches_.try_emplace(cache_key);
  if (inserted) {
    iter->second.SetCacheDuration(options_.cache_duration_sec);
  }
  return &iter->second;
}

bool TimedTransformResolver::Resolve(double timestamp,
                                     const std::string& frame_id,
                                     const std::string& child_frame_id,
                                     StampedTransform* transform) {
  CHECK_NOTNULL(transform_query_);
  CHECK_NOTNULL(transform);

  if (QueryTransform(timestamp, frame_id, child_frame_id, transform)) {
    if (options_.enable_extrapolation) {
      GetTransformCache(frame_id, child_frame_id)->AddTransform(*transform);
    }
    return true;
  }

  if (!options_.enable_extrapolation) {
    return false;
  }

  return GetTransformCache(frame_id, child_frame_id)
      ->QueryTransform(timestamp, transform,
                       options_.max_extrapolation_latency_sec);
}

bool TimedTransformResolver::QueryTransform(double timestamp,
                                            const std::string& frame_id,
                                            const std::string& child_frame_id,
                                            StampedTransform* transform) {
  CHECK_NOTNULL(transform_query_);
  CHECK_NOTNULL(transform);

  cyber::Time query_time(timestamp);
  std::string err_string;
  if (!transform_query_->CanTransform(frame_id, child_frame_id, query_time,
                                      options_.tf2_buffer_size_sec,
                                      &err_string)) {
    apollo::transform::TransformStamped latest_transform;
    double latest_buffer_time = 0.0;
    if (!options_.hardware_trigger) {
      std::string lookup_error;
      if (!transform_query_->LookupTransform(
              frame_id, child_frame_id, apollo::cyber::Time(0),
              &latest_transform, options_.tf2_buffer_size_sec, &lookup_error)) {
        AERROR << lookup_error;
        return false;
      }
      latest_buffer_time = latest_transform.header().timestamp_sec();
    }
    if (!cyber::common::GlobalData::Instance()->IsRealityMode()) {
      query_time = cyber::Time(0);
    } else if (!options_.hardware_trigger &&
               (timestamp - latest_buffer_time <
                options_.latest_lookup_fallback_tolerance_sec) &&
               (timestamp - latest_buffer_time > 0)) {
      query_time = apollo::cyber::Time(0);
    } else {
      AERROR << "Can not find transform. " << FORMAT_TIMESTAMP(timestamp)
             << " frame_id: " << frame_id
             << " child_frame_id: " << child_frame_id
             << " Error info: " << err_string;
      return false;
    }
  }

  apollo::transform::TransformStamped stamped_transform;
  std::string lookup_error;
  if (!transform_query_->LookupTransform(
          frame_id, child_frame_id, query_time, &stamped_transform,
          options_.tf2_buffer_size_sec, &lookup_error)) {
    AERROR << lookup_error;
    return false;
  }

  transform->timestamp = stamped_transform.header().timestamp_sec();
  transform->translation =
      Eigen::Translation3d(stamped_transform.transform().translation().x(),
                           stamped_transform.transform().translation().y(),
                           stamped_transform.transform().translation().z());
  transform->rotation =
      Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
                         stamped_transform.transform().rotation().qx(),
                         stamped_transform.transform().rotation().qy(),
                         stamped_transform.transform().rotation().qz());
  return true;
}

}  // namespace transform
}  // namespace apollo
