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
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/transform/internal/transform_cache.h"

namespace apollo {
namespace transform {

struct TimedTransformResolver::CacheState {
  internal::TransformCache* Get(const std::string& target,
                                const std::string& source, double duration) {
    auto [iter, inserted] = caches.try_emplace(target + "\n" + source);
    if (inserted) {
      iter->second.SetCacheDuration(duration);
    }
    return &iter->second;
  }
  std::unordered_map<std::string, internal::TransformCache> caches;
};

TimedTransformResolver::TimedTransformResolver()
    : cache_state_(std::make_unique<CacheState>()) {}

TimedTransformResolver::TimedTransformResolver(TransformQuery* query)
    : TimedTransformResolver() {
  SetTransformQuery(query);
}

TimedTransformResolver::~TimedTransformResolver() = default;

void TimedTransformResolver::SetTransformQuery(TransformQuery* query) {
  CHECK_NOTNULL(query);
  cache_state_->caches.clear();
  transform_query_ = query;
  time_epoch_ = query->GetTimeEpoch();
}

bool TimedTransformResolver::SetOptions(
    const TimedTransformResolverOptions& options) {
  cache_state_->caches.clear();
  options_valid_ =
      std::isfinite(options.query_timeout_sec) &&
      options.query_timeout_sec >= 0.0f &&
      options.query_timeout_sec <=
          std::chrono::duration<double>(
              std::chrono::steady_clock::duration::max() / 2)
              .count() &&
      std::isfinite(options.cache_duration_sec) &&
      options.cache_duration_sec >= 0.0 &&
      std::isfinite(options.max_extrapolation_sec) &&
      options.max_extrapolation_sec >= 0.0 &&
      std::isfinite(options.latest_lookup_fallback_tolerance_sec) &&
      options.latest_lookup_fallback_tolerance_sec >= 0.0;
  if (!options_valid_) {
    AERROR << "Transform resolver budgets must be finite and nonnegative.";
    return false;
  }
  options_ = options;
  return true;
}

void TimedTransformResolver::RecordResolveStatus(
    TimedTransformResolveStatus status, const std::string& error,
    double cache_extrapolation_sec) const {
  resolve_calls_.fetch_add(1, std::memory_order_relaxed);
  last_status_.store(status, std::memory_order_relaxed);
  switch (status) {
    case TimedTransformResolveStatus::kTf2Lookup:
      tf2_lookup_success_.fetch_add(1, std::memory_order_relaxed);
      break;
    case TimedTransformResolveStatus::kLatestFallback:
      latest_fallback_success_.fetch_add(1, std::memory_order_relaxed);
      break;
    case TimedTransformResolveStatus::kCacheInterpolation:
      cache_interpolation_success_.fetch_add(1, std::memory_order_relaxed);
      break;
    case TimedTransformResolveStatus::kCacheExtrapolation:
      cache_extrapolation_success_.fetch_add(1, std::memory_order_relaxed);
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        max_cache_extrapolation_sec_ =
            std::max(max_cache_extrapolation_sec_, cache_extrapolation_sec);
      }
      break;
    case TimedTransformResolveStatus::kFailedTf2Lookup:
      failed_tf2_lookup_.fetch_add(1, std::memory_order_relaxed);
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = error;
      }
      break;
    case TimedTransformResolveStatus::kFailedCacheMiss:
      failed_cache_miss_.fetch_add(1, std::memory_order_relaxed);
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = error;
      }
      break;
    case TimedTransformResolveStatus::kNotStarted:
      break;
  }
}

TimedTransformResolverDiagnostics
TimedTransformResolver::GetDiagnosticsSnapshot() const {
  TimedTransformResolverDiagnostics diag;
  diag.resolve_calls = resolve_calls_.load(std::memory_order_relaxed);
  diag.last_status = last_status_.load(std::memory_order_relaxed);
  diag.tf2_lookup_success = tf2_lookup_success_.load(std::memory_order_relaxed);
  diag.latest_fallback_success =
      latest_fallback_success_.load(std::memory_order_relaxed);
  diag.cache_interpolation_success =
      cache_interpolation_success_.load(std::memory_order_relaxed);
  diag.cache_extrapolation_success =
      cache_extrapolation_success_.load(std::memory_order_relaxed);
  diag.failed_tf2_lookup = failed_tf2_lookup_.load(std::memory_order_relaxed);
  diag.failed_cache_miss = failed_cache_miss_.load(std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    diag.max_cache_extrapolation_sec = max_cache_extrapolation_sec_;
    diag.last_error = last_error_;
  }
  return diag;
}

bool TimedTransformResolver::Resolve(double timestamp,
                                     const std::string& frame_id,
                                     const std::string& child_frame_id,
                                     StampedTransform* transform) {
  CHECK_NOTNULL(transform_query_);
  CHECK_NOTNULL(transform);

  if (!options_valid_ || !std::isfinite(timestamp) || timestamp <= 0.0 ||
      timestamp >=
          static_cast<double>(std::numeric_limits<uint64_t>::max()) / 1e9) {
    AERROR << "Timed TF requires valid options and a finite positive time.";
    RecordResolveStatus(TimedTransformResolveStatus::kFailedTf2Lookup,
                        "invalid timed transform request or options");
    return false;
  }
  const auto epoch = transform_query_->GetTimeEpoch();
  if (epoch != time_epoch_) {
    cache_state_->caches.clear();
    time_epoch_ = epoch;
  }
  TimedTransformResolveStatus status = TimedTransformResolveStatus::kTf2Lookup;
  std::string error;
  StampedTransform candidate;
  const bool found = QueryTransform(timestamp, frame_id, child_frame_id,
                                    &candidate, &status, &error);
  if (transform_query_->GetTimeEpoch() != epoch) {
    cache_state_->caches.clear();
    RecordResolveStatus(TimedTransformResolveStatus::kFailedTf2Lookup,
                        "TF time epoch changed during lookup");
    return false;
  }
  if (found) {
    if (options_.enable_extrapolation) {
      cache_state_->Get(frame_id, child_frame_id, options_.cache_duration_sec)
          ->AddTransform(candidate);
    }
    *transform = candidate;
    RecordResolveStatus(status);
    return true;
  }

  if (!options_.enable_extrapolation) {
    RecordResolveStatus(TimedTransformResolveStatus::kFailedTf2Lookup, error);
    return false;
  }

  double extrapolation_duration = 0.0;
  const bool success =
      cache_state_->Get(frame_id, child_frame_id, options_.cache_duration_sec)
          ->QueryTransform(timestamp, &candidate,
                           options_.max_extrapolation_sec,
                           &extrapolation_duration);
  if (!success) {
    RecordResolveStatus(TimedTransformResolveStatus::kFailedCacheMiss, error);
    return false;
  }
  if (transform_query_->GetTimeEpoch() != epoch) {
    cache_state_->caches.clear();
    RecordResolveStatus(TimedTransformResolveStatus::kFailedCacheMiss,
                        "TF time epoch changed during cache lookup");
    return false;
  }
  *transform = candidate;
  RecordResolveStatus(extrapolation_duration > 0.0
                          ? TimedTransformResolveStatus::kCacheExtrapolation
                          : TimedTransformResolveStatus::kCacheInterpolation,
                      "", extrapolation_duration);
  return true;
}

bool TimedTransformResolver::Resolve(double timestamp,
                                     const std::string& frame_id,
                                     const std::string& child_frame_id,
                                     Eigen::Affine3d* transform) {
  if (transform == nullptr) {
    AERROR << "Null timed transform output.";
    return false;
  }
  StampedTransform stamped_tf;
  if (!Resolve(timestamp, frame_id, child_frame_id, &stamped_tf)) {
    return false;
  }
  *transform = stamped_tf.ToAffine();
  return true;
}

bool TimedTransformResolver::QueryTransform(double timestamp,
                                            const std::string& frame_id,
                                            const std::string& child_frame_id,
                                            StampedTransform* transform,
                                            TimedTransformResolveStatus* status,
                                            std::string* error) {
  CHECK_NOTNULL(transform_query_);
  CHECK_NOTNULL(transform);

  const cyber::Time query_time(timestamp);
  apollo::transform::TransformStamped stamped_transform;
  std::string err_string;
  if (!transform_query_->CanTransform(frame_id, child_frame_id, query_time,
                                      options_.query_timeout_sec,
                                      &err_string)) {
    if (options_.hardware_trigger ||
        !std::isfinite(options_.latest_lookup_fallback_tolerance_sec) ||
        options_.latest_lookup_fallback_tolerance_sec <= 0.0) {
      AERROR << "Can not find transform. " << FORMAT_TIMESTAMP(timestamp)
             << " frame_id: " << frame_id
             << " child_frame_id: " << child_frame_id
             << " Error info: " << err_string;
      if (error != nullptr) {
        *error = err_string;
      }
      return false;
    }
    if (!transform_query_->LookupTransform(
            frame_id, child_frame_id, cyber::Time(0), &stamped_transform,
            options_.query_timeout_sec, &err_string)) {
      AERROR << err_string;
      if (error != nullptr) {
        *error = err_string;
      }
      return false;
    }
    const double lag = timestamp - stamped_transform.header().timestamp_sec();
    if (!std::isfinite(lag) || lag < 0.0 ||
        lag > options_.latest_lookup_fallback_tolerance_sec) {
      AERROR << "Latest transform outside fallback tolerance. requested: "
             << FORMAT_TIMESTAMP(timestamp) << " lag: " << lag;
      if (error != nullptr) {
        *error = "latest transform outside fallback tolerance";
      }
      return false;
    }
    if (status != nullptr) {
      *status = TimedTransformResolveStatus::kLatestFallback;
    }
  } else {
    if (!transform_query_->LookupTransform(
            frame_id, child_frame_id, query_time, &stamped_transform,
            options_.query_timeout_sec, &err_string)) {
      AERROR << err_string;
      if (error != nullptr) {
        *error = err_string;
      }
      return false;
    }
    if (status != nullptr) {
      *status = TimedTransformResolveStatus::kTf2Lookup;
    }
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
