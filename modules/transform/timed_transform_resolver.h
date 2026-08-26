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

#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

#include "Eigen/Geometry"

#include "modules/transform/transform_query.h"

namespace apollo {
namespace transform {

struct StampedTransform {
  double timestamp = 0.0;
  Eigen::Translation3d translation;
  Eigen::Quaterniond rotation;

  Eigen::Affine3d ToAffine() const {
    return translation * rotation;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct TimedTransformResolverOptions {
  float tf2_buffer_size_sec = 0.01f;
  double cache_duration_sec = 1.0;
  double max_extrapolation_latency_sec = 0.15;
  double latest_lookup_fallback_tolerance_sec = 0.015;
  bool enable_extrapolation = true;
  bool hardware_trigger = true;
};

class TransformCache {
 public:
  TransformCache() = default;
  ~TransformCache() = default;

  void AddTransform(const StampedTransform& transform);
  bool QueryTransform(double timestamp, StampedTransform* transform,
                      double max_duration = 0.0,
                      double* extrapolation_duration = nullptr);

  void SetCacheDuration(double duration) { cache_duration_ = duration; }

 private:
  std::deque<StampedTransform> transforms_;
  double cache_duration_ = 1.0;
};

class TimedTransformResolver {
 public:
  TimedTransformResolver() = default;
  explicit TimedTransformResolver(TransformQuery* query)
      : transform_query_(query) {}
  ~TimedTransformResolver() = default;

  void SetTransformQuery(TransformQuery* query) { transform_query_ = query; }
  void SetOptions(const TimedTransformResolverOptions& options);

  bool Resolve(double timestamp, const std::string& frame_id,
               const std::string& child_frame_id, StampedTransform* transform);

  bool Resolve(double timestamp, const std::string& frame_id,
               const std::string& child_frame_id, Eigen::Affine3d* transform);

  TimedTransformResolverDiagnostics GetDiagnosticsSnapshot() const;

 private:
  void RecordResolveStatus(TimedTransformResolveStatus status,
                           const std::string& error = "",
                           double cache_extrapolation_sec = 0.0) const;

  TransformCache* GetTransformCache(const std::string& frame_id,
                                    const std::string& child_frame_id);

  bool QueryTransform(double timestamp, const std::string& frame_id,
                      const std::string& child_frame_id,
                      StampedTransform* transform,
                      TimedTransformResolveStatus* status,
                      std::string* error);

  TransformQuery* transform_query_ = nullptr;
  TimedTransformResolverOptions options_;
  std::unordered_map<std::string, TransformCache> transform_caches_;
  mutable std::atomic<uint64_t> resolve_calls_{0};
  mutable std::atomic<uint64_t> tf2_lookup_success_{0};
  mutable std::atomic<uint64_t> latest_fallback_success_{0};
  mutable std::atomic<uint64_t> cache_interpolation_success_{0};
  mutable std::atomic<uint64_t> cache_extrapolation_success_{0};
  mutable std::atomic<uint64_t> failed_tf2_lookup_{0};
  mutable std::atomic<uint64_t> failed_cache_miss_{0};
  mutable std::atomic<TimedTransformResolveStatus> last_status_{
      TimedTransformResolveStatus::kNotStarted};
  mutable std::mutex error_mutex_;
  mutable double max_cache_extrapolation_sec_{0.0};
  mutable std::string last_error_;
};

}  // namespace transform
}  // namespace apollo
