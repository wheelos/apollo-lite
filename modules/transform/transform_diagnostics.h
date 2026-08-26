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

#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace apollo {
namespace transform {

struct TransformEdgeDiagnostics {
  std::string frame_id;
  std::string child_frame_id;
  bool is_static = false;
  double latest_timestamp_sec = 0.0;
  uint64_t received_count = 0;

  std::string ToJsonString() const {
    std::ostringstream oss;
    oss << "{\"frame_id\":\"" << frame_id << "\",\"child_frame_id\":\""
        << child_frame_id
        << "\",\"is_static\":" << (is_static ? "true" : "false")
        << ",\"latest_timestamp_sec\":" << std::fixed << std::setprecision(6)
        << latest_timestamp_sec << ",\"received_count\":" << received_count
        << "}";
    return oss.str();
  }
};

struct TransformBufferDiagnostics {
  std::string frames_as_string;
  std::string frames_as_yaml;
  double last_update_sec = 0.0;
  uint64_t dynamic_message_count = 0;
  uint64_t static_message_count = 0;
  uint64_t dynamic_transform_count = 0;
  uint64_t static_transform_count = 0;
  uint64_t time_jump_count = 0;
  std::vector<TransformEdgeDiagnostics> edges;

  std::string ToJsonString() const {
    std::ostringstream oss;
    oss << "{\"last_update_sec\":" << std::fixed << std::setprecision(6)
        << last_update_sec
        << ",\"dynamic_message_count\":" << dynamic_message_count
        << ",\"static_message_count\":" << static_message_count
        << ",\"dynamic_transform_count\":" << dynamic_transform_count
        << ",\"static_transform_count\":" << static_transform_count
        << ",\"time_jump_count\":" << time_jump_count
        << ",\"edge_count\":" << edges.size() << ",\"edges\":[";
    for (size_t i = 0; i < edges.size(); ++i) {
      if (i > 0) oss << ",";
      oss << edges[i].ToJsonString();
    }
    oss << "]}";
    return oss.str();
  }
};

struct TransformQueryDiagnostics {
  uint64_t can_transform_calls = 0;
  uint64_t can_transform_success = 0;
  uint64_t lookup_transform_calls = 0;
  uint64_t lookup_transform_success = 0;
  uint64_t affine_lookup_calls = 0;
  uint64_t affine_lookup_success = 0;
  uint64_t static_lookup_calls = 0;
  uint64_t static_lookup_success = 0;
  std::string last_error;

  std::string ToJsonString() const {
    std::ostringstream oss;
    oss << "{\"can_transform_calls\":" << can_transform_calls
        << ",\"can_transform_success\":" << can_transform_success
        << ",\"lookup_transform_calls\":" << lookup_transform_calls
        << ",\"lookup_transform_success\":" << lookup_transform_success
        << ",\"affine_lookup_calls\":" << affine_lookup_calls
        << ",\"affine_lookup_success\":" << affine_lookup_success
        << ",\"static_lookup_calls\":" << static_lookup_calls
        << ",\"static_lookup_success\":" << static_lookup_success
        << ",\"last_error\":\"" << last_error << "\"}";
    return oss.str();
  }
};

enum class TimedTransformResolveStatus {
  kNotStarted = 0,
  kTf2Lookup,
  kLatestFallback,
  kCacheInterpolation,
  kCacheExtrapolation,
  kFailedTf2Lookup,
  kFailedCacheMiss,
};

inline const char* ToString(TimedTransformResolveStatus status) {
  switch (status) {
    case TimedTransformResolveStatus::kNotStarted:
      return "not_started";
    case TimedTransformResolveStatus::kTf2Lookup:
      return "tf2_lookup";
    case TimedTransformResolveStatus::kLatestFallback:
      return "latest_fallback";
    case TimedTransformResolveStatus::kCacheInterpolation:
      return "cache_interpolation";
    case TimedTransformResolveStatus::kCacheExtrapolation:
      return "cache_extrapolation";
    case TimedTransformResolveStatus::kFailedTf2Lookup:
      return "failed_tf2_lookup";
    case TimedTransformResolveStatus::kFailedCacheMiss:
      return "failed_cache_miss";
  }
  return "unknown";
}

struct TimedTransformResolverDiagnostics {
  TimedTransformResolveStatus last_status =
      TimedTransformResolveStatus::kNotStarted;
  uint64_t resolve_calls = 0;
  uint64_t tf2_lookup_success = 0;
  uint64_t latest_fallback_success = 0;
  uint64_t cache_interpolation_success = 0;
  uint64_t cache_extrapolation_success = 0;
  uint64_t failed_tf2_lookup = 0;
  uint64_t failed_cache_miss = 0;
  double max_cache_extrapolation_sec = 0.0;
  std::string last_error;

  std::string ToJsonString() const {
    std::ostringstream oss;
    oss << "{\"last_status\":\"" << ToString(last_status) << "\""
        << ",\"resolve_calls\":" << resolve_calls
        << ",\"tf2_lookup_success\":" << tf2_lookup_success
        << ",\"latest_fallback_success\":" << latest_fallback_success
        << ",\"cache_interpolation_success\":" << cache_interpolation_success
        << ",\"cache_extrapolation_success\":" << cache_extrapolation_success
        << ",\"failed_tf2_lookup\":" << failed_tf2_lookup
        << ",\"failed_cache_miss\":" << failed_cache_miss
        << ",\"max_cache_extrapolation_sec\":" << std::fixed
        << std::setprecision(6) << max_cache_extrapolation_sec
        << ",\"last_error\":\"" << last_error << "\"}";
    return oss.str();
  }
};

}  // namespace transform
}  // namespace apollo
