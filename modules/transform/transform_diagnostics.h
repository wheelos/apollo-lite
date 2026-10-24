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

  std::string ToJsonString() const;
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
  uint64_t rejected_message_count = 0;
  std::vector<TransformEdgeDiagnostics> edges;

  std::string ToJsonString() const;
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

  std::string ToJsonString() const;
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

const char* ToString(TimedTransformResolveStatus status);

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

  std::string ToJsonString() const;
};

}  // namespace transform
}  // namespace apollo
