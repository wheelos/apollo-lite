// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/transform_diagnostics.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "nlohmann/json.hpp"

namespace apollo {
namespace transform {

std::string TransformEdgeDiagnostics::ToJsonString() const {
  std::ostringstream oss;
  oss << "{\"frame_id\":" << nlohmann::json(frame_id).dump()
      << ",\"child_frame_id\":" << nlohmann::json(child_frame_id).dump()
      << ",\"is_static\":" << (is_static ? "true" : "false")
      << ",\"latest_timestamp_sec\":" << std::fixed << std::setprecision(6)
      << latest_timestamp_sec << ",\"received_count\":" << received_count
      << "}";
  return oss.str();
}

std::string TransformBufferDiagnostics::ToJsonString() const {
  std::ostringstream oss;
  oss << "{\"last_update_sec\":" << std::fixed << std::setprecision(6)
      << last_update_sec
      << ",\"dynamic_message_count\":" << dynamic_message_count
      << ",\"static_message_count\":" << static_message_count
      << ",\"dynamic_transform_count\":" << dynamic_transform_count
      << ",\"static_transform_count\":" << static_transform_count
      << ",\"time_jump_count\":" << time_jump_count
      << ",\"rejected_message_count\":" << rejected_message_count
      << ",\"edge_count\":" << edges.size() << ",\"edges\":[";
  for (size_t i = 0; i < edges.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << edges[i].ToJsonString();
  }
  oss << "]}";
  return oss.str();
}

std::string TransformQueryDiagnostics::ToJsonString() const {
  std::ostringstream oss;
  oss << "{\"can_transform_calls\":" << can_transform_calls
      << ",\"can_transform_success\":" << can_transform_success
      << ",\"lookup_transform_calls\":" << lookup_transform_calls
      << ",\"lookup_transform_success\":" << lookup_transform_success
      << ",\"affine_lookup_calls\":" << affine_lookup_calls
      << ",\"affine_lookup_success\":" << affine_lookup_success
      << ",\"static_lookup_calls\":" << static_lookup_calls
      << ",\"static_lookup_success\":" << static_lookup_success
      << ",\"last_error\":" << nlohmann::json(last_error).dump() << "}";
  return oss.str();
}

const char* ToString(TimedTransformResolveStatus status) {
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

std::string TimedTransformResolverDiagnostics::ToJsonString() const {
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
      << ",\"last_error\":" << nlohmann::json(last_error).dump() << "}";
  return oss.str();
}

}  // namespace transform
}  // namespace apollo
