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

#include <cmath>
#include <iomanip>
#include <sstream>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "modules/common/util/string_util.h"

namespace apollo {
namespace transform {

namespace {

constexpr int kTransformMissLogFrequency = 10;
constexpr double kTimeCompareEpsilonSec = 1e-6;

std::string FormatTimestampForLog(double timestamp_sec) {
  std::ostringstream stream;
  if (!std::isfinite(timestamp_sec) || timestamp_sec < 0.0) {
    stream << timestamp_sec << "s";
    return stream.str();
  }

  stream << std::fixed << std::setprecision(9) << timestamp_sec << "s";
  if (timestamp_sec > 0.0) {
    stream << " [" << cyber::Time(timestamp_sec).ToString() << "]";
  }
  return stream.str();
}

std::string FormatDeltaForLog(double delta_sec) {
  const double abs_delta_sec = std::fabs(delta_sec);
  std::ostringstream stream;
  stream << std::showpos << std::fixed
         << std::setprecision(abs_delta_sec >= 1.0 ? 3 : 6) << delta_sec
         << "s" << std::noshowpos;
  if (abs_delta_sec >= 86400.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 86400.0 << " days)";
  } else if (abs_delta_sec >= 3600.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 3600.0 << " h)";
  } else if (abs_delta_sec >= 60.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec / 60.0 << " min)";
  } else if (abs_delta_sec > 0.0 && abs_delta_sec < 1.0) {
    stream << " (~" << std::fixed << std::setprecision(3)
           << abs_delta_sec * 1000.0 << " ms)";
  }
  return stream.str();
}

const char* DescribeTimeRelation(double delta_sec) {
  if (std::fabs(delta_sec) <= kTimeCompareEpsilonSec) {
    return "aligned";
  }
  return delta_sec > 0.0 ? "ahead" : "behind";
}

bool TryExtractBoundaryTimeSec(const std::string& reason,
                               double* boundary_time_sec,
                               std::string* boundary_label) {
  RETURN_VAL_IF_NULL2(boundary_time_sec, false);
  RETURN_VAL_IF_NULL2(boundary_label, false);

  struct Marker {
    const char* needle;
    const char* label;
  };
  constexpr Marker kMarkers[] = {
      {"latest data is at time ", "latest_tf"},
      {"earliest data is at time ", "earliest_tf"},
  };

  for (const auto& marker : kMarkers) {
    const size_t marker_pos = reason.find(marker.needle);
    if (marker_pos == std::string::npos) {
      continue;
    }
    const size_t digits_begin = marker_pos + std::strlen(marker.needle);
    size_t digits_end = digits_begin;
    while (digits_end < reason.size() && std::isdigit(reason[digits_end])) {
      ++digits_end;
    }
    if (digits_end == digits_begin) {
      continue;
    }
    try {
      const uint64_t nanoseconds =
          std::stoull(reason.substr(digits_begin, digits_end - digits_begin));
      *boundary_time_sec = static_cast<double>(nanoseconds) / 1e9;
      *boundary_label = marker.label;
      return true;
    } catch (const std::exception&) {
      return false;
    }
  }

  return false;
}

void LogTransformQueryFailure(const std::string& target_frame,
                              const std::string& source_frame,
                              double requested_time_sec,
                              const cyber::Time& lookup_time,
                              float timeout_sec,
                              const std::string& reason) {
  const double now_sec = cyber::Time::Now().ToSecond();
  const double request_vs_now_sec = requested_time_sec - now_sec;

  std::ostringstream message;
  message << "Transform unavailable target=" << target_frame
          << " source=" << source_frame
          << ", requested=" << FormatTimestampForLog(requested_time_sec);
  if (lookup_time.IsZero() && requested_time_sec > 0.0) {
    message << ", lookup=latest(0)";
  } else {
    message << ", lookup=" << FormatTimestampForLog(lookup_time.ToSecond());
  }

  double boundary_time_sec = 0.0;
  std::string boundary_label;
  if (TryExtractBoundaryTimeSec(reason, &boundary_time_sec, &boundary_label)) {
    const double request_vs_boundary_sec =
        requested_time_sec - boundary_time_sec;
    message << ", " << boundary_label << "="
            << FormatTimestampForLog(boundary_time_sec)
            << ", requested_vs_" << boundary_label << "="
            << FormatDeltaForLog(request_vs_boundary_sec) << " ("
            << DescribeTimeRelation(request_vs_boundary_sec) << ")";
  }

  message << ", requested_vs_now=" << FormatDeltaForLog(request_vs_now_sec)
          << " (" << DescribeTimeRelation(request_vs_now_sec) << ")"
          << ", timeout=" << std::fixed << std::setprecision(3)
          << timeout_sec << "s"
          << ", reason=" << reason;
  AINFO_EVERY(kTransformMissLogFrequency) << message.str();
}

}  // namespace

TimedTransformResolver::TimedTransformResolver(
    BufferInterface* buffer, const std::string& target_frame,
    const std::string& source_frame,
    const TimedTransformResolverOptions& options) {
  Configure(buffer, target_frame, source_frame, options);
}

void TimedTransformResolver::SetTransformQuery(TransformQuery* query) {
  owned_query_.reset();
  transform_query_ = query;
  ClearCache();
}

void TimedTransformResolver::Configure(
    BufferInterface* buffer, const std::string& target_frame,
    const std::string& source_frame,
    const TimedTransformResolverOptions& options) {
  owned_query_ = std::make_unique<TransformQuery>(buffer);
  transform_query_ = owned_query_.get();
  ConfigureFrames(target_frame, source_frame);
  SetOptions(options);
}

void TimedTransformResolver::ConfigureFrames(const std::string& target_frame,
                                             const std::string& source_frame) {
  target_frame_ = target_frame;
  source_frame_ = source_frame;
  ClearCache();
}

void TimedTransformResolver::SetOptions(
    const TimedTransformResolverOptions& options) {
  options_ = options;
  options_.query_timeout_sec = std::max(0.0f, options_.query_timeout_sec);
  options_.cache_duration_sec = std::max(0.0, options_.cache_duration_sec);
  options_.max_extrapolation_sec =
      std::max(0.0, options_.max_extrapolation_sec);
  options_.latest_lookup_fallback_tolerance_sec =
      std::max(0.0, options_.latest_lookup_fallback_tolerance_sec);
}

bool TimedTransformResolver::Resolve(double timestamp_sec,
                                     StampedTransform* transform,
                                     TransformResolveStatus* status) {
  CHECK_NOTNULL(transform);

  TransformStamped stamped_transform;
  if (QueryTransform(timestamp_sec, &stamped_transform)) {
    const double resolved_timestamp =
        stamped_transform.header().timestamp_sec() > 0.0
            ? stamped_transform.header().timestamp_sec()
            : timestamp_sec;
    const Eigen::Affine3d pose = ToAffine(stamped_transform);
    if (options_.enable_extrapolation) {
      InsertSample(resolved_timestamp, pose);
    }
    ToStamped(resolved_timestamp, pose, transform);
    if (status != nullptr) {
      *status = TransformResolveStatus::kOk;
    }
    return true;
  }

  if (!options_.enable_extrapolation) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kQueryFailed;
    }
    return false;
  }

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  if (!QueryCachedInternal(timestamp_sec, options_.max_extrapolation_sec, &pose,
                           status)) {
    return false;
  }
  ToStamped(timestamp_sec, pose, transform);
  return true;
}

bool TimedTransformResolver::Resolve(double timestamp_sec,
                                     const std::string& target_frame,
                                     const std::string& source_frame,
                                     StampedTransform* transform,
                                     TransformResolveStatus* status) {
  CHECK_NOTNULL(transform);
  if (transform_query_ == nullptr) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kQueryFailed;
    }
    return false;
  }

  TimedTransformResolver scoped_resolver(transform_query_);
  scoped_resolver.ConfigureFrames(target_frame, source_frame);
  scoped_resolver.SetOptions(options_);
  return scoped_resolver.Resolve(timestamp_sec, transform, status);
}

bool TimedTransformResolver::ResolveToAffine(double timestamp_sec,
                                             Eigen::Affine3d* pose,
                                             TransformResolveStatus* status) {
  CHECK_NOTNULL(pose);

  double resolved_timestamp_sec = timestamp_sec;
  if (QueryTransform(timestamp_sec, pose, &resolved_timestamp_sec)) {
    if (options_.enable_extrapolation) {
      InsertSample(resolved_timestamp_sec, *pose);
    }
    if (status != nullptr) {
      *status = TransformResolveStatus::kOk;
    }
    return true;
  }

  if (!options_.enable_extrapolation) {
    if (status != nullptr) {
      *status = TransformResolveStatus::kQueryFailed;
    }
    return false;
  }

  return QueryCachedInternal(timestamp_sec, options_.max_extrapolation_sec,
                             pose, status);
}

bool TimedTransformResolver::QueryTransform(double timestamp_sec,
                                            TransformStamped* transform) const {
  CHECK_NOTNULL(transform_query_);
  CHECK_NOTNULL(transform);

  if (target_frame_.empty() || source_frame_.empty()) {
    return false;
  }

  cyber::Time query_time(timestamp_sec);
  std::string err_string;
  if (!transform_query_->CanTransform(target_frame_, source_frame_, query_time,
                                      options_.query_timeout_sec,
                                      &err_string)) {
    double latest_buffer_time = 0.0;
    if (!options_.hardware_trigger) {
      std::string lookup_error;
      apollo::transform::TransformStamped latest_transform;
      if (!transform_query_->LookupTransform(
              target_frame_, source_frame_, apollo::cyber::Time(0),
              &latest_transform, options_.query_timeout_sec, &lookup_error)) {
        LogTransformQueryFailure(
            target_frame_, source_frame_, timestamp_sec, apollo::cyber::Time(0),
            options_.query_timeout_sec,
            "failed to fetch latest transform while diagnosing miss: " +
                lookup_error + "; canTransform=" + err_string);
        return false;
      }
      latest_buffer_time = latest_transform.header().timestamp_sec();
    }
    if (!cyber::common::GlobalData::Instance()->IsRealityMode()) {
      query_time = cyber::Time(0);
    } else if (!options_.hardware_trigger &&
               (timestamp_sec - latest_buffer_time <
                options_.latest_lookup_fallback_tolerance_sec) &&
               (timestamp_sec - latest_buffer_time > 0)) {
      query_time = apollo::cyber::Time(0);
    } else {
      LogTransformQueryFailure(target_frame_, source_frame_, timestamp_sec,
                               query_time, options_.query_timeout_sec,
                               err_string);
      return false;
    }
  }

  std::string lookup_error;
  if (!transform_query_->LookupTransform(
          target_frame_, source_frame_, query_time, transform,
          options_.query_timeout_sec, &lookup_error)) {
    LogTransformQueryFailure(target_frame_, source_frame_, timestamp_sec,
                             query_time, options_.query_timeout_sec,
                             lookup_error);
    return false;
  }

  return true;
}

bool TimedTransformResolver::QueryTransform(
    double timestamp_sec, Eigen::Affine3d* pose,
    double* resolved_timestamp_sec) const {
  CHECK_NOTNULL(pose);

  TransformStamped stamped_transform;
  if (!QueryTransform(timestamp_sec, &stamped_transform)) {
    return false;
  }

  *pose = ToAffine(stamped_transform);
  if (resolved_timestamp_sec != nullptr) {
    *resolved_timestamp_sec = stamped_transform.header().timestamp_sec() > 0.0
                                  ? stamped_transform.header().timestamp_sec()
                                  : timestamp_sec;
  }
  return true;
}

Eigen::Affine3d TimedTransformResolver::ToAffine(
    const TransformStamped& transform) {
  const auto& translation = transform.transform().translation();
  const auto& rotation = transform.transform().rotation();
  return Eigen::Translation3d(translation.x(), translation.y(),
                              translation.z()) *
         Eigen::Quaterniond(rotation.qw(), rotation.qx(), rotation.qy(),
                            rotation.qz());
}

void TimedTransformResolver::ToStamped(double timestamp_sec,
                                       const Eigen::Affine3d& pose,
                                       StampedTransform* transform) {
  CHECK_NOTNULL(transform);
  transform->timestamp = timestamp_sec;
  transform->translation = Eigen::Translation3d(
      pose.translation().x(), pose.translation().y(), pose.translation().z());
  transform->rotation = Eigen::Quaterniond(pose.linear());
}

}  // namespace transform
}  // namespace apollo
