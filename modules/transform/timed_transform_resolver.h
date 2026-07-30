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

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "Eigen/Geometry"

#include "modules/transform/transform_query.h"

namespace apollo {
namespace transform {

struct StampedTransform {
  double timestamp = 0.0;
  Eigen::Translation3d translation;
  Eigen::Quaterniond rotation;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum class TransformResolveStatus {
  kOk = 0,
  kEmpty = 1,
  kTooOld = 2,
  kTooEarly = 3,
  kQueryFailed = 4,
};

struct TimedTransformResolverOptions {
  float query_timeout_sec = 0.01f;
  double cache_duration_sec = 1.0;
  double max_extrapolation_sec = 0.15;
  double latest_lookup_fallback_tolerance_sec = 0.015;
  bool enable_extrapolation = true;
  bool hardware_trigger = true;
};

class BufferInterface;

class TimedTransformResolver {
 public:
  TimedTransformResolver() = default;
  explicit TimedTransformResolver(TransformQuery* query)
      : transform_query_(query) {}
  TimedTransformResolver(BufferInterface* buffer,
                         const std::string& target_frame,
                         const std::string& source_frame,
                         const TimedTransformResolverOptions& options = {});
  ~TimedTransformResolver() = default;

  void SetTransformQuery(TransformQuery* query);
  void Configure(BufferInterface* buffer, const std::string& target_frame,
                 const std::string& source_frame,
                 const TimedTransformResolverOptions& options);
  void ConfigureFrames(const std::string& target_frame,
                       const std::string& source_frame);
  void SetOptions(const TimedTransformResolverOptions& options);

  bool Prefetch(double timestamp_sec);

  bool PrefetchBatch(const std::vector<double>& timestamps_sec);

  bool QueryCached(double timestamp_sec, Eigen::Affine3d* pose,
                   TransformResolveStatus* status = nullptr) const;

  bool QueryCachedStrict(double timestamp_sec, Eigen::Affine3d* pose,
                         TransformResolveStatus* status = nullptr) const;

  bool QueryCachedBatch(const std::vector<double>& timestamps_sec,
                        std::vector<Eigen::Affine3d>* poses,
                        TransformResolveStatus* status = nullptr) const;

  bool QueryCachedBatchStrict(const std::vector<double>& timestamps_sec,
                              std::vector<Eigen::Affine3d>* poses,
                              TransformResolveStatus* status = nullptr) const;

  bool Resolve(double timestamp_sec, StampedTransform* transform,
               TransformResolveStatus* status = nullptr);

  bool Resolve(double timestamp_sec, const std::string& target_frame,
               const std::string& source_frame, StampedTransform* transform,
               TransformResolveStatus* status = nullptr);

  bool ResolveToAffine(double timestamp_sec, Eigen::Affine3d* pose,
                       TransformResolveStatus* status = nullptr);

 private:
  struct PoseSample {
    double timestamp_sec = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
  };

  static PoseSample FromAffine(double timestamp_sec,
                               const Eigen::Affine3d& pose);
  static Eigen::Affine3d ToAffine(const PoseSample& sample);
  static Eigen::Affine3d ToAffine(const TransformStamped& transform);
  static void ToStamped(double timestamp_sec, const Eigen::Affine3d& pose,
                        StampedTransform* transform);

  bool InsertSample(double timestamp_sec, const Eigen::Affine3d& pose);
  bool QueryCachedInternal(double timestamp_sec, double max_extrapolation_sec,
                           Eigen::Affine3d* pose,
                           TransformResolveStatus* status) const;
  bool QueryTransform(double timestamp_sec, TransformStamped* transform) const;
  bool QueryTransform(double timestamp_sec, Eigen::Affine3d* pose,
                      double* resolved_timestamp_sec = nullptr) const;
  void ClearCache();

  std::unique_ptr<TransformQuery> owned_query_;
  TransformQuery* transform_query_ = nullptr;
  std::string target_frame_;
  std::string source_frame_;
  TimedTransformResolverOptions options_;
  mutable std::mutex mutex_;
  std::vector<PoseSample> samples_;
};

}  // namespace transform
}  // namespace apollo
