// Copyright 2026 WheelOS. All Rights Reserved.
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

#include "modules/lidar_object_detection/types/model_input.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace apollo {
namespace lidar_object_detection {

namespace {

constexpr std::size_t kCenterPointFeatureCount = 5U;
constexpr double kNanosecondsToSeconds = 1.0e-9;

void SetError(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
}

bool IsPointFeatureSource(PointFeatureSource source) {
  return source == POINT_FEATURE_X || source == POINT_FEATURE_Y ||
         source == POINT_FEATURE_Z || source == POINT_FEATURE_INTENSITY ||
         source == POINT_FEATURE_TIME_OFFSET_SECONDS;
}

float ReadFeature(const apollo::drivers::PointXYZIT& point,
                  PointFeatureSource source, double measurement_time) {
  switch (source) {
    case POINT_FEATURE_X:
      return point.x();
    case POINT_FEATURE_Y:
      return point.y();
    case POINT_FEATURE_Z:
      return point.z();
    case POINT_FEATURE_INTENSITY:
      return static_cast<float>(point.intensity());
    case POINT_FEATURE_TIME_OFFSET_SECONDS:
      if (point.timestamp() == 0U) {
        return 0.0F;
      }
      return static_cast<float>(static_cast<double>(point.timestamp()) *
                                    kNanosecondsToSeconds -
                                measurement_time);
    default:
      return std::numeric_limits<float>::quiet_NaN();
  }
}

}  // namespace

bool ModelInputBuilder::Init(const CenterPointModelConfig& config,
                             std::string* error) {
  if (config.input_feature_size() !=
      static_cast<int>(kCenterPointFeatureCount)) {
    SetError(
        "CenterPoint input requires exactly five configured "
        "point features",
        error);
    return false;
  }
  if (config.input_feature(0).source() != POINT_FEATURE_X ||
      config.input_feature(1).source() != POINT_FEATURE_Y ||
      config.input_feature(2).source() != POINT_FEATURE_Z) {
    SetError(
        "The first three CenterPoint input features must be "
        "x, y, and z",
        error);
    return false;
  }
  for (const auto& feature : config.input_feature()) {
    if (!IsPointFeatureSource(feature.source()) ||
        !std::isfinite(feature.scale())) {
      SetError("CenterPoint input feature source or scale is invalid", error);
      return false;
    }
  }
  features_.assign(config.input_feature().begin(),
                   config.input_feature().end());
  initialized_ = true;
  return true;
}

bool ModelInputBuilder::Build(const apollo::drivers::PointCloud& cloud,
                              ModelInput* input, std::string* error) const {
  if (!initialized_) {
    SetError("CenterPoint model input builder is not initialized", error);
    return false;
  }
  if (input == nullptr) {
    SetError("CenterPoint model input output is null", error);
    return false;
  }

  ModelInput result;
  result.points.reserve(static_cast<std::size_t>(cloud.point_size()) *
                        kCenterPointFeatureCount);
  result.source_point_indices.reserve(
      static_cast<std::size_t>(cloud.point_size()));
  const double measurement_time = cloud.measurement_time();
  if (!std::isfinite(measurement_time)) {
    SetError("PointCloud measurement_time is not finite", error);
    return false;
  }

  for (int point_index = 0; point_index < cloud.point_size(); ++point_index) {
    const auto& point = cloud.point(point_index);
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      continue;
    }
    float point_features[kCenterPointFeatureCount];
    bool valid_point = true;
    for (std::size_t feature_index = 0;
         feature_index < kCenterPointFeatureCount; ++feature_index) {
      const auto& feature = features_[feature_index];
      const float value =
          ReadFeature(point, feature.source(), measurement_time) *
          feature.scale();
      if (!std::isfinite(value)) {
        valid_point = false;
        break;
      }
      point_features[feature_index] = value;
    }
    if (!valid_point) {
      continue;
    }
    result.points.insert(result.points.end(), point_features,
                         point_features + kCenterPointFeatureCount);
    result.source_point_indices.push_back(static_cast<uint32_t>(point_index));
  }

  *input = std::move(result);
  return true;
}

}  // namespace lidar_object_detection
}  // namespace apollo
