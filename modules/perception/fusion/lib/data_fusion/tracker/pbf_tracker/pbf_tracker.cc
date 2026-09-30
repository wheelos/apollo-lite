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
#include "modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker/pbf_tracker.h"

#include <cmath>

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

// TODO(all) fix the static string lint issue
std::string PbfTracker::s_type_fusion_method_ = "DstTypeFusion";  // NOLINT
std::string PbfTracker::s_existence_fusion_method_ =              // NOLINT
    "DstExistenceFusion";
std::string PbfTracker::s_motion_fusion_method_ =  // NOLINT
    "KalmanMotionFusion";
std::string PbfTracker::s_shape_fusion_method_ = "PbfShapeFusion";  // NOLINT
double PbfTracker::s_jerk_variance_ = 4.0;
double PbfTracker::s_covariance_floor_ = 0.001;
bool PbfTracker::s_use_covariance_intersection_ = true;
double PbfTracker::s_covariance_intersection_prior_weight_ = 0.5;

bool PbfTracker::InitParams() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("PbfTracker", &options)) {
    return false;
  }

  std::string woork_root_config = GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  AINFO << "Config file : " << config;
  PbfTrackerConfig params;
  if (!cyber::common::GetProtoFromFile(config, &params)) {
    AERROR << "Read config failed: " << config;
    return false;
  }

  AINFO << "Load PbfTrackerConfig: " << params.type_fusion_method() << ","
        << params.motion_fusion_method() << "," << params.shape_fusion_method()
        << "," << params.existence_fusion_method();
  s_type_fusion_method_ = params.type_fusion_method();
  s_motion_fusion_method_ = params.motion_fusion_method();
  s_existence_fusion_method_ = params.existence_fusion_method();
  s_shape_fusion_method_ = params.shape_fusion_method();
  s_jerk_variance_ = params.jerk_variance();
  s_covariance_floor_ = params.covariance_floor();
  s_use_covariance_intersection_ = params.use_covariance_intersection();
  s_covariance_intersection_prior_weight_ =
      params.covariance_intersection_prior_weight();
  if (!std::isfinite(s_jerk_variance_) || s_jerk_variance_ <= 0 ||
      !std::isfinite(s_covariance_floor_) || s_covariance_floor_ <= 0 ||
      !std::isfinite(s_covariance_intersection_prior_weight_) ||
      s_covariance_intersection_prior_weight_ <= 0 ||
      s_covariance_intersection_prior_weight_ >= 1) {
    AERROR << "Invalid fusion motion estimator parameters.";
    return false;
  }

  return true;
}

bool PbfTracker::InitMethods() {
  if (s_type_fusion_method_ == "DstTypeFusion") {
    type_fusion_.reset(new DstTypeFusion(track_));
  } else {
    AERROR << "Unknown type fusion : " << s_type_fusion_method_;
    return false;
  }

  if (s_motion_fusion_method_ == "KalmanMotionFusion") {
    motion_fusion_.reset(
        new KalmanMotionFusion(track_, s_jerk_variance_, s_covariance_floor_,
                               s_use_covariance_intersection_,
                               s_covariance_intersection_prior_weight_));
  } else {
    AERROR << "Unknown motion fusion : " << s_motion_fusion_method_;
    return false;
  }

  if (s_existence_fusion_method_ == "DstExistenceFusion") {
    existence_fusion_.reset(new DstExistenceFusion(track_));
  } else {
    AERROR << "Unknown existence fusion : " << s_existence_fusion_method_;
    return false;
  }

  if (s_shape_fusion_method_ == "PbfShapeFusion") {
    shape_fusion_.reset(new PbfShapeFusion(track_));
  } else {
    AERROR << "Unknown shape fusion : " << s_shape_fusion_method_;
    return false;
  }

  return true;
}

bool PbfTracker::Init(TrackPtr track, SensorObjectPtr measurement) {
  if (!track || !measurement) {
    AERROR << "Tracker initialization requires a track and measurement.";
    return false;
  }
  track_ = track;
  if (!InitMethods()) {
    return false;
  }
  if (!motion_fusion_->Init()) return false;
  return existence_fusion_->UpdateWithMeasurement(
      measurement, measurement->GetTimestamp(), 0.0);
}

bool PbfTracker::UpdateWithMeasurement(const TrackerOptions& options,
                                       const SensorObjectPtr measurement,
                                       double target_timestamp) {
  if (!std::isfinite(options.match_distance) || options.match_distance < 0 ||
      options.match_distance > 1 || !measurement || !track_ ||
      target_timestamp != measurement->GetTimestamp()) {
    AERROR << "Invalid fusion update or association loss.";
    return false;
  }
  const bool camera_2d = IsCamera(measurement) &&
      measurement->GetBaseObject()->camera_supplement.local_center.z() <= 0;
  if (camera_2d) {
    if (!motion_fusion_->PredictTo(target_timestamp)) return false;
  } else if (!motion_fusion_->UpdateWithMeasurement(
                 measurement, target_timestamp)) {
    return false;
  }
  if (!existence_fusion_->UpdateWithMeasurement(
      measurement, target_timestamp, options.match_distance)) return false;
  if (!camera_2d) {
    shape_fusion_->UpdateWithMeasurement(measurement, target_timestamp);
  }
  if (!type_fusion_->UpdateWithMeasurement(measurement, target_timestamp)) {
    return false;
  }
  track_->UpdateWithSensorObject(measurement);
  return true;
}

bool PbfTracker::UpdateWithoutMeasurement(const TrackerOptions& options,
                                          const std::string& sensor_id,
                                          double measurement_timestamp,
                                          double target_timestamp) {
  if (!std::isfinite(options.match_distance) || options.match_distance < 0 ||
      options.match_distance > 1 ||
      !motion_fusion_->UpdateWithoutMeasurement(
          sensor_id, measurement_timestamp, target_timestamp)) {
    AERROR << "Invalid missed fusion update or association similarity.";
    return false;
  }
  if (!existence_fusion_->UpdateWithoutMeasurement(
      sensor_id, measurement_timestamp, target_timestamp,
      options.match_distance)) return false;
  shape_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                          target_timestamp);
  if (!type_fusion_->UpdateWithoutMeasurement(
      sensor_id, measurement_timestamp, target_timestamp,
      options.match_distance)) return false;
  track_->UpdateWithoutSensorObject(sensor_id, measurement_timestamp);
  return true;
}

bool PbfTracker::PredictTo(double timestamp) {
  if (!motion_fusion_->PredictTo(timestamp)) return false;
  track_->ExpireSensorObjects(timestamp);
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
