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
#pragma once

#include <string>

#include "modules/perception/fusion/common/kalman_filter.h"
#include "modules/perception/fusion/lib/interface/base_motion_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

class KalmanMotionFusion : public BaseMotionFusion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit KalmanMotionFusion(TrackPtr track, double jerk_variance = 4.0,
                              double covariance_floor = 0.001,
                              bool use_covariance_intersection = true,
                              double prior_weight = 0.5)
      : BaseMotionFusion(track),
        jerk_variance_(jerk_variance),
        covariance_floor_(covariance_floor),
        use_covariance_intersection_(use_covariance_intersection),
        prior_weight_(prior_weight) {}

  bool Init() override;
  bool PredictTo(double timestamp) override;
  bool UpdateWithMeasurement(const SensorObjectConstPtr& measurement,
                             double target_timestamp) override;
  bool UpdateWithoutMeasurement(const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;
  std::string Name() const override { return "KalmanMotionFusion"; }
  void GetStates(Eigen::Vector3d* center, Eigen::Vector3d* velocity);

 private:
  bool ValidateMeasurement(const SensorObjectConstPtr& measurement) const;
  bool UpdateMotionState();

  KalmanFilter kalman_filter_;
  bool initialized_ = false;
  bool velocity_initialized_ = false;
  double state_timestamp_ = 0.0;
  double jerk_variance_;
  double covariance_floor_;
  bool use_covariance_intersection_;
  double prior_weight_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
