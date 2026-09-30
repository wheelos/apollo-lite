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

#include "modules/perception/common/graph/gated_hungarian_bigraph_matcher.h"
#include "modules/perception/fusion/lib/interface/base_data_association.h"

namespace apollo {
namespace perception {
namespace fusion {

struct AssociationParams {
  double max_center_distance = 4.0;
  double max_mahalanobis_distance = 9.21;
  double covariance_floor = 0.001;
  double class_penalty = 0.2;
  double min_camera_similarity = 0.5;
  double max_velocity_difference = 10.0;
  double velocity_penalty = 0.1;
  double size_penalty = 0.1;
  double heading_penalty = 0.1;
};

class HMTrackersObjectsAssociation : public BaseDataAssociation {
 public:
  explicit HMTrackersObjectsAssociation(
      const AssociationParams& params = AssociationParams()) : params_(params) {}

  bool Init() override;
  bool Associate(const AssociationOptions& options,
                 SensorFramePtr sensor_measurements, ScenePtr scene,
                 AssociationResult* association_result) override;

 private:
  bool Evaluate(const TrackPtr& track, const SensorObjectPtr& object,
                double* loss, bool* accepted);

  AssociationParams params_;
  common::GatedHungarianMatcher<float> optimizer_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
