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

//  Created Date: 2026-08-28
//  Author: daohu527

#pragma once

#include <memory>
#include <string>

#include "modules/lane/proto/lane_component_config.pb.h"
#include "wheelos_msgs/perception_msgs/perception_lane.pb.h"
#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

#include "cyber/component/component.h"
#include "modules/lane/calibration/static_lane_calibrator.h"
#include "modules/lane/component/lane_transform_adapter.h"
#include "modules/lane/inference/tensorrt_executor.h"
#include "modules/lane/inference/ufldv2.h"

namespace apollo {
namespace lane {

class LaneDetectionComponent final
    : public cyber::Component<apollo::drivers::Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::drivers::Image>& image) override;

 private:
  bool MakeImageView(const apollo::drivers::Image& message,
                     ImageView* image_view) const;
  bool Serialize(const LaneDetectionResult& result,
                 apollo::perception::PerceptionLanes* message) const;

  LaneComponentConfig config_;
  StaticLaneCalibrator calibrator_;
  LaneTransformAdapter transform_adapter_;
  std::unique_ptr<TensorRtUfldv2Executor> executor_;
  std::unique_ptr<Ufldv2Detector> detector_;
  std::shared_ptr<cyber::Writer<apollo::perception::PerceptionLanes>> writer_;
};

CYBER_REGISTER_COMPONENT(LaneDetectionComponent)

}  // namespace lane
}  // namespace apollo
