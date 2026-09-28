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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/prediction/evaluator/evaluator.h"
#include "modules/prediction/evaluator/vehicle/hivt_scene_feature_builder.h"
#include "modules/prediction/evaluator/vehicle/hivt_tensorrt_executor.h"

namespace apollo {
namespace prediction {

class HiVTSceneEvaluator final : public Evaluator {
 public:
  HiVTSceneEvaluator();
  ~HiVTSceneEvaluator() override = default;

  bool Evaluate(Obstacle* obstacle,
                ObstaclesContainer* obstacles_container) override;
  bool EvaluateScene(const std::vector<Obstacle*>& targets,
                     ObstaclesContainer* obstacles_container);
  std::string GetName() override { return "HIVT_SCENE_EVALUATOR"; }

 private:
  HiVTSceneFeatureBuilder feature_builder_;
  HiVTTensorRtExecutor executor_;
  bool executor_ready_ = false;
};

}  // namespace prediction
}  // namespace apollo
