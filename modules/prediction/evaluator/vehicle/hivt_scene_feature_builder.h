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

#include <vector>

#include "modules/prediction/container/obstacles/obstacle.h"
#include "modules/prediction/evaluator/vehicle/hivt_scene.h"

namespace apollo {
namespace prediction {

class HiVTSceneFeatureBuilder final {
 public:
  static std::vector<Obstacle*> SelectTargets(
      const Obstacle* ego, const std::vector<Obstacle*>& candidates);

  static std::vector<Obstacle*> SelectActors(
      Obstacle* ego, const std::vector<Obstacle*>& targets,
      const std::vector<Obstacle*>& candidates);

  bool Build(const Obstacle* ego, const std::vector<Obstacle*>& actors,
             HiVTSceneInput* input) const;
};

}  // namespace prediction
}  // namespace apollo
