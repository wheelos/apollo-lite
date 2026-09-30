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

#pragma once

#include "modules/common/status/status.h"
#include "modules/world_model/local_map/lane_synthesis.h"
#include "modules/world_model/local_map/local_scene.h"

namespace apollo {
namespace world_model {

// Combines independently produced layers into one validated immutable scene.
common::Status AssembleLocalScene(const SceneSource& source,
                                  const DrivableEnvironment& environment,
                                  const LaneLayer* lanes,
                                  const std::string& area_reason,
                                  const NavigationPrior* navigation, double now,
                                  LocalScene* scene);

}  // namespace world_model
}  // namespace apollo
