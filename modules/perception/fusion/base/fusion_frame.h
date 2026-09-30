/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include <vector>

#include "modules/perception/base/frame.h"
#include "modules/perception/base/object.h"
#include "modules/perception/fusion/base/scene.h"
#include "modules/perception/fusion/base/sensor_frame.h"

namespace apollo {
namespace perception {
namespace fusion {

struct FusionFrame {
  base::FramePtr frame;

  // Admission snapshots observations only. A tick drains the bounded queue,
  // predicts the scene, then collects output at frame->timestamp.
  bool publish_tick = false;
  bool admitted = false;
  // Visualization uses the last processed main-sensor pose, not an admitted
  // future pose or an invented identity pose.
  bool has_publish_pose = false;
  bool ready = false;
  bool degraded = false;
  double max_prediction_age = 0.5;

  std::vector<SensorFramePtr> sensor_frames;

  std::vector<base::ObjectPtr> fused_objects;

  ScenePtr scene_ptr;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
