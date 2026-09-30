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
#include "modules/perception/fusion/lib/gatekeeper/collect_fused_object.h"

#include <cmath>

#include "modules/perception/fusion/base/object_snapshot.h"
#include "modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"

namespace apollo {
namespace perception {
namespace fusion {

bool CollectFusedObject::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) return false;
  const auto it = plugin_config_map_.find(PluginType::PBF_GATEKEEPER);
  if (it == plugin_config_map_.end() || !it->second.enabled()) {
    AERROR << "Fusion collection requires an enabled publication policy.";
    return false;
  }
  gate_keeper_.reset(new PbfGatekeeper());
  return gate_keeper_->Init(it->second);
}

bool CollectFusedObject::Process(DataFrame* data_frame) {
  if (!data_frame || !data_frame->fusion_frame) {
    AERROR << "Missing fusion collection input.";
    return false;
  }
  auto* frame = data_frame->fusion_frame;
  frame->fused_objects.clear();
  if (!frame->ready) return true;
  if (!frame->frame || !frame->scene_ptr ||
      !std::isfinite(frame->max_prediction_age) ||
      frame->max_prediction_age <= 0 || !gate_keeper_) {
    AERROR << "Fusion collection requires a valid completed cycle.";
    return false;
  }
  scenes_ = frame->scene_ptr;
  max_prediction_age_ = frame->max_prediction_age;
  if (!Process(frame->frame->timestamp, &frame->fused_objects)) {
    frame->fused_objects.clear();
    return false;
  }
  return true;
}

bool CollectFusedObject::Process(double timestamp,
                               std::vector<base::ObjectPtr>* output) {
  for (const auto* tracks : {&scenes_->GetForegroundTracks(),
                            &scenes_->GetBackgroundTracks()}) {
    for (const auto& track : *tracks) {
      const double age = timestamp - track->GetLastMotionObservationTimestamp();
      if (age < 0 || age > max_prediction_age_ ||
          !gate_keeper_->AbleToPublish(track)) {
        continue;
      }
      if (!AppendTrackSnapshot(timestamp, track, output)) return false;
    }
  }
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
