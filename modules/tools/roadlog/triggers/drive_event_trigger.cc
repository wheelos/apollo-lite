/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/tools/roadlog/triggers/drive_event_trigger.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"

namespace apollo {
namespace data {

DriveEventTrigger::DriveEventTrigger() { trigger_name_ = "DriveEventTrigger"; }

std::set<std::string> DriveEventTrigger::GetObservedChannels() const {
  return {FLAGS_drive_event_topic};
}

void DriveEventTrigger::Pull(const cyber::record::RecordMessage& msg) {
  if (!trigger_obj_->enabled()) {
    return;
  }
  // Simply check the channel
  if (msg.channel_name == FLAGS_drive_event_topic) {
    AINFO << "drive event trigger is pulled: " << msg.time << " - "
          << msg.channel_name;
    TriggerIt(msg.time);
  }
}

}  // namespace data
}  // namespace apollo
