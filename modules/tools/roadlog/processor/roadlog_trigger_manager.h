/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/tools/roadlog/proto/smart_recorder_triggers.pb.h"

#include "cyber/cyber.h"
#include "modules/tools/roadlog/triggers/trigger_base.h"

namespace apollo {
namespace data {

class RoadlogTriggerManager {
 public:
  bool Init(const SmartRecordTrigger& trigger_conf,
            const std::shared_ptr<apollo::cyber::Node>& node);
  void TickPassiveTriggers();
  std::deque<TriggerEvent> DrainPendingEvents();
  std::set<std::string> GetRequiredChannels() const;

 private:
  bool InitTriggers(const SmartRecordTrigger& trigger_conf);
  bool InitTriggerReaders();
  void HandleTriggerMessage(
      const std::string& channel_name,
      const std::shared_ptr<apollo::cyber::message::RawMessage>& message);
  void HandleTriggerEvent(const TriggerEvent& trigger_event);

  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::vector<std::unique_ptr<TriggerBase>> triggers_;
  std::vector<std::shared_ptr<apollo::cyber::ReaderBase>> trigger_readers_;
  std::unordered_map<std::string, std::vector<TriggerBase*>> channel_triggers_;
  std::vector<TriggerBase*> passive_triggers_;
  std::mutex trigger_mutex_;
  std::mutex event_mutex_;
  std::deque<TriggerEvent> pending_trigger_events_;
};

}  // namespace data
}  // namespace apollo
