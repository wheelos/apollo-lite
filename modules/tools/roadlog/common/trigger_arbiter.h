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

#include <cstdint>
#include <vector>

#include "modules/tools/roadlog/common/trigger_types.h"

namespace apollo {
namespace data {

class TriggerArbiter {
 public:
  TriggerArbiter() = default;

  ArbitratedEvent RegisterTrigger(const TriggerEvent& trigger_event);
  std::vector<ArbitratedEvent> GetExportCandidates(uint64_t current_time,
                                                   bool include_all) const;
  void MarkExported(uint64_t event_id);
  void RemoveEvent(uint64_t event_id);

 private:
  ArbitratedEvent* FindMergeCandidate(const TriggerEvent& trigger_event);
  ArbitratedEvent CreateEvent(const TriggerEvent& trigger_event);
  void MergeTrigger(const TriggerEvent& trigger_event, ArbitratedEvent* event);
  void UpdateTriggerSummary(const TriggerEvent& trigger_event,
                            ArbitratedEvent* event);

  uint64_t next_event_id_ = 1;
  std::vector<ArbitratedEvent> events_;
};

}  // namespace data
}  // namespace apollo
