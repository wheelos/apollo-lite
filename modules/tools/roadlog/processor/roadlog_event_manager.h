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
#include <mutex>
#include <string>
#include <unordered_map>

#include "modules/tools/roadlog/common/trigger_arbiter.h"
#include "modules/tools/roadlog/processor/roadlog_runtime_types.h"

namespace apollo {
namespace data {

class RoadlogSegmentManager;

class RoadlogEventManager {
 public:
  void RegisterTriggers(std::deque<TriggerEvent> pending_events,
                        RoadlogSegmentManager* segment_manager);
  void RefreshPinnedSegments(RoadlogSegmentManager* segment_manager);
  bool PrepareExportPlan(uint64_t current_time, bool shutting_down,
                         const std::string& events_dir,
                         const RoadlogSegmentManager& segment_manager,
                         RoadlogEventExportPlan* plan);
  void FinalizeExport(const RoadlogEventExportPlan& plan, bool exported,
                      RoadlogSegmentManager* segment_manager);

 private:
  bool EventMatchesPlan(const ArbitratedEvent& event,
                        const RoadlogEventExportPlan& plan) const;
  void UpdateEventStateLocked(const ArbitratedEvent& event,
                              RoadlogSegmentManager* segment_manager);
  std::string GetEventOutputDir(const std::string& events_dir,
                                const ArbitratedEvent& event) const;

  TriggerArbiter trigger_arbiter_;
  std::mutex mutex_;
  std::unordered_map<uint64_t, RoadlogEventState> events_;
};

}  // namespace data
}  // namespace apollo
