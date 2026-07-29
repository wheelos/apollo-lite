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

#include "modules/tools/roadlog/processor/roadlog_event_manager.h"

#include <utility>

#include "absl/strings/str_cat.h"

#include "modules/tools/roadlog/processor/roadlog_segment_manager.h"

namespace apollo {
namespace data {

void RoadlogEventManager::RegisterTriggers(
    std::deque<TriggerEvent> pending_events,
    RoadlogSegmentManager* segment_manager) {
  if (segment_manager == nullptr || pending_events.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  while (!pending_events.empty()) {
    const auto event = trigger_arbiter_.RegisterTrigger(pending_events.front());
    UpdateEventStateLocked(event, segment_manager);
    pending_events.pop_front();
  }
}

void RoadlogEventManager::RefreshPinnedSegments(
    RoadlogSegmentManager* segment_manager) {
  if (segment_manager == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& event_and_state : events_) {
    auto& event_state = event_and_state.second;
    if (event_state.metadata.exported || event_state.exporting) {
      continue;
    }
    segment_manager->PinWindow(event_state.metadata.window_begin_time,
                               event_state.metadata.window_end_time,
                               &event_state.segment_paths);
  }
}

bool RoadlogEventManager::PrepareExportPlan(
    const uint64_t current_time, const bool shutting_down,
    const std::string& events_dir, const RoadlogSegmentManager& segment_manager,
    RoadlogEventExportPlan* plan) {
  if (plan == nullptr) {
    return false;
  }
  const auto candidates =
      trigger_arbiter_.GetExportCandidates(current_time, shutting_down);
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& candidate : candidates) {
    auto event_iter = events_.find(candidate.event_id);
    if (event_iter == events_.end()) {
      continue;
    }
    auto& event_state = event_iter->second;
    event_state.metadata = candidate;
    if (event_state.exporting || event_state.metadata.exported) {
      continue;
    }
    if (event_state.segment_paths.empty() && !shutting_down) {
      continue;
    }
    if (!shutting_down &&
        !segment_manager.IsWindowCovered(
            event_state.segment_paths, event_state.metadata.window_end_time)) {
      continue;
    }
    event_state.exporting = true;
    plan->metadata = event_state.metadata;
    plan->output_dir = GetEventOutputDir(events_dir, plan->metadata);
    plan->segments = segment_manager.CollectSegments(
        plan->metadata.window_begin_time, plan->metadata.window_end_time,
        event_state.segment_paths, &plan->partial);
    return true;
  }
  return false;
}

void RoadlogEventManager::FinalizeExport(
    const RoadlogEventExportPlan& plan, const bool exported,
    RoadlogSegmentManager* segment_manager) {
  if (segment_manager == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  auto event_iter = events_.find(plan.metadata.event_id);
  if (event_iter == events_.end()) {
    return;
  }
  auto& event_state = event_iter->second;
  event_state.exporting = false;
  if (!exported) {
    return;
  }
  if (!EventMatchesPlan(event_state.metadata, plan)) {
    return;
  }
  event_state.metadata.exported = true;
  trigger_arbiter_.MarkExported(plan.metadata.event_id);
  segment_manager->Release(event_state.segment_paths);
  trigger_arbiter_.RemoveEvent(plan.metadata.event_id);
  events_.erase(event_iter);
}

bool RoadlogEventManager::EventMatchesPlan(
    const ArbitratedEvent& event, const RoadlogEventExportPlan& plan) const {
  const auto& planned = plan.metadata;
  if (event.event_id != planned.event_id || event.group != planned.group ||
      event.first_trigger_time != planned.first_trigger_time ||
      event.last_trigger_time != planned.last_trigger_time ||
      event.window_begin_time != planned.window_begin_time ||
      event.window_end_time != planned.window_end_time ||
      event.export_ready_time != planned.export_ready_time ||
      event.total_trigger_count != planned.total_trigger_count ||
      event.suppressed_duplicate_count != planned.suppressed_duplicate_count ||
      event.trigger_summaries.size() != planned.trigger_summaries.size()) {
    return false;
  }
  for (const auto& trigger_name_and_summary : planned.trigger_summaries) {
    const auto event_summary_iter =
        event.trigger_summaries.find(trigger_name_and_summary.first);
    if (event_summary_iter == event.trigger_summaries.end()) {
      return false;
    }
    const auto& lhs = event_summary_iter->second;
    const auto& rhs = trigger_name_and_summary.second;
    if (lhs.trigger_name != rhs.trigger_name ||
        lhs.description != rhs.description || lhs.group != rhs.group ||
        lhs.first_trigger_time != rhs.first_trigger_time ||
        lhs.last_trigger_time != rhs.last_trigger_time ||
        lhs.backward_time != rhs.backward_time ||
        lhs.forward_time != rhs.forward_time ||
        lhs.cooldown_time != rhs.cooldown_time || lhs.count != rhs.count) {
      return false;
    }
  }
  return true;
}

void RoadlogEventManager::UpdateEventStateLocked(
    const ArbitratedEvent& event, RoadlogSegmentManager* segment_manager) {
  auto& event_state = events_[event.event_id];
  event_state.metadata = event;
  segment_manager->PinWindow(event_state.metadata.window_begin_time,
                             event_state.metadata.window_end_time,
                             &event_state.segment_paths);
}

std::string RoadlogEventManager::GetEventOutputDir(
    const std::string& events_dir, const ArbitratedEvent& event) const {
  return absl::StrCat(events_dir, "/event_", event.event_id, "_",
                      event.window_begin_time);
}

}  // namespace data
}  // namespace apollo
