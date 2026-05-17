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

#include "modules/tools/roadlog/common/trigger_arbiter.h"

#include <algorithm>

namespace apollo {
namespace data {

ArbitratedEvent TriggerArbiter::RegisterTrigger(
    const TriggerEvent& trigger_event) {
  ArbitratedEvent* event = FindMergeCandidate(trigger_event);
  if (event == nullptr) {
    events_.emplace_back(CreateEvent(trigger_event));
    return events_.back();
  }
  MergeTrigger(trigger_event, event);
  return *event;
}

std::vector<ArbitratedEvent> TriggerArbiter::GetExportCandidates(
    const uint64_t current_time, const bool include_all) const {
  std::vector<ArbitratedEvent> candidates;
  for (const auto& event : events_) {
    if (event.exported) {
      continue;
    }
    if (!include_all && current_time < event.export_ready_time) {
      continue;
    }
    candidates.emplace_back(event);
  }
  return candidates;
}

void TriggerArbiter::MarkExported(const uint64_t event_id) {
  for (auto& event : events_) {
    if (event.event_id == event_id) {
      event.exported = true;
      return;
    }
  }
}

void TriggerArbiter::RemoveEvent(const uint64_t event_id) {
  events_.erase(
      std::remove_if(events_.begin(), events_.end(),
                     [event_id](const ArbitratedEvent& event) {
                       return event.event_id == event_id;
                     }),
      events_.end());
}

ArbitratedEvent* TriggerArbiter::FindMergeCandidate(
    const TriggerEvent& trigger_event) {
  if (trigger_event.group == TriggerGroup::kPeriodicSnapshot) {
    return nullptr;
  }
  for (auto& event : events_) {
    if (event.exported || event.group != trigger_event.group) {
      continue;
    }
    const bool overlaps_event_window =
        trigger_event.begin_time <= event.window_end_time &&
        trigger_event.end_time >= event.window_begin_time;
    const bool within_reopen_window =
        trigger_event.trigger_time <= event.export_ready_time &&
        trigger_event.end_time >= event.window_begin_time;
    if (overlaps_event_window || within_reopen_window) {
      return &event;
    }
  }
  return nullptr;
}

ArbitratedEvent TriggerArbiter::CreateEvent(const TriggerEvent& trigger_event) {
  ArbitratedEvent event;
  event.event_id = next_event_id_++;
  event.group = trigger_event.group;
  event.first_trigger_time = trigger_event.trigger_time;
  event.last_trigger_time = trigger_event.trigger_time;
  event.window_begin_time = trigger_event.begin_time;
  event.window_end_time = trigger_event.end_time;
  event.export_ready_time = trigger_event.end_time + trigger_event.cooldown_time;
  event.total_trigger_count = 1;
  UpdateTriggerSummary(trigger_event, &event);
  return event;
}

void TriggerArbiter::MergeTrigger(const TriggerEvent& trigger_event,
                                  ArbitratedEvent* event) {
  event->last_trigger_time =
      std::max(event->last_trigger_time, trigger_event.trigger_time);
  event->window_begin_time =
      std::min(event->window_begin_time, trigger_event.begin_time);
  event->window_end_time =
      std::max(event->window_end_time, trigger_event.end_time);
  event->export_ready_time =
      std::max(event->export_ready_time,
               trigger_event.end_time + trigger_event.cooldown_time);
  event->total_trigger_count += 1;
  if (event->trigger_summaries.find(trigger_event.trigger_name) !=
      event->trigger_summaries.end()) {
    event->suppressed_duplicate_count += 1;
  }
  UpdateTriggerSummary(trigger_event, event);
}

void TriggerArbiter::UpdateTriggerSummary(const TriggerEvent& trigger_event,
                                          ArbitratedEvent* event) {
  auto& summary = event->trigger_summaries[trigger_event.trigger_name];
  if (summary.count == 0) {
    summary.trigger_name = trigger_event.trigger_name;
    summary.description = trigger_event.description;
    summary.group = trigger_event.group;
    summary.first_trigger_time = trigger_event.trigger_time;
    summary.backward_time = trigger_event.backward_time;
    summary.forward_time = trigger_event.forward_time;
    summary.cooldown_time = trigger_event.cooldown_time;
  }
  summary.last_trigger_time = trigger_event.trigger_time;
  summary.count += 1;
}

}  // namespace data
}  // namespace apollo
