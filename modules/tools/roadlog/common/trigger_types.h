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

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace apollo {
namespace data {

enum class TriggerGroup {
  kIncident = 0,
  kPeriodicSnapshot = 1,
};

struct TriggerEvent {
  std::string trigger_name;
  std::string description;
  TriggerGroup group = TriggerGroup::kIncident;
  uint64_t trigger_time = 0;
  uint64_t backward_time = 0;
  uint64_t forward_time = 0;
  uint64_t cooldown_time = 0;
  uint64_t begin_time = 0;
  uint64_t end_time = 0;
};

struct TriggerSummary {
  std::string trigger_name;
  std::string description;
  TriggerGroup group = TriggerGroup::kIncident;
  uint64_t first_trigger_time = 0;
  uint64_t last_trigger_time = 0;
  uint64_t backward_time = 0;
  uint64_t forward_time = 0;
  uint64_t cooldown_time = 0;
  size_t count = 0;
};

struct ArbitratedEvent {
  uint64_t event_id = 0;
  TriggerGroup group = TriggerGroup::kIncident;
  uint64_t first_trigger_time = 0;
  uint64_t last_trigger_time = 0;
  uint64_t window_begin_time = 0;
  uint64_t window_end_time = 0;
  uint64_t export_ready_time = 0;
  size_t total_trigger_count = 0;
  size_t suppressed_duplicate_count = 0;
  bool exported = false;
  std::map<std::string, TriggerSummary> trigger_summaries;
};

}  // namespace data
}  // namespace apollo
