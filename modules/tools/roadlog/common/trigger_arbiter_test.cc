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

#include "gtest/gtest.h"

namespace apollo {
namespace data {
namespace {

TriggerEvent MakeIncidentTrigger(const std::string& name, const uint64_t time_ns,
                                 const uint64_t backward_ns,
                                 const uint64_t forward_ns,
                                 const uint64_t cooldown_ns) {
  TriggerEvent event;
  event.trigger_name = name;
  event.description = name;
  event.group = TriggerGroup::kIncident;
  event.trigger_time = time_ns;
  event.backward_time = backward_ns;
  event.forward_time = forward_ns;
  event.cooldown_time = cooldown_ns;
  event.begin_time = time_ns - backward_ns;
  event.end_time = time_ns + forward_ns;
  return event;
}

TEST(TriggerArbiterTest, MergesOverlappingIncidentTriggers) {
  TriggerArbiter arbiter;
  const auto first =
      arbiter.RegisterTrigger(MakeIncidentTrigger("DriveEventTrigger", 100, 10, 20, 5));
  const auto second =
      arbiter.RegisterTrigger(MakeIncidentTrigger("EmergencyModeTrigger", 115, 5, 30, 8));

  EXPECT_EQ(first.event_id, second.event_id);
  EXPECT_EQ(90U, second.window_begin_time);
  EXPECT_EQ(145U, second.window_end_time);
  EXPECT_EQ(153U, second.export_ready_time);
  EXPECT_EQ(2U, second.total_trigger_count);
  EXPECT_EQ(0U, second.suppressed_duplicate_count);
  EXPECT_EQ(2U, second.trigger_summaries.size());
}

TEST(TriggerArbiterTest, ReopensIncidentWithinCooldownWindow) {
  TriggerArbiter arbiter;
  const auto first =
      arbiter.RegisterTrigger(MakeIncidentTrigger("DriveEventTrigger", 100, 10, 20, 20));
  const auto second =
      arbiter.RegisterTrigger(MakeIncidentTrigger("DriveEventTrigger", 135, 5, 10, 20));

  EXPECT_EQ(first.event_id, second.event_id);
  EXPECT_EQ(2U, second.total_trigger_count);
  EXPECT_EQ(1U, second.suppressed_duplicate_count);
  EXPECT_EQ(165U, second.export_ready_time);
}

TEST(TriggerArbiterTest, SplitsIncidentAfterCooldownWindow) {
  TriggerArbiter arbiter;
  const auto first =
      arbiter.RegisterTrigger(MakeIncidentTrigger("DriveEventTrigger", 100, 10, 10, 5));
  const auto second =
      arbiter.RegisterTrigger(MakeIncidentTrigger("DriveEventTrigger", 130, 10, 10, 5));

  EXPECT_NE(first.event_id, second.event_id);
}

TEST(TriggerArbiterTest, KeepsPeriodicSnapshotsIndependent) {
  TriggerArbiter arbiter;
  TriggerEvent periodic = MakeIncidentTrigger("RegularIntervalTrigger", 100, 0, 0, 0);
  periodic.group = TriggerGroup::kPeriodicSnapshot;

  const auto first = arbiter.RegisterTrigger(periodic);
  periodic.trigger_time = 101;
  periodic.begin_time = 101;
  periodic.end_time = 101;
  const auto second = arbiter.RegisterTrigger(periodic);

  EXPECT_NE(first.event_id, second.event_id);
  EXPECT_EQ(1U, first.total_trigger_count);
  EXPECT_EQ(1U, second.total_trigger_count);
}

}  // namespace
}  // namespace data
}  // namespace apollo
