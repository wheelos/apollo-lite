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

#include "modules/tools/roadlog/triggers/trigger_base.h"

#include "cyber/common/log.h"

namespace apollo {
namespace data {

bool TriggerBase::Init(const SmartRecordTrigger& trigger_conf,
                       TriggerEventHandler event_handler) {
  LockTrigger(trigger_conf);
  event_handler_ = std::move(event_handler);
  return true;
}

uint64_t TriggerBase::SecondsToNanoSeconds(const double seconds) const {
  static constexpr uint64_t kSecondsToNanoSecondsFactor = 1000000000UL;
  return static_cast<uint64_t>(kSecondsToNanoSecondsFactor * seconds);
}

void TriggerBase::LockTrigger(const SmartRecordTrigger& trigger_conf) {
  for (const auto& trigger : trigger_conf.triggers()) {
    if (trigger.trigger_name() == trigger_name_) {
      trigger_obj_.reset(new Trigger(trigger));
      break;
    }
  }
  if (trigger_obj_ == nullptr) {
    Trigger disabled_trigger;
    disabled_trigger.set_trigger_name(trigger_name_);
    disabled_trigger.set_enabled(false);
    trigger_obj_ = std::make_unique<Trigger>(disabled_trigger);
  }
}

uint64_t TriggerBase::GetValidValueInRange(const double desired_value,
                                           const double min_limit,
                                           const double max_limit) const {
  return SecondsToNanoSeconds(desired_value < min_limit   ? min_limit
                              : desired_value > max_limit ? max_limit
                                                          : desired_value);
}

void TriggerBase::TriggerIt(const uint64_t msg_time) const {
  static constexpr float kMaxBackwardTime = 80.0;
  static constexpr float kMaxForwardTime = 80.0;
  static constexpr float kMaxCooldownTime = 300.0;
  static constexpr uint64_t kZero = 0.0;
  const uint64_t backward_time = GetValidValueInRange(
      trigger_obj_->backward_time(), kZero, kMaxBackwardTime);
  const uint64_t forward_time = GetValidValueInRange(
      trigger_obj_->forward_time(), kZero, kMaxForwardTime);
  const uint64_t cooldown_time = GetValidValueInRange(
      trigger_obj_->cooldown_time(), kZero, kMaxCooldownTime);
  if (event_handler_ == nullptr) {
    AERROR << "missing trigger event handler for " << trigger_name_;
    return;
  }
  TriggerEvent trigger_event;
  trigger_event.trigger_name = trigger_obj_->trigger_name();
  trigger_event.description = trigger_obj_->description();
  trigger_event.group = IsPeriodicTrigger() ? TriggerGroup::kPeriodicSnapshot
                                            : TriggerGroup::kIncident;
  trigger_event.trigger_time = msg_time;
  trigger_event.backward_time = backward_time;
  trigger_event.forward_time = forward_time;
  trigger_event.cooldown_time = cooldown_time;
  trigger_event.begin_time = msg_time - backward_time;
  trigger_event.end_time = msg_time + forward_time;
  event_handler_(trigger_event);
}

}  // namespace data
}  // namespace apollo
