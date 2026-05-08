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

#include "modules/tools/roadlog/processor/roadlog_trigger_manager.h"

#include "cyber/common/log.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/record_message.h"
#include "cyber/time/time.h"
#include "modules/tools/roadlog/triggers/bumper_crash_trigger.h"
#include "modules/tools/roadlog/triggers/drive_event_trigger.h"
#include "modules/tools/roadlog/triggers/emergency_mode_trigger.h"
#include "modules/tools/roadlog/triggers/hard_brake_trigger.h"
#include "modules/tools/roadlog/triggers/regular_interval_trigger.h"
#include "modules/tools/roadlog/triggers/swerve_trigger.h"

namespace apollo {
namespace data {

using apollo::cyber::Time;
using apollo::cyber::message::RawMessage;
using apollo::cyber::record::RecordMessage;

bool RoadlogTriggerManager::Init(
    const SmartRecordTrigger& trigger_conf,
    const std::shared_ptr<apollo::cyber::Node>& node) {
  node_ = node;
  if (node_ == nullptr) {
    AERROR << "smart recorder node must not be null";
    return false;
  }
  return InitTriggers(trigger_conf) && InitTriggerReaders();
}

bool RoadlogTriggerManager::InitTriggers(
    const SmartRecordTrigger& trigger_conf) {
  TriggerBase::TriggerEventHandler event_handler =
      [this](const TriggerEvent& trigger_event) {
        HandleTriggerEvent(trigger_event);
      };

  triggers_.clear();
  triggers_.push_back(std::make_unique<DriveEventTrigger>());
  triggers_.push_back(std::make_unique<EmergencyModeTrigger>());
  triggers_.push_back(std::make_unique<HardBrakeTrigger>());
  triggers_.push_back(std::make_unique<RegularIntervalTrigger>());
  triggers_.push_back(std::make_unique<SwerveTrigger>());
  triggers_.push_back(std::make_unique<BumperCrashTrigger>());
  for (auto& trigger : triggers_) {
    if (!trigger->Init(trigger_conf, event_handler)) {
      AERROR << "unable to initiate trigger " << trigger->GetTriggerName();
      return false;
    }
  }
  return true;
}

bool RoadlogTriggerManager::InitTriggerReaders() {
  channel_triggers_.clear();
  passive_triggers_.clear();
  trigger_readers_.clear();
  for (const auto& trigger : triggers_) {
    if (!trigger->IsEnabled()) {
      continue;
    }
    const auto channels = trigger->GetObservedChannels();
    if (channels.empty()) {
      passive_triggers_.push_back(trigger.get());
      continue;
    }
    for (const auto& channel : channels) {
      if (!channel.empty()) {
        channel_triggers_[channel].push_back(trigger.get());
      }
    }
  }

  for (const auto& channel_and_triggers : channel_triggers_) {
    apollo::cyber::ReaderConfig config;
    config.channel_name = channel_and_triggers.first;
    config.pending_queue_size = 10;
    auto reader = node_->CreateReader<RawMessage>(
        config, [this, channel_name = channel_and_triggers.first](
                    const std::shared_ptr<RawMessage>& message) {
          HandleTriggerMessage(channel_name, message);
        });
    if (reader == nullptr) {
      AERROR << "failed to create trigger reader for "
             << channel_and_triggers.first;
      return false;
    }
    trigger_readers_.push_back(reader);
  }
  return true;
}

void RoadlogTriggerManager::TickPassiveTriggers() {
  if (passive_triggers_.empty()) {
    return;
  }
  RecordMessage tick;
  tick.time = Time::Now().ToNanosecond();
  std::lock_guard<std::mutex> lock(trigger_mutex_);
  for (auto* trigger : passive_triggers_) {
    trigger->Pull(tick);
  }
}

std::deque<TriggerEvent> RoadlogTriggerManager::DrainPendingEvents() {
  std::deque<TriggerEvent> pending_events;
  std::lock_guard<std::mutex> lock(event_mutex_);
  pending_events.swap(pending_trigger_events_);
  return pending_events;
}

std::set<std::string> RoadlogTriggerManager::GetRequiredChannels() const {
  std::set<std::string> channels;
  for (const auto& trigger : triggers_) {
    if (!trigger->IsEnabled()) {
      continue;
    }
    const auto observed_channels = trigger->GetObservedChannels();
    channels.insert(observed_channels.begin(), observed_channels.end());
  }
  return channels;
}

void RoadlogTriggerManager::HandleTriggerMessage(
    const std::string& channel_name,
    const std::shared_ptr<RawMessage>& message) {
  if (message == nullptr) {
    AERROR << "trigger message is nullptr, channel: " << channel_name;
    return;
  }
  RecordMessage record_message;
  record_message.channel_name = channel_name;
  record_message.content = message->message;
  record_message.time = Time::Now().ToNanosecond();
  std::lock_guard<std::mutex> lock(trigger_mutex_);
  const auto trigger_iter = channel_triggers_.find(channel_name);
  if (trigger_iter == channel_triggers_.end()) {
    return;
  }
  for (auto* trigger : trigger_iter->second) {
    trigger->Pull(record_message);
  }
}

void RoadlogTriggerManager::HandleTriggerEvent(
    const TriggerEvent& trigger_event) {
  std::lock_guard<std::mutex> lock(event_mutex_);
  pending_trigger_events_.push_back(trigger_event);
}

}  // namespace data
}  // namespace apollo
