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

#include "modules/tools/roadlog/processor/roadlog_runtime.h"

#include <unistd.h>

#include <chrono>
#include <thread>

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/init.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/monitor/common/monitor_manager.h"

using Time = ::apollo::cyber::Time;

namespace apollo {
namespace data {

namespace {

using apollo::common::Header;
using apollo::monitor::MonitorManager;
using cyber::CreateNode;
using cyber::common::ClearDirectory;
using cyber::common::EnsureDirectory;

}  // namespace

RoadlogRuntime::RoadlogRuntime(const std::string& roadlog_root_dir)
    : layout_(BuildRoadlogLayout(roadlog_root_dir)),
      recording_service_(layout_),
      segment_manager_(layout_) {}

bool RoadlogRuntime::Init(const SmartRecordTrigger& trigger_conf) {
  if (layout_.root_dir.empty()) {
    AERROR << "roadlog_root_dir is required";
    return false;
  }
  if (!PrepareRuntimeDirectories()) {
    return false;
  }

  cyber::Init("smart_recorder");
  smart_recorder_node_ = CreateNode(absl::StrCat("smart_recorder_", getpid()));
  if (smart_recorder_node_ == nullptr) {
    AERROR << "create smart recorder node failed: " << getpid();
    return false;
  }
  recorder_status_writer_ =
      smart_recorder_node_->CreateWriter<SmartRecorderStatus>(
          FLAGS_recorder_status_topic);

  segment_manager_.ConfigureRetention(
      static_cast<uint32_t>(std::max(1, trigger_conf.ring_segment_count())));
  if (!trigger_manager_.Init(trigger_conf, smart_recorder_node_)) {
    return false;
  }
  if (!recording_service_.Init(trigger_conf,
                               trigger_manager_.GetRequiredChannels())) {
    return false;
  }
  return true;
}

bool RoadlogRuntime::Run() {
  if (!recording_service_.Start()) {
    AERROR << "smart recorder failed to start recorder";
    return false;
  }
  PublishStatus(RecordingState::RECORDING, "smart recorder started");
  MonitorManager::Instance()->LogBuffer().INFO("SmartRecorder is recording...");
  monitor_thread_ =
      std::make_shared<std::thread>([this]() { this->MonitorStatus(); });
  while (!cyber::IsShutdown() && !is_terminating_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  is_terminating_.store(true);
  if (monitor_thread_ != nullptr && monitor_thread_->joinable()) {
    monitor_thread_->join();
    monitor_thread_ = nullptr;
  }
  PublishStatus(RecordingState::STOPPED, "smart recorder stopped");
  MonitorManager::Instance()->LogBuffer().INFO("SmartRecorder is stopped");
  return true;
}

bool RoadlogRuntime::PrepareRuntimeDirectories() const {
  if (!EnsureDirectory(layout_.root_dir) ||
      !EnsureDirectory(layout_.ring_dir) ||
      !EnsureDirectory(layout_.events_dir) ||
      !EnsureDirectory(layout_.meta_dir)) {
    AERROR << "unable to initialize roadlog directories under "
           << layout_.root_dir;
    return false;
  }
  if (!ClearDirectory(layout_.ring_dir) || !ClearDirectory(layout_.meta_dir)) {
    AERROR << "unable to clean roadlog runtime directories under "
           << layout_.root_dir;
    return false;
  }
  return true;
}

void RoadlogRuntime::MonitorStatus() {
  int status_counter = 0;
  int passive_trigger_counter = 0;
  while (!cyber::IsShutdown() && !is_terminating_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (++passive_trigger_counter >= 10) {
      passive_trigger_counter = 0;
      trigger_manager_.TickPassiveTriggers();
    }
    if (++status_counter >= 30) {
      status_counter = 0;
      PublishStatus(RecordingState::RECORDING, "smart recorder recording");
    }
    SyncState(false);
  }
  is_terminating_.store(true);
  recording_service_.Stop();
  SyncState(true);
}

void RoadlogRuntime::SyncState(const bool shutting_down) {
  segment_manager_.RefreshSegments();
  event_manager_.RegisterTriggers(trigger_manager_.DrainPendingEvents(),
                                  &segment_manager_);
  event_manager_.RefreshPinnedSegments(&segment_manager_);
  RoadlogEventExportPlan plan;
  const uint64_t current_time = Time::Now().ToNanosecond();
  while (event_manager_.PrepareExportPlan(current_time, shutting_down,
                                          layout_.events_dir, segment_manager_,
                                          &plan)) {
    const bool exported = event_exporter_.Export(plan);
    event_manager_.RegisterTriggers(trigger_manager_.DrainPendingEvents(),
                                    &segment_manager_);
    event_manager_.FinalizeExport(plan, exported, &segment_manager_);
    if (!exported) {
      break;
    }
    plan = RoadlogEventExportPlan();
  }
  if (segment_manager_.CleanupExpired()) {
    AWARN << "roadlog source ring is full because all old segments are pinned";
  }
}

void RoadlogRuntime::PublishStatus(const RecordingState state,
                                   const std::string& message) const {
  SmartRecorderStatus status;
  Header* status_headerpb = status.mutable_header();
  status_headerpb->set_timestamp_sec(Time::Now().ToSecond());
  status.set_recording_state(state);
  status.set_state_message(message);
  recorder_status_writer_->Write(status);
}

}  // namespace data
}  // namespace apollo
