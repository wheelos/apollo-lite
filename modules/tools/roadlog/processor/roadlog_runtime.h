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

#include <atomic>
#include <memory>
#include <string>

#include "wheelos_msgs/monitor_msgs/smart_recorder_status.pb.h"
#include "modules/tools/roadlog/proto/smart_recorder_triggers.pb.h"

#include "cyber/cyber.h"
#include "modules/tools/roadlog/common/roadlog_layout.h"
#include "modules/tools/roadlog/processor/roadlog_event_exporter.h"
#include "modules/tools/roadlog/processor/roadlog_event_manager.h"
#include "modules/tools/roadlog/processor/roadlog_recording_service.h"
#include "modules/tools/roadlog/processor/roadlog_segment_manager.h"
#include "modules/tools/roadlog/processor/roadlog_trigger_manager.h"

namespace apollo {
namespace data {

class RoadlogRuntime {
 public:
  explicit RoadlogRuntime(const std::string& roadlog_root_dir);

  bool Init(const SmartRecordTrigger& trigger_conf);
  bool Run();

 private:
  bool PrepareRuntimeDirectories() const;
  void MonitorStatus();
  void SyncState(bool shutting_down);
  void PublishStatus(RecordingState state, const std::string& message) const;

  RoadlogLayout layout_;
  RoadlogRecordingService recording_service_;
  RoadlogTriggerManager trigger_manager_;
  RoadlogSegmentManager segment_manager_;
  RoadlogEventManager event_manager_;
  RoadlogEventExporter event_exporter_;
  std::shared_ptr<cyber::Node> smart_recorder_node_ = nullptr;
  std::shared_ptr<cyber::Writer<SmartRecorderStatus>> recorder_status_writer_ =
      nullptr;
  std::shared_ptr<std::thread> monitor_thread_ = nullptr;
  std::atomic<bool> is_terminating_{false};
};

}  // namespace data
}  // namespace apollo
