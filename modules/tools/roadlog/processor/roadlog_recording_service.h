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

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "modules/tools/roadlog/proto/smart_recorder_triggers.pb.h"

#include "cyber/tools/cyber_recorder/channel_rate_filter.h"
#include "cyber/tools/cyber_recorder/message_size_filter.h"
#include "cyber/tools/cyber_recorder/recorder.h"
#include "modules/tools/roadlog/common/roadlog_layout.h"

namespace apollo {
namespace data {

class RoadlogRecordingService {
 public:
  struct RecorderOptions {
    std::string output_path;
    bool all_channels = true;
    std::vector<std::string> include_channels;
    std::vector<std::string> exclude_channels;
    apollo::cyber::proto::Header header;
    apollo::cyber::record::MessageSizeFilterConfig message_size_filter_config;
    apollo::cyber::record::ChannelRateFilterConfig channel_rate_filter_config;
  };

  explicit RoadlogRecordingService(const RoadlogLayout& layout);

  bool Init(const SmartRecordTrigger& trigger_conf,
            const std::set<std::string>& required_channels);
  bool Start();
  void Stop();

  static bool BuildRecorderOptions(const SmartRecordTrigger& trigger_conf,
                                   const std::string& output_path,
                                   RecorderOptions* options,
                                   std::string* error);

 private:
  static bool BuildLargeMessageFilterConfig(
      const RecorderPolicy& recorder_policy,
      apollo::cyber::record::MessageSizeFilterConfig* config,
      std::string* error);
  static bool ValidateChannelPolicies(const RecorderPolicy& recorder_policy,
                                      const RecorderOptions& options,
                                      std::string* error);
  static bool ValidateRequiredChannels(
      const std::set<std::string>& required_channels,
      const RecorderOptions& options, std::string* error);

  RoadlogLayout layout_;
  std::shared_ptr<apollo::cyber::record::Recorder> recorder_ = nullptr;
};

}  // namespace data
}  // namespace apollo
