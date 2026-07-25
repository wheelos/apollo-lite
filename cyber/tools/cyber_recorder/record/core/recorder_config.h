// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBER_TOOLS_CYBER_RECORDER_RECORDER_CONFIG_H_
#define CYBER_TOOLS_CYBER_RECORDER_RECORDER_CONFIG_H_

#include <string>
#include <vector>

#include "cyber/tools/cyber_recorder/record/filters/channel_rate_filter.h"
#include "cyber/tools/cyber_recorder/record/filters/message_size_filter.h"
#include "cyber/tools/cyber_recorder/record/selector/policy_resolver.h"
#include "cyber/tools/cyber_recorder/record/selector/subscription_selector.h"

namespace apollo {
namespace cyber {
namespace record {

struct RecorderConfigBundle {
  uint32_t version = 1;
  SubscriptionSelectorConfig subscription;
  PolicyResolverConfig policies;
};

bool BuildRecorderConfigFromLegacyOptions(
    bool all_channels, const std::vector<std::string>& white_channels,
    const std::vector<std::string>& black_channels,
    const MessageSizeFilterConfig& message_size_filter_config,
    const ChannelRateFilterConfig& channel_rate_filter_config,
    RecorderConfigBundle* config, std::string* error);

bool LoadRecorderConfigFromYamlFile(const std::string& path,
                                    RecorderConfigBundle* config,
                                    std::string* error);

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_RECORDER_CONFIG_H_
