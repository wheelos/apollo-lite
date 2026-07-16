/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef CYBER_TOOLS_CYBER_RECORDER_RECORDER_H_
#define CYBER_TOOLS_CYBER_RECORDER_RECORDER_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cyber/base/signal.h"
#include "cyber/cyber.h"
#include "cyber/message/raw_message.h"
#include "cyber/proto/record.pb.h"
#include "cyber/proto/topology_change.pb.h"
#include "cyber/record/record_writer.h"
#include "cyber/tools/cyber_recorder/record/core/conflict_warning_emitter.h"
#include "cyber/tools/cyber_recorder/record/core/recorder_config.h"
#include "cyber/tools/cyber_recorder/record/core/subscription_set.h"
#include "cyber/tools/cyber_recorder/record/filters/channel_rate_filter.h"
#include "cyber/tools/cyber_recorder/record/filters/message_size_filter.h"
#include "cyber/tools/cyber_recorder/record/filters/qos_stateful_filter.h"
#include "cyber/tools/cyber_recorder/record/selector/policy_resolver.h"
#include "cyber/tools/cyber_recorder/record/selector/subscription_selector.h"

using apollo::cyber::Node;
using apollo::cyber::ReaderBase;
using apollo::cyber::base::Connection;
using apollo::cyber::message::RawMessage;
using apollo::cyber::proto::ChangeMsg;
using apollo::cyber::proto::RoleAttributes;
using apollo::cyber::proto::RoleType;
using apollo::cyber::service_discovery::ChannelManager;
using apollo::cyber::service_discovery::TopologyManager;

namespace apollo {
namespace cyber {
namespace record {

class Recorder : public std::enable_shared_from_this<Recorder> {
 public:
  Recorder(const std::string& output, bool all_channels,
           const std::vector<std::string>& white_channels,
           const std::vector<std::string>& black_channels);
  Recorder(const std::string& output, bool all_channels,
           const std::vector<std::string>& white_channels,
           const std::vector<std::string>& black_channels,
           const proto::Header& header);
  Recorder(const std::string& output, bool all_channels,
           const std::vector<std::string>& white_channels,
           const std::vector<std::string>& black_channels,
           const proto::Header& header,
           const MessageSizeFilterConfig& message_size_filter_config);
  Recorder(const std::string& output, bool all_channels,
           const std::vector<std::string>& white_channels,
           const std::vector<std::string>& black_channels,
           const proto::Header& header,
           const MessageSizeFilterConfig& message_size_filter_config,
           const ChannelRateFilterConfig& channel_rate_filter_config);
  Recorder(const std::string& output, const proto::Header& header,
           const RecorderConfigBundle& config_bundle);
  ~Recorder();
  bool Start();
  bool Stop();

 private:
  struct ChannelMetadata {
    std::string message_type;
    std::string proto_desc;
  };

  std::atomic<bool> is_started_{false};
  std::atomic<bool> is_stopping_{false};
  std::shared_ptr<Node> node_ = nullptr;
  std::shared_ptr<RecordWriter> writer_ = nullptr;
  std::shared_ptr<std::thread> display_thread_ = nullptr;
  Connection<const ChangeMsg&> change_conn_;
  std::string output_;
  proto::Header header_;
  SubscriptionSelector subscription_selector_;
  PolicyResolver policy_resolver_;
  QosStatefulFilter qos_stateful_filter_;
  ConflictWarningEmitter conflict_warning_emitter_;
  SubscriptionSet subscription_set_;
  mutable std::mutex channel_reader_mutex_;
  std::unordered_map<std::string, ChannelMetadata> channel_metadata_map_;
  std::unordered_set<std::string> written_channels_;
  std::atomic<uint64_t> message_count_{0};
  std::atomic<uint64_t> message_time_{0};
  std::atomic<uint64_t> dropped_message_count_{0};
  std::atomic<uint64_t> throttled_message_count_{0};

  bool InitReadersImpl();

  bool FreeReadersImpl();

  bool EnsureSubscriptionReader(const std::string& channel_name,
                                const std::string& message_type);

  void TopologyCallback(const ChangeMsg& msg);

  void ReaderCallback(const std::shared_ptr<RawMessage>& message,
                      const std::string& channel_name);

  void FindNewChannel(const RoleAttributes& role_attr);

  void ShowProgress();
  size_t ChannelCount() const;
  bool EnsureChannelWritten(const std::string& channel_name);
};

}  // namespace record
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_TOOLS_CYBER_RECORDER_RECORDER_H_
