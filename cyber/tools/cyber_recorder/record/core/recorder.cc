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

#include "cyber/tools/cyber_recorder/record/core/recorder.h"

#include <stdexcept>

#include "cyber/record/header_builder.h"

namespace apollo {
namespace cyber {
namespace record {
namespace {

RecorderConfigBundle BuildLegacyConfigOrThrow(
    const bool all_channels, const std::vector<std::string>& white_channels,
    const std::vector<std::string>& black_channels,
    const MessageSizeFilterConfig& message_size_filter_config,
    const ChannelRateFilterConfig& channel_rate_filter_config) {
  RecorderConfigBundle config;
  std::string error;
  if (!BuildRecorderConfigFromLegacyOptions(
          all_channels, white_channels, black_channels,
          message_size_filter_config, channel_rate_filter_config, &config,
          &error)) {
    throw std::invalid_argument(error);
  }
  return config;
}

}  // namespace

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels)
    : Recorder(output, all_channels, white_channels, black_channels,
               HeaderBuilder::GetHeader()) {}

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels,
                   const proto::Header& header)
    : Recorder(output, all_channels, white_channels, black_channels, header,
               MessageSizeFilterConfig()) {}

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels,
                   const proto::Header& header,
                   const MessageSizeFilterConfig& message_size_filter_config)
    : Recorder(output, all_channels, white_channels, black_channels, header,
               message_size_filter_config, ChannelRateFilterConfig()) {}

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels,
                   const proto::Header& header,
                   const MessageSizeFilterConfig& message_size_filter_config,
                   const ChannelRateFilterConfig& channel_rate_filter_config)
    : Recorder(output, header,
               BuildLegacyConfigOrThrow(all_channels, white_channels,
                                        black_channels,
                                        message_size_filter_config,
                                        channel_rate_filter_config)) {}

Recorder::Recorder(const std::string& output, const proto::Header& header,
                   const RecorderConfigBundle& config_bundle)
    : output_(output),
      header_(header),
      subscription_selector_(config_bundle.subscription),
      policy_resolver_(config_bundle.policies),
      conflict_warning_emitter_(config_bundle.version) {}

Recorder::~Recorder() { Stop(); }

bool Recorder::Start() {
  writer_.reset(new RecordWriter(header_));
  if (!writer_->Open(output_)) {
    AERROR << "Datafile open file error.";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(channel_reader_mutex_);
    channel_metadata_map_.clear();
    written_channels_.clear();
  }
  subscription_set_.Clear();
  std::string node_name = "cyber_recorder_record_" + std::to_string(getpid());
  node_ = ::apollo::cyber::CreateNode(node_name);
  if (node_ == nullptr) {
    AERROR << "create node failed, node: " << node_name;
    return false;
  }
  if (!InitReadersImpl()) {
    AERROR << " _init_readers error.";
    return false;
  }
  message_count_.store(0);
  message_time_.store(0);
  dropped_message_count_.store(0);
  throttled_message_count_.store(0);
  is_started_.store(true);
  display_thread_ =
      std::make_shared<std::thread>([this]() { this->ShowProgress(); });
  if (display_thread_ == nullptr) {
    AERROR << "init display thread error.";
    return false;
  }
  return true;
}

bool Recorder::Stop() {
  if (!is_started_.load() || is_stopping_.exchange(true)) {
    return false;
  }
  if (!FreeReadersImpl()) {
    AERROR << " _free_readers error.";
    return false;
  }
  writer_->Close();
  node_.reset();
  {
    subscription_set_.Clear();
  }
  if (display_thread_ && display_thread_->joinable()) {
    display_thread_->join();
    display_thread_ = nullptr;
  }
  is_started_.store(false);
  is_stopping_.store(false);
  return true;
}

void Recorder::TopologyCallback(const ChangeMsg& change_message) {
  ADEBUG << "ChangeMsg in Topology Callback:" << std::endl
         << change_message.ShortDebugString();
  if (change_message.role_type() != apollo::cyber::proto::ROLE_WRITER) {
    ADEBUG << "Change message role type is not ROLE_WRITER.";
    return;
  }
  FindNewChannel(change_message.role_attr());
}

void Recorder::FindNewChannel(const RoleAttributes& role_attr) {
  if (!role_attr.has_channel_name() || role_attr.channel_name().empty()) {
    AWARN << "change message not has a channel name or has an empty one.";
    return;
  }
  if (!role_attr.has_message_type() || role_attr.message_type().empty()) {
    AWARN << "Change message not has a message type or has an empty one.";
    return;
  }
  if (!role_attr.has_proto_desc() || role_attr.proto_desc().empty()) {
    AWARN << "Change message not has a proto desc or has an empty one.";
    return;
  }
  const TopicMetadata metadata = {role_attr.channel_name(),
                                  role_attr.message_type()};
  const SubscriptionDecision subscription_decision =
      subscription_selector_.Evaluate(metadata);
  if (subscription_decision.has_conflict) {
    conflict_warning_emitter_.WarnSubscriptionConflict(
        metadata.topic, subscription_decision.include_matches,
        subscription_decision.exclude_matches);
  }

  {
    std::lock_guard<std::mutex> lock(channel_reader_mutex_);
    channel_metadata_map_[role_attr.channel_name()] = {
        role_attr.message_type(), role_attr.proto_desc()};
  }

  if (!subscription_decision.should_subscribe) {
    subscription_set_.Unsubscribe(role_attr.channel_name());
    return;
  }
  if (!EnsureSubscriptionReader(role_attr.channel_name(),
                                role_attr.message_type())) {
    AERROR << "init reader fail, channel:" << role_attr.channel_name();
  }
}

bool Recorder::InitReadersImpl() {
  std::shared_ptr<ChannelManager> channel_manager =
      TopologyManager::Instance()->channel_manager();

  // get historical writers
  std::vector<proto::RoleAttributes> role_attr_vec;
  channel_manager->GetWriters(&role_attr_vec);
  for (auto role_attr : role_attr_vec) {
    FindNewChannel(role_attr);
  }

  // listen new writers in future
  change_conn_ = channel_manager->AddChangeListener(
      std::bind(&Recorder::TopologyCallback, this, std::placeholders::_1));
  if (!change_conn_.IsConnected()) {
    AERROR << "change connection is not connected";
    return false;
  }
  return true;
}

bool Recorder::FreeReadersImpl() {
  std::shared_ptr<ChannelManager> channel_manager =
      TopologyManager::Instance()->channel_manager();

  channel_manager->RemoveChangeListener(change_conn_);

  return true;
}

bool Recorder::EnsureSubscriptionReader(const std::string& channel_name,
                                        const std::string& message_type) {
  (void)message_type;
  try {
    std::weak_ptr<Recorder> weak_this = shared_from_this();
    return subscription_set_.EnsureSubscribed(
        channel_name, [this, weak_this, channel_name]() {
          auto callback = [weak_this, channel_name](
                              const std::shared_ptr<RawMessage>& raw_message) {
            auto share_this = weak_this.lock();
            if (!share_this) {
              return;
            }
            share_this->ReaderCallback(raw_message, channel_name);
          };
          ReaderConfig config;
          config.channel_name = channel_name;
          config.pending_queue_size =
              gflags::Int32FromEnv("CYBER_PENDING_QUEUE_SIZE", 50);
          auto reader = node_->CreateReader<RawMessage>(config, callback);
          if (reader == nullptr) {
            AERROR << "Create reader failed.";
          }
          return reader;
        });
  } catch (const std::bad_weak_ptr& e) {
    AERROR << e.what();
    return false;
  }
}

void Recorder::ReaderCallback(const std::shared_ptr<RawMessage>& message,
                              const std::string& channel_name) {
  if (!is_started_.load() || is_stopping_.load()) {
    AERROR << "record procedure is not started or stopping.";
    return;
  }

  if (message == nullptr) {
    AERROR << "message is nullptr, channel: " << channel_name;
    return;
  }

  if (!subscription_set_.Contains(channel_name)) {
    return;
  }

  const uint64_t record_time_ns = Time::Now().ToNanosecond();
  message_time_.store(record_time_ns);
  const auto resolved_policy = policy_resolver_.Resolve(channel_name);
  if (resolved_policy.has_conflict) {
    conflict_warning_emitter_.WarnPolicyConflict(
        channel_name, resolved_policy.selected_rule,
        resolved_policy.shadowed_rules);
  }
  const auto filter_decision = qos_stateful_filter_.Evaluate(
      channel_name, message->message.size(), record_time_ns,
      resolved_policy.policy);
  if (!filter_decision.should_record) {
    if (filter_decision.dropped_by_size) {
      dropped_message_count_.fetch_add(1);
    }
    if (filter_decision.throttled_by_rate ||
        filter_decision.throttled_by_bandwidth) {
      throttled_message_count_.fetch_add(1);
    }
    return;
  }
  if (!EnsureChannelWritten(channel_name)) {
    AERROR << "write channel metadata fail, channel: " << channel_name;
    return;
  }
  if (!writer_->WriteMessage(channel_name, message, record_time_ns)) {
    AERROR << "write data fail, channel: " << channel_name;
    return;
  }

  message_count_.fetch_add(1);
}

void Recorder::ShowProgress() {
  while (is_started_.load() && !is_stopping_.load()) {
    std::cout << "\r[RUNNING]  Record Time: " << std::setprecision(3)
              << message_time_.load() / 1000000000
              << "    Progress: " << ChannelCount() << " channels, "
              << message_count_.load() << " messages, "
              << dropped_message_count_.load() << " dropped, "
              << throttled_message_count_.load() << " throttled";
    std::cout.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << std::endl;
}

size_t Recorder::ChannelCount() const {
  return subscription_set_.Size();
}

bool Recorder::EnsureChannelWritten(const std::string& channel_name) {
  ChannelMetadata metadata;
  {
    std::lock_guard<std::mutex> lock(channel_reader_mutex_);
    if (written_channels_.find(channel_name) != written_channels_.end()) {
      return true;
    }
    const auto metadata_it = channel_metadata_map_.find(channel_name);
    if (metadata_it == channel_metadata_map_.end()) {
      AERROR << "channel metadata not found, channel: " << channel_name;
      return false;
    }
    metadata = metadata_it->second;
  }

  if (!writer_->WriteChannel(channel_name, metadata.message_type,
                             metadata.proto_desc)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(channel_reader_mutex_);
  written_channels_.insert(channel_name);
  return true;
}

}  // namespace record
}  // namespace cyber
}  // namespace apollo
