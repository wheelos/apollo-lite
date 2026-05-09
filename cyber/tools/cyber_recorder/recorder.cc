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

#include "cyber/tools/cyber_recorder/recorder.h"

#include <algorithm>

#include "cyber/record/header_builder.h"

namespace apollo {
namespace cyber {
namespace record {

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels)
    : output_(output),
      all_channels_(all_channels),
      white_channels_(white_channels),
      black_channels_(black_channels) {
  header_ = HeaderBuilder::GetHeader();
}

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels,
                   const proto::Header& header)
    : output_(output),
      all_channels_(all_channels),
      white_channels_(white_channels),
      black_channels_(black_channels),
      header_(header) {}

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels,
                   const proto::Header& header,
                   const MessageSizeFilterConfig& message_size_filter_config)
    : output_(output),
      all_channels_(all_channels),
      white_channels_(white_channels),
      black_channels_(black_channels),
      header_(header),
      message_size_filter_(message_size_filter_config) {}

Recorder::Recorder(const std::string& output, bool all_channels,
                   const std::vector<std::string>& white_channels,
                   const std::vector<std::string>& black_channels,
                   const proto::Header& header,
                   const MessageSizeFilterConfig& message_size_filter_config,
                   const ChannelRateFilterConfig& channel_rate_filter_config)
    : output_(output),
      all_channels_(all_channels),
      white_channels_(white_channels),
      black_channels_(black_channels),
      header_(header),
      message_size_filter_(message_size_filter_config),
      channel_rate_filter_(channel_rate_filter_config) {}

Recorder::~Recorder() { Stop(); }

bool Recorder::Start() {
  for (const auto& channel_name : white_channels_) {
    if (std::find(black_channels_.begin(), black_channels_.end(),
                  channel_name) != black_channels_.end()) {
      AERROR << "find channel in both of white list and black list, channel: "
             << channel_name;
      return false;
    }
  }

  writer_.reset(new RecordWriter(header_));
  if (!writer_->Open(output_)) {
    AERROR << "Datafile open file error.";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(channel_reader_mutex_);
    channel_reader_map_.clear();
    channel_metadata_map_.clear();
    written_channels_.clear();
  }
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
    std::lock_guard<std::mutex> lock(channel_reader_mutex_);
    channel_reader_map_.clear();
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
  if (!all_channels_ &&
      std::find(white_channels_.begin(), white_channels_.end(),
                role_attr.channel_name()) == white_channels_.end()) {
    ADEBUG << "New channel '" << role_attr.channel_name()
           << "' was found, but not in record list.";
    return;
  }

  if (std::find(black_channels_.begin(), black_channels_.end(),
      role_attr.channel_name()) != black_channels_.end()) {
    ADEBUG << "New channel '" << role_attr.channel_name()
           << "' was found, but it appears in the blacklist.";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(channel_reader_mutex_);
    if (channel_reader_map_.find(role_attr.channel_name()) !=
        channel_reader_map_.end()) {
      return;
    }
    channel_metadata_map_[role_attr.channel_name()] = {
        role_attr.message_type(), role_attr.proto_desc()};
  }
  if (!InitReaderImpl(role_attr.channel_name(), role_attr.message_type())) {
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

bool Recorder::InitReaderImpl(const std::string& channel_name,
                              const std::string& message_type) {
  try {
    std::weak_ptr<Recorder> weak_this = shared_from_this();
    std::shared_ptr<ReaderBase> reader = nullptr;
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
    reader = node_->CreateReader<RawMessage>(config, callback);
    if (reader == nullptr) {
      AERROR << "Create reader failed.";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(channel_reader_mutex_);
      channel_reader_map_[channel_name] = reader;
    }
    return true;
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

  const uint64_t record_time_ns = Time::Now().ToNanosecond();
  message_time_.store(record_time_ns);
  const auto filter_decision = message_size_filter_.Evaluate(
      channel_name, message->message.size(), record_time_ns);
  if (!filter_decision.should_record) {
    if (filter_decision.dropped_by_size) {
      dropped_message_count_.fetch_add(1);
    }
    return;
  }
  const auto channel_rate_decision =
      channel_rate_filter_.Evaluate(channel_name, record_time_ns);
  if (!channel_rate_decision.should_record) {
    if (channel_rate_decision.throttled_by_rate) {
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
  std::lock_guard<std::mutex> lock(channel_reader_mutex_);
  return channel_reader_map_.size();
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
