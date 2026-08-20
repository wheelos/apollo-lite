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

#include "modules/drivers/lidar/processor/transport/pod_pointcloud_adapter.h"

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool DecodePodPointCloud(
    const apollo::cyber::transport::PodMessage& message,
    uint32_t expected_schema_hash, PointCloud* point_cloud) {
  if (point_cloud == nullptr) {
    return false;
  }

  const auto view = message.View();
  if (view.payload == nullptr ||
      view.header.payload_kind != static_cast<uint32_t>(
          apollo::cyber::transport::PodPayloadKind::POINT_CLOUD) ||
      view.payload_size >
          static_cast<size_t>(std::numeric_limits<int>::max()) ||
      (expected_schema_hash != 0U &&
       view.header.schema_hash != expected_schema_hash)) {
    return false;
  }

  return point_cloud->ParseFromArray(
      view.payload, static_cast<int>(view.payload_size));
}

bool PodPointCloudAdapter::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "Failed to load POD point-cloud adapter config: "
           << ConfigFilePath();
    return false;
  }

  std::set<std::string> input_channels;
  std::set<std::string> output_channels;
  for (const auto& channel : config_.channels()) {
    if (channel.input_channel().empty() || channel.output_channel().empty() ||
        !input_channels.insert(channel.input_channel()).second ||
        !output_channels.insert(channel.output_channel()).second) {
      AERROR << "POD point-cloud adapter channels must be non-empty and unique";
      return false;
    }

    auto writer = node_->CreateWriter<PointCloud>(channel.output_channel());
    if (writer == nullptr) {
      AERROR << "Failed to create point-cloud writer: "
             << channel.output_channel();
      return false;
    }
    writers_.emplace(channel.input_channel(), std::move(writer));
    expected_schema_hashes_.emplace(channel.input_channel(),
                                    channel.expected_schema_hash());
  }

  for (const auto& channel : config_.channels()) {
    auto reader =
        node_->CreateReader<apollo::cyber::transport::PodMessage>(
            channel.input_channel(),
            [this, input_channel = channel.input_channel()](
                const std::shared_ptr<
                    apollo::cyber::transport::PodMessage>& message) {
              OnMessage(input_channel, message);
            });
    if (reader == nullptr) {
      AERROR << "Failed to create POD point-cloud reader: "
             << channel.input_channel();
      return false;
    }
    readers_.emplace(channel.input_channel(), std::move(reader));
  }

  return !readers_.empty();
}

void PodPointCloudAdapter::OnMessage(
    const std::string& input_channel,
    const std::shared_ptr<apollo::cyber::transport::PodMessage>& message) {
  const auto writer = writers_.find(input_channel);
  const auto schema_hash = expected_schema_hashes_.find(input_channel);
  if (message == nullptr || writer == writers_.end() ||
      schema_hash == expected_schema_hashes_.end()) {
    AERROR << "Invalid POD point-cloud adapter route: " << input_channel;
    return;
  }

  auto point_cloud = std::make_shared<PointCloud>();
  if (!DecodePodPointCloud(*message, schema_hash->second, point_cloud.get())) {
    AERROR << "Failed to decode POD protobuf point cloud from "
           << input_channel;
    return;
  }
  writer->second->Write(point_cloud);
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
