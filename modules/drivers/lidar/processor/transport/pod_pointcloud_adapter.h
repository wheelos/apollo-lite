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

#pragma once

#include <map>
#include <memory>
#include <string>

#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/component/component.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"
#include "cyber/transport/message/pod_message.h"
#include "modules/drivers/lidar/proto/pod_pointcloud_adapter_config.pb.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool DecodePodPointCloud(
    const apollo::cyber::transport::PodMessage& message,
    uint32_t expected_schema_hash, PointCloud* point_cloud);

class PodPointCloudAdapter final : public apollo::cyber::Component<> {
 public:
  bool Init() override;

 private:
  void OnMessage(const std::string& input_channel,
                 const std::shared_ptr<apollo::cyber::transport::PodMessage>&
                     message);

  PodPointCloudAdapterConfig config_;
  std::map<std::string, uint32_t> expected_schema_hashes_;
  std::map<
      std::string,
      std::shared_ptr<apollo::cyber::Reader<
          apollo::cyber::transport::PodMessage>>>
      readers_;
  std::map<std::string, std::shared_ptr<apollo::cyber::Writer<PointCloud>>>
      writers_;
};

CYBER_REGISTER_COMPONENT(PodPointCloudAdapter)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
