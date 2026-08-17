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

#include <string>

#include <gtest/gtest.h>

namespace apollo {
namespace drivers {
namespace lidar {

TEST(PodPointCloudAdapterTest, DecodesProtobufPointCloudPayload) {
  PointCloud source;
  source.set_frame_id("lidar");
  source.set_measurement_time(10.0);
  source.add_point()->set_x(1.0F);
  std::string payload;
  ASSERT_TRUE(source.SerializeToString(&payload));

  apollo::cyber::transport::PodChunkHeader header;
  header.payload_kind = static_cast<uint32_t>(
      apollo::cyber::transport::PodPayloadKind::POINT_CLOUD);
  header.payload_size = payload.size();
  header.schema_hash = 42U;
  apollo::cyber::transport::PodMessage message(
      header, payload.data(), payload.size());

  PointCloud decoded;
  ASSERT_TRUE(DecodePodPointCloud(message, 42U, &decoded));
  EXPECT_EQ(decoded.frame_id(), "lidar");
  ASSERT_EQ(decoded.point_size(), 1);
  EXPECT_FLOAT_EQ(decoded.point(0).x(), 1.0F);
}

TEST(PodPointCloudAdapterTest, RejectsWrongKindOrSchema) {
  const std::string payload = "not-a-point-cloud";
  apollo::cyber::transport::PodChunkHeader header;
  header.payload_kind = static_cast<uint32_t>(
      apollo::cyber::transport::PodPayloadKind::IMAGE);
  header.payload_size = payload.size();
  header.schema_hash = 7U;
  apollo::cyber::transport::PodMessage message(
      header, payload.data(), payload.size());

  PointCloud decoded;
  EXPECT_FALSE(DecodePodPointCloud(message, 7U, &decoded));

  header.payload_kind = static_cast<uint32_t>(
      apollo::cyber::transport::PodPayloadKind::POINT_CLOUD);
  apollo::cyber::transport::PodMessage point_message(
      header, payload.data(), payload.size());
  EXPECT_FALSE(DecodePodPointCloud(point_message, 8U, &decoded));
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
