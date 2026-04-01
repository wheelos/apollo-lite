/******************************************************************************
 * Copyright 2026 The Wheelos Team. All Rights Reserved.
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

#include <livox_def.h>  //NOLINT
#include <livox_sdk.h>  //NOLINT

#include <deque>
#include <memory>
#include <string>

#include <boost/format.hpp>

#include "modules/drivers/lidar/livox/proto/livox.pb.h"

#include "modules/drivers/lidar/common/lidar_component_base.h"
#include "modules/drivers/lidar/common/util.h"

namespace apollo {
namespace drivers {
namespace lidar {

enum TimestampType {
  kTimestampTypeNoSync = 0,    /**< No sync signal mode. */
  kTimestampTypeGptpOrPtp = 1, /**< gPTP or PTP sync mode */
  kTimestampTypeGps = 2        /**< GPS sync mode. */
};

/** 8bytes stamp to uint64_t stamp */
typedef union {
  struct {
    uint32_t low;
    uint32_t high;
  } stamp_word;
  uint8_t stamp_bytes[8];
  int64_t stamp;
} LdsStamp;

uint64_t GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp,
                               uint8_t size);

class LivoxLidarComponent final : public LidarComponentBase<livox::LivoxScan> {
 public:
  void BinaryDataProcess(const unsigned char* data, int data_type,
                         int point_size, uint64_t pkt_timestamp);
  static void PointCloudCallback(uint8_t handle, LivoxEthPacket* data,
                                 uint32_t data_num, void* client_data);
  size_t GetEthPacketByteSize(LivoxEthPacket* data, uint32_t data_num);
  void PreparePointsMsg(PointCloud& msg);
  bool Init() override;
  void ReadScanCallback(
      const std::shared_ptr<livox::LivoxScan>& scan_message) override;
  void CheckTimestampAndPublishPointCloud();
  static void OnDeviceBroadcast(const BroadcastDeviceInfo* info);
  static void OnDeviceStateUpdate(const DeviceInfo* device, DeviceEvent event);
  static void OnLidarStartSamplingCb(livox_status status, uint8_t handle,
                                     uint8_t response, void* client_data);
  livox::Config config_;
  std::deque<PointXYZIT> integral_queue_;
  uint64_t last_pointcloud_pub_timestamp_{0};
  double pointcloud_freq_ = {10.0};
  double integral_time_ = {0.4};
  uint8_t lidar_handle_ = {0};
  bool sdk_initialized_ = {false};
};
CYBER_REGISTER_COMPONENT(LivoxLidarComponent)
}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
