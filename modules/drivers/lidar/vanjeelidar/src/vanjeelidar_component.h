/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include <thread>

#include <vanjee_driver/msg/device_ctrl_msg.hpp>
#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/packet.hpp>
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>
#include <vanjee_driver/utility/sync_queue.hpp>

#include "modules/drivers/lidar/vanjeelidar/proto/vanjeelidar.pb.h"
#include "modules/drivers/lidar/vanjeelidar/proto/vanjeelidar_config.pb.h"

#include "modules/drivers/lidar/common/lidar_component_base.h"
#include "modules/drivers/lidar/common/sync_buffering.h"

namespace vanjee {
namespace lidar {
template <typename T_PointCloud>
class LidarDriver;
}  // namespace lidar
}  // namespace vanjee

namespace apollo {
namespace drivers {
namespace lidar {

typedef ::vanjee::lidar::PointXYZIRT PointT;
typedef ::vanjee::lidar::PointCloudT<PointT> PointCloudMsg;

class VanjeelidarComponent : public LidarComponentBase<vanjee::VanjeePacket> {
 public:
  ~VanjeelidarComponent();
  bool Init() override;

  void ReadScanCallback(
      const std::shared_ptr<vanjee::VanjeePacket>& packet) override;

  void VanjeePacketCallback(std::shared_ptr<::vanjee::lidar::Packet> pkt);

  std::shared_ptr<::vanjee::lidar::ScanData> VanjeeScanDataAllocateCallback();

  void VanjeeScanDataPutCallback(
      std::shared_ptr<::vanjee::lidar::ScanData> scan_data);

  std::shared_ptr<::vanjee::lidar::ImuPacket> VanjeeImuPacketAllocateCallback();

  void VanjeeImuPacketPutCallback(
      std::shared_ptr<::vanjee::lidar::ImuPacket> imu_packet);

  std::shared_ptr<PointCloudMsg> VanjeeCloudAllocateCallback();

  void VanjeeCloudPutCallback(std::shared_ptr<PointCloudMsg> vanjee_cloud);

  std::shared_ptr<::vanjee::lidar::DeviceCtrl>
  VanjeeDeviceCtrlAllocateCallback();
  void VanjeeDeviceCtrlPutCallback(
      std::shared_ptr<::vanjee::lidar::DeviceCtrl> device_ctrl);

  void ProcessCloud();

 private:
  friend class VanjeelidarComponentTest;

  std::shared_ptr<::vanjee::lidar::LidarDriver<PointCloudMsg>> driver_ptr_;
  apollo::drivers::vanjee::Config conf_;
  std::shared_ptr<cyber::Writer<vanjee::VanjeeImu>> imu_writer_;

  ::vanjee::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> cloud_queue_;
  ::vanjee::lidar::SyncQueue<std::shared_ptr<::vanjee::lidar::ScanData>>
      scan_queue_;
  ::vanjee::lidar::SyncQueue<std::shared_ptr<::vanjee::lidar::DeviceCtrl>>
      device_ctrl_queue_;
  std::shared_ptr<SyncBuffering<PointCloudMsg>> cloud_buffer_;
  std::shared_ptr<SyncBuffering<::vanjee::lidar::ScanData>> scan_buffer_;
  std::shared_ptr<SyncBuffering<::vanjee::lidar::ImuPacket>> imu_buffer_;
  std::shared_ptr<SyncBuffering<::vanjee::lidar::DeviceCtrl>>
      device_ctrl_buffer_;

  std::thread cloud_handle_thread_;
  std::atomic<bool> running_{false};
};
CYBER_REGISTER_COMPONENT(VanjeelidarComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
