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

#include "modules/drivers/lidar/livox/component/livox_component.h"

#include <iomanip>
#include <string>
#include <utility>

namespace apollo {
namespace drivers {
namespace lidar {

static LivoxLidarComponent* g_livox_component = nullptr;

uint64_t LivoxLidarComponent::GetEthPacketTimestamp(uint8_t handle,
                                                     uint8_t timestamp_type,
                                                     uint8_t* time_stamp,
                                                     uint8_t size) {
  LdsStamp time;
  memcpy(time.stamp_bytes, time_stamp, size);
  uint64_t packet_timestamp = time.stamp;
  uint64_t system_time_ns = cyber::Time::Now().ToNanosecond();

  // Default is use_lidar_clock=true, check if explicitly disabled
  bool use_lidar_clock = config_.has_use_lidar_clock() ?
                         config_.use_lidar_clock() : true;

  // If use_lidar_clock is false, always use system time
  // Only warn once since this is the user's explicit configuration
  if (!use_lidar_clock) {
    if (!warned_not_use_lidar_clock_) {
      AWARN << "[Livox Timestamp] use_lidar_clock is DISABLED. "
            << "Using system time (data arrival time) instead of lidar clock. "
            << "This means measurement_time will NOT reflect laser scan time!";
      warned_not_use_lidar_clock_ = true;
    }
    return system_time_ns;
  }

  // use_lidar_clock=true: Try to use packet timestamp (laser scan time)
  double max_diff_s = config_.has_max_timestamp_diff_s() ?
                       config_.max_timestamp_diff_s() : 10.0;
  uint64_t max_diff_ns = static_cast<uint64_t>(max_diff_s * 1e9);

  // apply timestamp offset
  if (config_.timestamp_offset() != 0) {
    packet_timestamp += static_cast<int64_t>(config_.timestamp_offset() * 1e9);
  }
  // Calculate absolute difference
  uint64_t diff_ns;
  if (packet_timestamp > system_time_ns) {
    diff_ns = packet_timestamp - system_time_ns;
  } else {
    diff_ns = system_time_ns - packet_timestamp;
  }

  // If diff is too large, packet time is likely not synchronized
  if (diff_ns > max_diff_ns) {
    double packet_time_s = packet_timestamp / 1e9;
    double system_time_s = system_time_ns / 1e9;
    double diff_s = static_cast<double>(diff_ns) / 1e9;
    AWARN_EVERY(10000) << std::fixed << std::setprecision(3)
          << "[Livox Ts] Handle=" << static_cast<int>(handle)
          << " type=" << static_cast<int>(timestamp_type)
          << " pkt_ts=" << packet_time_s
          << " sys_ts=" << system_time_s
          << " diff=" << diff_s << "s > " << max_diff_s << "s, using sys time";
    return system_time_ns;
  }

  // Use packet timestamp (lidar clock / laser scan time)
  return packet_timestamp;
}

void LivoxLidarComponent::BinaryDataProcess(const unsigned char* data,
                                            int data_type, int point_size,
                                            uint64_t pkt_timestamp) {
  if (data_type == kExtendCartesian) {
    auto* p_point_data = reinterpret_cast<const LivoxExtendRawPoint*>(data);

    for (int i = 0; i < point_size; ++i) {
      PointXYZIT point;
      point.set_x(p_point_data[i].x * 0.001);
      point.set_y(p_point_data[i].y * 0.001);
      point.set_z(p_point_data[i].z * 0.001);
      point.set_intensity(p_point_data[i].reflectivity);
      point.set_timestamp(pkt_timestamp);
      integral_queue_.push_back(std::move(point));
    }

    uint64_t integral_timestamp_diff = integral_time_ * 1e9;
    while (!integral_queue_.empty() &&
           integral_queue_.front().timestamp() <
               integral_queue_.back().timestamp() - integral_timestamp_diff) {
      integral_queue_.pop_front();
    }

  } else if (data_type == kCartesian) {
    auto* p_point_data = reinterpret_cast<const LivoxRawPoint*>(data);

    for (int i = 0; i < point_size; ++i) {
      PointXYZIT point;
      point.set_x(p_point_data[i].x * 0.001);
      point.set_y(p_point_data[i].y * 0.001);
      point.set_z(p_point_data[i].z * 0.001);
      point.set_intensity(p_point_data[i].reflectivity);
      point.set_timestamp(pkt_timestamp);
      integral_queue_.push_back(std::move(point));
    }

    uint64_t integral_timestamp_diff = integral_time_ * 1e9;
    while (!integral_queue_.empty() &&
           integral_queue_.front().timestamp() <
               integral_queue_.back().timestamp() - integral_timestamp_diff) {
      integral_queue_.pop_front();
    }

  } else if (data_type == kSpherical || data_type == kExtendSpherical) {
    auto* p_point_data = reinterpret_cast<const LivoxSpherPoint*>(data);
    (void)p_point_data;
  }
}

void LivoxLidarComponent::PointCloudCallback(uint8_t handle,
                                             LivoxEthPacket* data,
                                             uint32_t data_num,
                                             void* client_data) {
  if (data == nullptr || data_num == 0) {
    return;
  }

  if (!g_livox_component) {
    return;
  }

  ADEBUG << boost::format(
              "point cloud handle: %d, data_num: %d, data_type: %d") %
              static_cast<int>(handle) % data_num %
              static_cast<int>(data->data_type);

  size_t byte_size = g_livox_component->GetEthPacketByteSize(data, data_num);
  uint64_t pkt_timestamp = g_livox_component->GetEthPacketTimestamp(
      handle, data->timestamp_type, data->timestamp, sizeof(data->timestamp));

  g_livox_component->BinaryDataProcess(data->data, data->data_type, data_num,
                                       pkt_timestamp);

  if (byte_size > 0) {
    std::shared_ptr<livox::LivoxScan> scan_message =
        std::make_shared<livox::LivoxScan>();
    scan_message->set_data_type(data->data_type);
    scan_message->set_timestamp(pkt_timestamp);
    auto* data_addr = static_cast<unsigned char*>(data->data);
    scan_message->set_data(data_addr, byte_size);
    scan_message->set_point_size(data_num);
    g_livox_component->WriteScan(scan_message);
  }

  g_livox_component->CheckTimestampAndPublishPointCloud();
}

size_t LivoxLidarComponent::GetEthPacketByteSize(LivoxEthPacket* data,
                                                 uint32_t data_num) {
  size_t byte_size = 0;
  if (data == nullptr) {
    return 0;
  }
  switch (data->data_type) {
    case kExtendCartesian:
      byte_size = sizeof(LivoxExtendRawPoint) * data_num;
      break;
    case kCartesian:
      byte_size = sizeof(LivoxRawPoint) * data_num;
      break;
    case kSpherical:
      byte_size = sizeof(LivoxSpherPoint) * data_num;
      break;
    case kExtendSpherical:
      byte_size = sizeof(LivoxExtendSpherPoint) * data_num;
      break;
    default:
      byte_size = 0;
      break;
  }
  return byte_size;
}

void LivoxLidarComponent::PreparePointsMsg(PointCloud& msg) {
  msg.set_height(1);
  msg.set_width(msg.point_size() / msg.height());
  msg.set_is_dense(false);
  const auto timestamp =
      msg.point(static_cast<int>(msg.point_size()) - 1).timestamp();
  msg.set_measurement_time(
      GetSecondTimestampFromNanosecondTimestamp(timestamp));

  double lidar_time = GetSecondTimestampFromNanosecondTimestamp(timestamp);
  double diff_time = msg.header().timestamp_sec() - lidar_time;
  if (diff_time > 0.2) {
    AINFO << "timestamp difference too large " << std::fixed
          << std::setprecision(16)
          << "system time: " << msg.header().timestamp_sec()
          << ", lidar time: " << lidar_time << ", diff is:" << diff_time;
  }

  msg.mutable_header()->set_lidar_timestamp(timestamp);
}

void LivoxLidarComponent::OnDeviceBroadcast(const BroadcastDeviceInfo* info) {
  if (!info) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    AINFO << "In lidar mode, ignoring hub device: " << info->broadcast_code;
    return;
  }

  if (!g_livox_component) {
    AERROR << "g_livox_component is nullptr";
    return;
  }

  std::string broadcast_code = g_livox_component->config_.broadcast_code();
  if (!broadcast_code.empty() &&
      broadcast_code != std::string(info->broadcast_code)) {
    AINFO << "Broadcast code mismatch, skipping device: "
          << info->broadcast_code;
    return;
  }

  livox_status result = kStatusFailure;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    g_livox_component->lidar_handle_ = handle;
    SetDataCallback(handle, LivoxLidarComponent::PointCloudCallback, nullptr);
    AINFO << "Livox lidar added to connect. handle="
          << static_cast<int>(handle);
  } else {
    AERROR << "Add lidar to connect failed. result=" << result
           << " handle=" << static_cast<int>(handle);
  }
}

void LivoxLidarComponent::OnDeviceStateUpdate(const DeviceInfo* device,
                                              DeviceEvent event) {
  if (!device) {
    return;
  }

  if (!g_livox_component) {
    return;
  }

  uint8_t handle = device->handle;

  if (event == kEventConnect) {
    AINFO << "Lidar[" << device->broadcast_code << "] connected!";
    QueryDeviceInformation(handle, nullptr, nullptr);
  } else if (event == kEventDisconnect) {
    AERROR << "Lidar[" << device->broadcast_code << "] disconnected!";
  } else if (event == kEventStateChange) {
    AINFO << "Lidar[" << device->broadcast_code << "] state change to "
          << static_cast<int>(device->state);
    if (device->state == kLidarStateNormal) {
      LidarStartSampling(handle, LivoxLidarComponent::OnLidarStartSamplingCb,
                         g_livox_component);
    }
  }
}

void LivoxLidarComponent::OnLidarStartSamplingCb(livox_status status,
                                                 uint8_t handle,
                                                 uint8_t response,
                                                 void* client_data) {
  if (status != kStatusSuccess || response != 0) {
    AERROR << "Lidar start sampling failed. status=" << status
           << " handle=" << static_cast<int>(handle)
           << " response=" << static_cast<int>(response);
    return;
  }
  AINFO << "Lidar start sampling success. handle=" << static_cast<int>(handle);
}

bool LivoxLidarComponent::Init() {
  GetProtoConfig(&config_);
  RETURN_VAL_IF(
      !LidarComponentBase<livox::LivoxScan>::InitBase(config_.config_base()),
      false);

  integral_time_ = config_.integral_time();

  if (config_.config_base().source_type() ==
      LidarConfigBase_SourceType_ONLINE_LIDAR) {
    if (!config_.has_enable_sdk_console_log() ||
        !config_.enable_sdk_console_log()) {
      DisableConsoleLogger();
    }

    g_livox_component = this;

    if (!::Init()) {
      ::Uninit();
      AERROR << "Livox SDK init fail!";
      return false;
    }

    SetBroadcastCallback(LivoxLidarComponent::OnDeviceBroadcast);
    SetDeviceStateUpdateCallback(LivoxLidarComponent::OnDeviceStateUpdate);

    if (!Start()) {
      ::Uninit();
      AERROR << "Livox SDK start fail!";
      return false;
    }

    AINFO << "Livox SDK initialized, waiting for device discovery...";
    sdk_initialized_ = true;
  }
  return true;
}

void LivoxLidarComponent::ReadScanCallback(
    const std::shared_ptr<livox::LivoxScan>& scan_message) {
  auto data_addr = (unsigned char*)scan_message->data().c_str();
  BinaryDataProcess(data_addr, scan_message->data_type(),
                    scan_message->point_size(), scan_message->timestamp());
  CheckTimestampAndPublishPointCloud();
}

void LivoxLidarComponent::CheckTimestampAndPublishPointCloud() {
  if (!integral_queue_.empty()) {
    uint64_t timestamp_now = cyber::Time::Now().ToNanosecond();
    uint64_t timestamp_dist = timestamp_now - last_pointcloud_pub_timestamp_;
    uint64_t tolerable_timestamp = (1e9 / pointcloud_freq_);

    if (timestamp_dist > tolerable_timestamp) {
      auto pcd_frame =
          LidarComponentBase<livox::LivoxScan>::AllocatePointCloud();

      for (auto it = integral_queue_.begin(); it != integral_queue_.end();
           ++it) {
        auto point = pcd_frame->add_point();
        point->CopyFrom(*it);
      }

      PreparePointsMsg(*pcd_frame);
      LidarComponentBase<livox::LivoxScan>::WritePointCloud(pcd_frame);
      AINFO << "pcd frame write, Publish timestamp = " << timestamp_now;
      last_pointcloud_pub_timestamp_ = timestamp_now;
    }
  }
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
