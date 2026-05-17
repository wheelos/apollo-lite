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

#include "modules/drivers/lidar/vanjeelidar/src/vanjeelidar_component.h"

#include <memory>
#include <vector>

#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/utility/buffer.hpp>

namespace apollo {
namespace drivers {
namespace lidar {

VanjeelidarComponent::~VanjeelidarComponent() {
  if (cloud_handle_thread_.joinable()) {
    cloud_handle_thread_.join();
  }
  if (driver_ptr_ != nullptr) {
    driver_ptr_->stop();
    driver_ptr_ = nullptr;
  }
}

bool VanjeelidarComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "load config error, file:" << config_file_path_;
    return false;
  }

  this->InitBase(conf_.config_base());

  scan_buffer_ = std::make_shared<SyncBuffering<::vanjee::lidar::ScanData>>();
  scan_buffer_->Init();
  imu_buffer_ = std::make_shared<SyncBuffering<::vanjee::lidar::ImuPacket>>();
  imu_buffer_->Init();
  cloud_buffer_ = std::make_shared<SyncBuffering<PointCloudMsg>>();
  cloud_buffer_->Init();
  device_ctrl_buffer_ =
      std::make_shared<SyncBuffering<::vanjee::lidar::DeviceCtrl>>();
  device_ctrl_buffer_->Init();

  cloud_handle_thread_ = std::thread(&VanjeelidarComponent::ProcessCloud, this);
  // TODO(All): add thread to handle scan data and imu data

  driver_ptr_ = std::make_shared<::vanjee::lidar::LidarDriver<PointCloudMsg>>();

  // TODO(All): implement in a more elegant way
  ::vanjee::lidar::WJDecoderParam decoder_param;
  decoder_param.config_from_file =
      conf_.driver_param().decoder_param().config_from_file();
  decoder_param.wait_for_difop =
      conf_.driver_param().decoder_param().wait_for_difop();
  decoder_param.min_distance =
      conf_.driver_param().decoder_param().min_distance();
  decoder_param.max_distance =
      conf_.driver_param().decoder_param().max_distance();
  decoder_param.start_angle =
      conf_.driver_param().decoder_param().start_angle();
  decoder_param.end_angle = conf_.driver_param().decoder_param().end_angle();
  decoder_param.use_lidar_clock =
      conf_.driver_param().decoder_param().use_lidar_clock();
  decoder_param.dense_points =
      conf_.driver_param().decoder_param().dense_points();
  decoder_param.ts_first_point =
      conf_.driver_param().decoder_param().ts_first_point();
  decoder_param.use_offset_timestamp =
      conf_.driver_param().decoder_param().use_offset_timestamp();
  decoder_param.publish_mode =
      conf_.driver_param().decoder_param().publish_mode();
  decoder_param.rpm = conf_.driver_param().decoder_param().rpm();
  decoder_param.angle_path_ver =
      conf_.driver_param().decoder_param().angle_path_ver();
  decoder_param.angle_path_hor =
      conf_.driver_param().decoder_param().angle_path_hor();
  decoder_param.imu_param_path =
      conf_.driver_param().decoder_param().imu_param_path();
  decoder_param.query_via_external_interface_enable =
      conf_.driver_param()
          .decoder_param()
          .query_via_external_interface_enable();
  decoder_param.point_cloud_enable =
      conf_.driver_param().decoder_param().point_cloud_enable();
  decoder_param.imu_enable = conf_.driver_param().decoder_param().imu_enable();
  decoder_param.imu_orientation_enable =
      conf_.driver_param().decoder_param().imu_orientation_enable();
  decoder_param.laser_scan_enable =
      conf_.driver_param().decoder_param().laser_scan_enable();
  decoder_param.device_ctrl_state_enable =
      conf_.driver_param().decoder_param().device_ctrl_state_enable();
  decoder_param.device_ctrl_cmd_enable =
      conf_.driver_param().decoder_param().device_ctrl_cmd_enable();
  decoder_param.send_packet_enable =
      conf_.driver_param().decoder_param().send_packet_enable();
  decoder_param.recv_packet_enable =
      conf_.driver_param().decoder_param().recv_packet_enable();
  decoder_param.send_lidar_param_enable =
      conf_.driver_param().decoder_param().send_lidar_param_enable();
  decoder_param.recv_lidar_param_cmd_enable =
      conf_.driver_param().decoder_param().recv_lidar_param_cmd_enable();
  decoder_param.tail_filter_enable =
      conf_.driver_param().decoder_param().tail_filter_enable();
  decoder_param.hide_points_range =
      conf_.driver_param().decoder_param().hide_points_range();

  ::vanjee::lidar::WJTransfromParam transform_param;
  if (conf_.driver_param().decoder_param().has_transform_param()) {
    transform_param.x =
        conf_.driver_param().decoder_param().transform_param().x();
    transform_param.y =
        conf_.driver_param().decoder_param().transform_param().y();
    transform_param.z =
        conf_.driver_param().decoder_param().transform_param().z();
    transform_param.roll =
        conf_.driver_param().decoder_param().transform_param().roll();
    transform_param.pitch =
        conf_.driver_param().decoder_param().transform_param().pitch();
    transform_param.yaw =
        conf_.driver_param().decoder_param().transform_param().yaw();
    transform_param.x_imu =
        conf_.driver_param().decoder_param().transform_param().x_imu();
    transform_param.y_imu =
        conf_.driver_param().decoder_param().transform_param().y_imu();
    transform_param.z_imu =
        conf_.driver_param().decoder_param().transform_param().z_imu();
    transform_param.roll_imu =
        conf_.driver_param().decoder_param().transform_param().roll_imu();
    transform_param.pitch_imu =
        conf_.driver_param().decoder_param().transform_param().pitch_imu();
    transform_param.yaw_imu =
        conf_.driver_param().decoder_param().transform_param().yaw_imu();
    decoder_param.transform_param = transform_param;
  }

  ::vanjee::lidar::WJInputParam input_param;
  input_param.connect_type = conf_.driver_param().input_param().connect_type();
  input_param.host_msop_port =
      conf_.driver_param().input_param().host_msop_port();
  input_param.lidar_msop_port =
      conf_.driver_param().input_param().lidar_msop_port();
  input_param.difop_port = conf_.driver_param().input_param().difop_port();
  input_param.host_address = conf_.driver_param().input_param().host_address();
  input_param.lidar_address =
      conf_.driver_param().input_param().lidar_address();
  input_param.group_address =
      conf_.driver_param().input_param().group_address();
  input_param.pcap_path = conf_.driver_param().input_param().pcap_path();
  input_param.pcap_repeat = conf_.driver_param().input_param().pcap_repeat();
  input_param.pcap_rate = conf_.driver_param().input_param().pcap_rate();
  input_param.use_vlan = conf_.driver_param().input_param().use_vlan();
  input_param.user_layer_bytes =
      conf_.driver_param().input_param().user_layer_bytes();
  input_param.tail_layer_bytes =
      conf_.driver_param().input_param().tail_layer_bytes();
  input_param.port_name = conf_.driver_param().input_param().port_name();
  input_param.baud_rate = conf_.driver_param().input_param().baud_rate();
  input_param.network_interface =
      conf_.driver_param().input_param().network_interface();

  ::vanjee::lidar::WJDriverParam driver_param;

  driver_param.input_param = input_param;
  driver_param.decoder_param = decoder_param;

  driver_param.lidar_type =
      ::vanjee::lidar::strToLidarType(conf_.driver_param().lidar_type());
  if (conf_.driver_param().has_input_type()) {
    driver_param.input_type =
        static_cast<InputType>(conf_.driver_param().input_type());
  } else {
    // use the common config
    if (conf_.config_base().source_type() ==
        LidarConfigBase_SourceType_RAW_PACKET) {
      driver_param.input_type = InputType::RAW_PACKET;
    } else if (conf_.config_base().source_type() ==
               LidarConfigBase_SourceType_ONLINE_LIDAR) {
      driver_param.input_type = InputType::ONLINE_LIDAR;
    }
  }

  driver_ptr_->regScanDataCallback(
      std::bind(&VanjeelidarComponent::VanjeeScanDataAllocateCallback, this),
      std::bind(&VanjeelidarComponent::VanjeeScanDataPutCallback, this,
                std::placeholders::_1));

  driver_ptr_->regImuPacketCallback(
      std::bind(&VanjeelidarComponent::VanjeeImuPacketAllocateCallback, this),
      std::bind(&VanjeelidarComponent::VanjeeImuPacketPutCallback, this,
                std::placeholders::_1));

  driver_ptr_->regPointCloudCallback(
      std::bind(&VanjeelidarComponent::VanjeeCloudAllocateCallback, this),
      std::bind(&VanjeelidarComponent::VanjeeCloudPutCallback, this,
                std::placeholders::_1));
  driver_ptr_->regDeviceCtrlCallback(
      std::bind(&VanjeelidarComponent::VanjeeDeviceCtrlAllocateCallback, this),
      std::bind(&VanjeelidarComponent::VanjeeDeviceCtrlPutCallback, this,
                std::placeholders::_1));
  driver_ptr_->regExceptionCallback([](const ::vanjee::lidar::Error& code) {
    WJ_WARNING << code.toString() << WJ_REND;
  });

  driver_ptr_->regPacketCallback(
      std::bind(&VanjeelidarComponent::VanjeePacketCallback, this,
                std::placeholders::_1));

  static ::vanjee::lidar::LidarParameterInterface g_lidar_param;
  driver_ptr_->regLidarParameterInterfaceCallback(
      []() {
        // TODO(All): implement lidar parameter interface allocate callback if
        // necessary
        return std::shared_ptr<::vanjee::lidar::LidarParameterInterface>(
            &g_lidar_param, [](::vanjee::lidar::LidarParameterInterface*) {});
      },
      [](std::shared_ptr<::vanjee::lidar::LidarParameterInterface>
             lidar_param) {
        // TODO(All): implement lidar parameter interface put callback if
        // necessary
        (void)lidar_param;
      });

  AINFO << "vanjee driver version: "
        << ::vanjee::lidar::getDriverVersion().str();
  driver_param.print();

  if (!driver_ptr_->init(driver_param)) {
    AERROR << "vanjee Driver init failed";
    return false;
  }

  if (!driver_ptr_->start()) {
    AERROR << "vanjee Driver start failed";
    return false;
  }
  AINFO << "vanjee lidar init finished";
  return true;
}

void VanjeelidarComponent::ReadScanCallback(
    const std::shared_ptr<vanjee::VanjeePacket>& packet) {
  ADEBUG << __FUNCTION__ << " start";
  std::vector<uint8_t> pkt_data;
  pkt_data.assign(packet->data().begin(), packet->data().end());
  std::shared_ptr<::vanjee::lidar::Packet> pkt =
      std::make_shared<::vanjee::lidar::Packet>(pkt_data.size());
  pkt->seq = packet->seq();
  pkt->timestamp = packet->timestamp();
  pkt->buf = pkt_data;
  driver_ptr_->decodePacket(*pkt);
}

void VanjeelidarComponent::VanjeePacketCallback(
    std::shared_ptr<::vanjee::lidar::Packet> pkt) {
  ADEBUG << __FUNCTION__ << " start";
  std::shared_ptr<vanjee::VanjeePacket> packet =
      std::make_shared<vanjee::VanjeePacket>();
  packet->set_timestamp(pkt->timestamp);
  packet->set_seq(pkt->seq);
  packet->mutable_data()->assign(pkt->buf.begin(), pkt->buf.end());
  WriteScan(packet);
}

std::shared_ptr<::vanjee::lidar::ScanData>
VanjeelidarComponent::VanjeeScanDataAllocateCallback() {
  return scan_buffer_->AllocateElement();
}
void VanjeelidarComponent::VanjeeScanDataPutCallback(
    std::shared_ptr<::vanjee::lidar::ScanData> scan_data) {
  scan_queue_.push(scan_data);
}

std::shared_ptr<::vanjee::lidar::ImuPacket>
VanjeelidarComponent::VanjeeImuPacketAllocateCallback() {
  return imu_buffer_->AllocateElement();
}
void VanjeelidarComponent::VanjeeImuPacketPutCallback(
    std::shared_ptr<::vanjee::lidar::ImuPacket> imu_packet) {
  imu_queue_.push(imu_packet);
}

std::shared_ptr<PointCloudMsg>
VanjeelidarComponent::VanjeeCloudAllocateCallback() {
  return cloud_buffer_->AllocateElement();
}

void VanjeelidarComponent::VanjeeCloudPutCallback(
    std::shared_ptr<PointCloudMsg> vanjee_cloud) {
  cloud_queue_.push(vanjee_cloud);
}

std::shared_ptr<::vanjee::lidar::DeviceCtrl>
VanjeelidarComponent::VanjeeDeviceCtrlAllocateCallback() {
  return device_ctrl_buffer_->AllocateElement();
}
void VanjeelidarComponent::VanjeeDeviceCtrlPutCallback(
    std::shared_ptr<::vanjee::lidar::DeviceCtrl> device_ctrl) {
  device_ctrl_queue_.push(device_ctrl);
}

void VanjeelidarComponent::PreparePointsMsg(PointCloud& msg) {
  msg.set_height(1);
  msg.set_width(msg.point_size() / msg.height());

  const auto timestamp =
      msg.point(static_cast<int>(msg.point_size()) - 1).timestamp();
  msg.set_measurement_time(
      GetSecondTimestampFromNanosecondTimestamp(timestamp));
  double lidar_time = GetSecondTimestampFromNanosecondTimestamp(timestamp);
  double diff_time = msg.header().timestamp_sec() - lidar_time;

  if (diff_time > 0.02) {
    AINFO << std::fixed << std::setprecision(16)
          << "system time: " << msg.header().timestamp_sec()
          << ", lidar time: " << lidar_time << ", diff is:" << diff_time;
  }

  msg.mutable_header()->set_lidar_timestamp(timestamp);
}

void VanjeelidarComponent::ProcessCloud() {
  while (!cyber::IsShutdown()) {
    std::shared_ptr<PointCloudMsg> msg = cloud_queue_.popWait();
    if (msg.get() == NULL) {
      AWARN << "VanjeeCloudPutCallback get nullptr, continue";
      continue;
    }
    auto apollo_pc = AllocatePointCloud();

    for (auto p : msg->points) {
      PointXYZIT* point = apollo_pc->add_point();
      point->set_x(p.x);
      point->set_y(p.y);
      point->set_z(p.z);
      point->set_intensity(static_cast<uint32_t>(p.intensity));
      point->set_timestamp(GetNanosecondTimestampFromSecondTimestamp(
          p.timestamp + msg->timestamp));
    }
    apollo_pc->set_is_dense(msg->is_dense);
    this->PreparePointsMsg(*apollo_pc);
    ADEBUG << "ProcessCloud: apollo_pc point size: " << apollo_pc->point_size();
    if (apollo_pc->point_size() != 0) {
      WritePointCloud(apollo_pc);
    }
  }
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
