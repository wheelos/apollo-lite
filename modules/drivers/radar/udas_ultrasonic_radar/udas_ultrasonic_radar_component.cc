/******************************************************************************
 * Copyright 2025 The WheelOS Team. All Rights Reserved.
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
#include "modules/drivers/radar/udas_ultrasonic_radar/udas_ultrasonic_radar_component.h"

#include "modules/drivers/radar/udas_ultrasonic_radar/proto/ultrasonic_radar_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"

namespace apollo {
namespace drivers {
namespace ultrasonic_radar {

UdasUltrasonicRadarComponent::UdasUltrasonicRadarComponent() {}

UdasUltrasonicRadarComponent::~UdasUltrasonicRadarComponent() {
  can_receiver_.Stop();
  can_client_->Stop();
}

bool UdasUltrasonicRadarComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "unable to load ultrasonic radar config file: "
           << ConfigFilePath();
    return false;
  }
  AINFO << "the ultrasonic radar config file is loaded: " << ConfigFilePath();
  ADEBUG << "ultrasonic radar config: " << config_.ShortDebugString();
  auto can_factory = apollo::drivers::canbus::CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(config_.can_card_parameter());
  if (!can_client_) {
    AERROR << "failed to create can client for ultrasonic radar.";
    return false;
  }
  AINFO << "can client is created successfully.";
  message_manager_.reset(new UdasUltrasonicRadarMessageManager());
  if (message_manager_ == nullptr) {
    AERROR << "failed to create message manager for ultrasonic radar.";
    return false;
  }
  AINFO << "udas ultrasonic radar message manager is created successfully.";
  if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                         config_.enable_receiver_log()) !=
      apollo::common::ErrorCode::OK) {
    AERROR << "failed to initialize can receiver for ultrasonic radar.";
    return false;
  }
  AINFO << "can receiver is initialized successfully.";

  if (can_client_->Start() != apollo::common::ErrorCode::OK) {
    AERROR << "failed to start can client for ultrasonic radar.";
    return false;
  }

  if (can_receiver_.Start() != apollo::common::ErrorCode::OK) {
    AERROR << "failed to start can receiver for ultrasonic radar.";
    return false;
  }

  // ranges_.resize(config_.entrance_num(), 0);
  writer_ = node_->CreateWriter<apollo::drivers::Ultrasonic>(
      FLAGS_ultrasonic_radar_topic);

  return true;
}

bool UdasUltrasonicRadarComponent::Proc() {
  apollo::drivers::udas_ultrasonic_radar::Udas_ultrasonic_radar message;
  // TODO(Pride): revise it to make it more general for other ultrasonic radars.
  message_manager_->GetSensorData(&message);
  ranges_.clear();
  ranges_.resize(12, 0.0);
  uint32_t idx = 0;
  if (message.has_sensor_dist_f_5c9()) {
    std::vector<double> ranges = {
        static_cast<float>(message.sensor_dist_f_5c9().sensor_1_fa()),
        static_cast<float>(message.sensor_dist_f_5c9().sensor_2_fb()),
        static_cast<float>(message.sensor_dist_f_5c9().sensor_3_fc()),
        static_cast<float>(message.sensor_dist_f_5c9().sensor_4_fd()),
        static_cast<float>(message.sensor_dist_f_5c9().sensor_5_fe()),
        static_cast<float>(message.sensor_dist_f_5c9().sensor_6_ff())};
    for (size_t i = 0; i < ranges.size(); ++i) {
      ranges_[idx++] = ranges[i];
    }
  }
  if (message.has_sensor_dist_r_5c8()) {
    std::vector<double> ranges = {
        static_cast<float>(message.sensor_dist_r_5c8().sensor_1_ra()),
        static_cast<float>(message.sensor_dist_r_5c8().sensor_2_rb()),
        static_cast<float>(message.sensor_dist_r_5c8().sensor_3_rc()),
        static_cast<float>(message.sensor_dist_r_5c8().sensor_4_rd()),
        static_cast<float>(message.sensor_dist_r_5c8().sensor_5_re()),
        static_cast<float>(message.sensor_dist_r_5c8().sensor_6_rf())};
    for (size_t i = 0; i < ranges.size(); ++i) {
      ranges_[idx++] = ranges[i];
    }
  }
  apollo::drivers::Ultrasonic publish_message;
  publish_message.Clear();
  apollo::common::util::FillHeader("ulrtasonic_radar", &publish_message);
  for (size_t i = 0; i < ranges_.size(); ++i) {
    publish_message.add_ranges(ranges_[i]);
  }
  writer_->Write(publish_message);
  return true;
}

}  // namespace ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
