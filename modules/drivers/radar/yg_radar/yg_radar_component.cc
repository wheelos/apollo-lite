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
#include "modules/drivers/radar/yg_radar/yg_radar_component.h"

// #include "modules/drivers/radar/yg_radar/proto/ultrasonic_radar_config.pb.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"

namespace apollo {
namespace drivers {
namespace yg_radar {

YgRadarComponent::YgRadarComponent() {}

YgRadarComponent::~YgRadarComponent() {
  can_receiver_.Stop();
  can_client_->Stop();
}

bool YgRadarComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AERROR << "unable to load yg radar config file: " << ConfigFilePath();
    return false;
  }
  AINFO << "the yg radar config file is loaded: " << ConfigFilePath();
  ADEBUG << "yg radar config: " << config_.ShortDebugString();

  writer_ = node_->CreateWriter<apollo::drivers::RadarObstacles>(
      config_.radar_channel());

  auto can_factory = apollo::drivers::canbus::CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(config_.can_card_parameter());
  if (!can_client_) {
    AERROR << "failed to create can client for yg radar.";
    return false;
  }
  AINFO << "can client is created successfully.";
  message_manager_.reset(new YgRadarMessageManager());
  if (message_manager_ == nullptr) {
    AERROR << "failed to create message manager for yg radar.";
    return false;
  }
  message_manager_->set_message_id_offset(config_.can_message_id_offset());
  message_manager_->set_writer(writer_);
  AINFO << "yg radar message manager is created successfully.";
  if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                         config_.enable_receiver_log()) !=
      apollo::common::ErrorCode::OK) {
    AERROR << "failed to initialize can receiver for yg radar.";
    return false;
  }
  AINFO << "can receiver is initialized successfully.";

  if (can_client_->Start() != apollo::common::ErrorCode::OK) {
    AERROR << "failed to start can client for yg radar.";
    return false;
  }

  if (can_receiver_.Start() != apollo::common::ErrorCode::OK) {
    AERROR << "failed to start can receiver for yg radar.";
    return false;
  }

  return true;
}

bool YgRadarComponent::Proc() { return true; }

}  // namespace yg_radar
}  // namespace drivers
}  // namespace apollo
