/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/gnss/parser/rtcm_parser.h"

#include <memory>

#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm3/rtcm3_parser.h"
#include "modules/drivers/gnss/util/util.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::drivers::gnss::EpochObservation;
using ::apollo::drivers::gnss::GnssEphemeris;

RtcmParser::RtcmParser(const config::Config& config,
                       const std::shared_ptr<apollo::cyber::Node>& node)
    : config_(config), node_(node) {}

bool RtcmParser::Init() {
  rtcm_parser_.reset(new Rtcm3Parser(true));

  if (!rtcm_parser_) {
    AERROR << "Failed to create rtcm parser.";
    return false;
  }

  gnssephemeris_writer_ =
      node_->CreateWriter<GnssEphemeris>(FLAGS_gnss_rtk_eph_topic);
  epochobservation_writer_ =
      node_->CreateWriter<EpochObservation>(FLAGS_gnss_rtk_obs_topic);
  init_flag_ = true;
  return true;
}

void RtcmParser::ParseRtcmData(const std::string& msg) {
  if (!init_flag_) {
    AWARN << "RtcmParser not initialized.";
    return;
  }

  rtcm_parser_->AppendData(msg);
  auto messages = rtcm_parser_->ParseAllMessages();

  for (auto& [msg_type, msg_variant] : messages) {
    if (std::holds_alternative<Parser::ProtoMessagePtr>(msg_variant)) {
      DispatchMessage(msg_type, std::get<Parser::ProtoMessagePtr>(msg_variant));
    } else {
      AERROR << "Unknown message type variant.";
    }
  }
}

void RtcmParser::DispatchMessage(Parser::MessageType type,
                                 const Parser::ProtoMessagePtr& msg_ptr) {
  switch (type) {
    case Parser::MessageType::EPHEMERIDES:
      PublishEphemeris(msg_ptr);
      break;

    case Parser::MessageType::OBSERVATION:
      PublishObservation(msg_ptr);
      break;

    default:
      AWARN << "Unhandled RTCM message type: " << static_cast<int>(type);
      break;
  }
}

void RtcmParser::PublishEphemeris(const Parser::ProtoMessagePtr& msg_ptr) {
  auto eph = std::dynamic_pointer_cast<GnssEphemeris>(msg_ptr);
  if (!eph) {
    AERROR << "Failed to cast Message to GnssEphemeris";
    return;
  }
  gnssephemeris_writer_->Write(eph);
}

void RtcmParser::PublishObservation(const Parser::ProtoMessagePtr& msg_ptr) {
  auto observation = std::dynamic_pointer_cast<EpochObservation>(msg_ptr);
  if (!observation) {
    AERROR << "Failed to cast Message to EpochObservation";
    return;
  }
  epochobservation_writer_->Write(observation);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
