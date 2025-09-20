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

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// #define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <proj.h>
// #include <proj_api.h>

#include "modules/common_msgs/localization_msgs/gps.pb.h"
#include "modules/common_msgs/localization_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/transform/transform_broadcaster.h"

namespace apollo {
namespace drivers {
namespace gnss {

class DataParser {
 public:
  // MessageMap is used to store raw byte messages like GPGGA
  // uint64_t is used as a timestamp (in nanoseconds) for the message,
  // std::vector<uint8_t> is used to hold the raw data.
  // Note: TryGetMessage is marked thread unsafe. Access to this map
  // might need synchronization depending on usage context.
  using MessageMap =
      std::unordered_map<Parser::MessageType,
                         std::pair<uint64_t, std::vector<uint8_t>>>;

  DataParser(const config::Config& config,
             const std::shared_ptr<apollo::cyber::Node>& node);

  // Destructor to release PROJ resources
  ~DataParser();

  // Initializes the parser and writers. Returns true on success.
  bool Init();

  // Parses raw data string. Assumes msg contains data from the GNSS/IMU device.
  void ParseRawData(const std::string& msg);

  // Get the parsed raw message data for a specific type.
  // Currently only used for raw GPGGA data.
  // Thread unsafe - direct access to message_map_.
  [[nodiscard]] std::optional<std::pair<uint64_t, std::vector<uint8_t>>>
  TryGetMessage(const Parser::MessageType& type) const {
    if (auto it = message_map_.find(type); it != message_map_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

 private:
  // Dispatches parsed Protobuf messages to the appropriate handling function.
  void DispatchMessage(Parser::MessageType type,
                       const Parser::ProtoMessagePtr& msg_ptr);

  // Publishes specific message types. Using const& or passing by value/rvalue
  // where possible to avoid unnecessary shared_ptr creation for messages
  // not requiring shared ownership within this class.
  void PublishInsStat(const Parser::ProtoMessagePtr& msg_ptr);
  void PublishOdometry(const Parser::ProtoMessagePtr& msg_ptr);
  void PublishCorrimu(const Parser::ProtoMessagePtr& msg_ptr);
  void PublishImu(const Parser::ProtoMessagePtr& msg_ptr);
  void PublishBestpos(const Parser::ProtoMessagePtr& msg_ptr);
  void PublishEphemeris(const Parser::ProtoMessagePtr& msg_ptr);
  void PublishObservation(const Parser::ProtoMessagePtr& msg_ptr);
  void PublishHeading(const Parser::ProtoMessagePtr& msg_ptr);

  // Checks and updates internal status based on Ins message.
  void CheckInsStatus(const std::shared_ptr<Ins>& ins);

  // Checks and updates internal status based on Gnss message.
  void CheckGnssStatus(const std::shared_ptr<Gnss>& gnss);

  // Converts Gps message to TransformStamped for TF broadcasting.
  void GpsToTransformStamped(
      const std::shared_ptr<apollo::localization::Gps>& gps,
      apollo::transform::TransformStamped* transform);

  bool init_flag_ = false;
  config::Config config_;
  // Use gnss parser to parse data, which will call different hardware parsers
  std::unique_ptr<Parser> gnss_parser_;
  apollo::transform::TransformBroadcaster tf_broadcaster_;

  GnssStatus gnss_status_;
  InsStatus ins_status_;
  uint32_t ins_status_record_ = static_cast<uint32_t>(0);

  PJ_CONTEXT* proj_context_;
  PJ* proj_transform_;

  // Map to store raw messages (currently only GPGGA). See comments for
  // MessageMap type.
  MessageMap message_map_;

  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;

  // Cyber writers for various message types.
  // Initialized in Init(). Use shared_ptr as Cyber Writers are shared
  // resources.
  std::shared_ptr<apollo::cyber::Writer<GnssStatus>> gnssstatus_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<InsStatus>> insstatus_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<GnssBestPose>> gnssbestpose_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::CorrectedImu>>
      corrimu_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<Imu>> rawimu_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::localization::Gps>>
      gps_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<InsStat>> insstat_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<GnssEphemeris>> gnssephemeris_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Writer<EpochObservation>>
      epochobservation_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<Heading>> heading_writer_ = nullptr;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
