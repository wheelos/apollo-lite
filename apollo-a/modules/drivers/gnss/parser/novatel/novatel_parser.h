// Copyright 2024 daohu527@gmail.com
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

//  Created Date: 2024-12-30
//  Author: daohu527

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "third_party/rtklib/rtklib.h"
// rtklib.h define lock, conflicts with the standard library, so this macro
// needs to be removed
#undef lock

#include "modules/common_msgs/sensor_msgs/gnss.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"

#include "modules/drivers/gnss/parser/novatel/novatel_messages.h"
#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class NovatelParser : public Parser {
 public:
  NovatelParser();
  explicit NovatelParser(const config::Config& config);
  ~NovatelParser() = default;

  // Override base class pure virtual methods for state machine logic
  bool ProcessHeader() override;
  std::optional<std::vector<Parser::ParsedMessage>> ProcessPayload() override;

 private:
  // Helper function to check CRC - takes data pointer and size for the *full*
  // message block
  bool CheckCRC(std::string_view message_view);

  // Helper function to dispatch message parsing based on ID
  // Takes payload data/size and header data, populates internal members, and
  // returns messages.
  std::vector<Parser::ParsedMessage> PrepareMessage(
      std::string_view payload_view, std::string_view header_view);

  // The handle_xxx functions - Now take a pointer to the target Protobuf object
  // to populate. They return bool indicating successful data
  // extraction/validation for this specific message type.
  bool HandleBestPos(const novatel::BestPos* pos, uint16_t gps_week,
                     uint32_t gps_millisecs);

  bool HandleGnssBestpos(const novatel::BestPos* pos, uint16_t gps_week,
                         uint32_t gps_millisecs);

  bool HandleBestVel(const novatel::BestVel* vel, uint16_t gps_week,
                     uint32_t gps_millisecs);

  bool HandleCorrImuData(const novatel::CorrImuData* imu);

  // Handles INS covariance, updates the INS message member
  bool HandleInsCov(const novatel::InsCov* cov);

  bool HandleInsPva(const novatel::InsPva* pva);

  // Handles INS PVA with extended status, populates InsStat message member
  bool HandleInsPvax(const novatel::InsPvaX* pvax, uint16_t gps_week,
                     uint32_t gps_millisecs);

  // Handles Raw IMU data (IMU measurements/integrals)
  bool HandleRawImuX(const novatel::RawImuX* imu);

  bool HandleRawImu(const novatel::RawImu* imu);

  // Handles Ephemeris data
  bool HandleBdsEph(const novatel::BDS_Ephemeris* bds_emph);

  bool HandleGpsEph(const novatel::GPS_Ephemeris* gps_emph);

  bool HandleGloEph(const novatel::GLO_Ephemeris* glo_emph);

  // Helper for setting observation time based on RTKLIB structure
  void SetObservationTime();

  // Decodes raw observation data using RTKLIB functionality
  // Takes data pointer and size of the observation payload, and target message
  // pointer
  bool DecodeGnssObservation(const uint8_t* obs_data,
                             const uint8_t* obs_data_end);

  bool HandleHeading(const novatel::Heading* heading, uint16_t gps_week,
                     uint32_t gps_millisecs);

 private:
  // The offset within the buffer where our search for a sync sequence should
  // resume. This helps avoid re-scanning the same garbage data repeatedly.
  size_t search_start_offset_ = 0;

  // Store total message length and header length determined in ProcessHeader
  size_t total_length_ = 0;
  size_t header_length_ = 0;

  // IMU configuration and state
  config::ImuType imu_type_ = config::ImuType::ADIS16488;
  double gyro_scale_ = 0.0;
  double accel_scale_ = 0.0;
  float imu_measurement_span_ = 0.0f;  // Initialized on first IMU message
  float imu_measurement_hz_ = 0.0f;    // Initialized on first IMU message
  int imu_frame_mapping_ = 5;          // Default frame mapping (RFU to FLU)
  double imu_measurement_time_previous_ = -1.0;

  // GNSS/INS status/type state variables (used for logging changes)
  novatel::SolutionStatus solution_status_ = novatel::SolutionStatus::NONE;
  novatel::SolutionType position_type_ = novatel::SolutionType::NONE;
  novatel::SolutionType velocity_type_ = novatel::SolutionType::NONE;
  novatel::InsStatus ins_status_ = novatel::InsStatus::NONE;

  // RTKLIB raw observation data structure
  raw_t raw_;

  ::apollo::drivers::gnss::Gnss gnss_;  // For combined position/velocity
  ::apollo::drivers::gnss::GnssBestPose bestpos_;
  ::apollo::drivers::gnss::Imu imu_;  // For RAWIMU/RAWIMUX
  ::apollo::drivers::gnss::Ins ins_;  // For CORRIMUDATA, INSPVA, INSCOV
  ::apollo::drivers::gnss::InsStat ins_stat_;              // For INSPVAX
  ::apollo::drivers::gnss::GnssEphemeris gnss_ephemeris_;  // For Ephemerides
  ::apollo::drivers::gnss::EpochObservation gnss_observation_;
  ::apollo::drivers::gnss::Heading heading_;  // For HEADING

  DISALLOW_COPY_AND_ASSIGN(NovatelParser);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
