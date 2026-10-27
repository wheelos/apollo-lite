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

#include <memory>
#include <optional>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "third_party/rtklib/rtklib.h"
// rtklib.h define lock, conflicts with the standard library, so this macro
// needs to be removed
#undef lock

#include "wheelos_msgs/sensor_msgs/gnss.pb.h"
#include "wheelos_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "wheelos_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "wheelos_msgs/sensor_msgs/heading.pb.h"
#include "wheelos_msgs/sensor_msgs/imu.pb.h"
#include "wheelos_msgs/sensor_msgs/ins.pb.h"

#include "modules/drivers/gnss/parser/novatel/novatel_messages.h"
#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Protocol boundary only: RawStream owns transport/receiver commands, this
// class owns framing/CRC/wire units, and DataParser owns projection, Unix-time
// headers and Cyber publication. One accepted log produces a fresh snapshot;
// PVA, raw IMU and corrected IMU are not paired or used as mutual fallbacks.
//
// Supported payloads and downstream outputs (topic suffixes are defaults):
//   BESTGNSSPOS 1429         -> BEST_GNSS_POS (best_pose) + GNSS (gnss_status).
//   BESTPOS/PSRPOS 42/47     -> GNSS position/status only, not best_pose.
//   BESTGNSSVEL/BESTVEL/
//     PSRVEL 1430/99/100     -> GNSS velocity only; no fabricated position.
//   INSPVA(S) 507/508        -> INS (odometry and ins_status).
//   INSPVAX 1465             -> INS_STAT (ins_stat), not odometry.
//   RAWIMU/S/X/SX
//     268/325/1461/1462      -> IMU (imu).
//   CORRIMUDATA(S) 812/813,
//     IMURATECORRIMUS 1362   -> CORRECTED_IMU (corrected_imu), not odometry.
//   INSCOV(S) 264/320        -> cache only: attached to same-epoch PVA if the
//                              covariance arrived first; never replays PVA.
//   HEADING 971              -> HEADING (heading), dual-antenna baseline.
//   RANGE 43                 -> OBSERVATION (rtk_obs), GPS/BDS/GLO bands
//                              supported by the RTKLIB adapter.
//   GPS/BDS/GLO ephemerides  -> *EPHEMERIDES (rtk_eph), broadcast orbit data.
//                              GLO reference/frame times normalized to GPS,
//                              matching the RTCM3 producer.
// Other logs, including modern CORRIMUS, MARK1PVA and GPGGA, are not decoded.
// BESTVEL snapshots remain internal: no standalone velocity topic is emitted;
// odometry velocity comes from PVA. gnss_status follows position logs only.
// ins_status is emitted on a state change or the next PVA after >1 s of wall
// time, not by a fixed-rate timer. These two status headers use host time;
// IMU/PVA/best_pose/heading measurement headers use GPS-to-Unix event time.
//
// Rates are controlled by receiver LOG commands, not this parser. Raw counts
// are converted with the model's native rate; CORRIMUDATA(S) increments use
// corrimu_rate_hz = 1 / LOG ONTIME period. IMURATECORRIMUS uses the native
// rate. Enable only one corrected-IMU log family. No arrival-time rate
// estimation, resampling or gap filling is performed. Duplicate/backward
// raw/corrected IMU epochs are rejected; recreate the parser after a
// receiver/replay time reset.
//
// measurement_time is GPS-epoch seconds, from the payload where available
// (header time otherwise). It is not host receive time or Unix time. Raw output
// is FLU specific force [m/s^2, including gravity] and angular rate [rad/s];
// measurement_span is one native sample even after a gap. imu_orientation is
// a discrete enclosure-to-FLU mapping, not arbitrary mounting calibration.
// Corrected IMU has gravity/Earth rotation/estimated biases removed; DataParser
// converts its internal FLU vectors back to legacy RFU for corrected_imu.
// Do not substitute it for raw specific force in a LIO frontend.
//
// PVA is LLH (ellipsoid height), ENU velocity and (roll,-pitch,90-azimuth)
// radians. Odometry retains the legacy RFU-to-ENU quaternion. Consumers must
// gate odometry with ins_status; a PVA event is not itself a validity
// guarantee. Covariance retains the legacy computational-axis mapping;
// firmware-specific covariance axes and mounting/lever arms still require
// device verification. RAWIMUX's coarse error byte is checked; model-specific
// imuStatus bits and receiver time_status are not a complete
// sensor-health/time-quality filter.
class NovatelParser : public Parser {
 public:
  NovatelParser();
  explicit NovatelParser(const config::Config& config);
  ~NovatelParser() override;

  bool ProcessHeader() override;
  std::optional<std::vector<Parser::ParsedMessage>> ProcessPayload() override;

 private:
  // CRC covers header + payload, excluding the trailing CRC itself.
  bool CheckCRC(std::string_view message_view);

  // Dispatch a CRC-validated full frame.
  std::vector<Parser::ParsedMessage> PrepareMessage(
      std::string_view message_view);

  // Decode wire payloads into independent message snapshots.
  bool HandleBestPos(const novatel::BestPos* pos, uint16_t gps_week,
                     uint32_t gps_millisecs);

  bool HandleGnssBestpos(const novatel::BestPos* pos, uint16_t gps_week,
                         uint32_t gps_millisecs);

  bool HandleBestVel(const novatel::BestVel* vel, uint16_t gps_week,
                     uint32_t gps_millisecs);

  bool HandleCorrImuData(const novatel::CorrImuData* imu,
                         novatel::MessageId message_id);

  // Cache covariance for a PVA with the same measurement epoch.
  bool HandleInsCov(const novatel::InsCov* cov);

  bool HandleInsPva(const novatel::InsPva* pva);

  // INSPVAX is intentionally a status output, not an alternative PVA source.
  bool HandleInsPvax(const novatel::InsPvaX* pvax, uint16_t gps_week,
                     uint32_t gps_millisecs);

  // Handles Raw IMU data (IMU measurements/integrals)
  bool HandleRawImuX(const novatel::RawImuX* imu);

  bool HandleRawImu(const novatel::RawImu* imu);

  bool DecodeRawImu(const novatel::RawImu& imu, config::ImuType type);

  // Handles Ephemeris data
  bool HandleBdsEph(const novatel::BDS_Ephemeris* bds_emph);

  bool HandleGpsEph(const novatel::GPS_Ephemeris* gps_emph);

  bool HandleGloEph(const novatel::GLO_Ephemeris* glo_emph);

  void SetObservationTime();

  // RTKLIB consumes complete OEM frames, including header and CRC.
  bool DecodeGnssObservation(const uint8_t* obs_data,
                             const uint8_t* obs_data_end);

  bool HandleHeading(const novatel::Heading* heading, uint16_t gps_week,
                     uint32_t gps_millisecs);

 private:
  // Store total message length and header length determined in ProcessHeader
  size_t total_length_ = 0;
  size_t header_length_ = 0;

  // IMU configuration and state
  config::ImuType imu_type_ = config::ImuType::ADIS16488;
  bool has_configured_imu_type_ = false;
  std::optional<config::ImuType> detected_imu_type_;
  double corrimu_rate_hz_ = 0.0;
  int imu_frame_mapping_ = 5;
  // Reject duplicate/backward IMU epochs. Recreate the parser on a time reset.
  double imu_measurement_time_previous_ = -1.0;
  double corrimu_time_previous_ = -1.0;
  std::optional<novatel::InsCov> ins_cov_;

  // Used only to log state changes, never to merge measurement payloads.
  novatel::SolutionStatus solution_status_ = novatel::SolutionStatus::NONE;
  novatel::SolutionType position_type_ = novatel::SolutionType::NONE;
  novatel::SolutionType velocity_type_ = novatel::SolutionType::NONE;
  novatel::InsStatus ins_status_ = novatel::InsStatus::NONE;

  // RTKLIB raw observation data structure
  raw_t raw_;

  ::apollo::drivers::gnss::Gnss
      gnss_;  // Position or velocity, never stale pairs.
  ::apollo::drivers::gnss::GnssBestPose bestpos_;
  ::apollo::drivers::gnss::Imu imu_;  // For RAWIMU/RAWIMUX
  ::apollo::drivers::gnss::Ins ins_;  // PVA only; never carries stale IMU data.
  ::apollo::drivers::gnss::Ins corrimu_;
  ::apollo::drivers::gnss::InsStat ins_stat_;              // For INSPVAX
  ::apollo::drivers::gnss::GnssEphemeris gnss_ephemeris_;  // For Ephemerides
  ::apollo::drivers::gnss::EpochObservation gnss_observation_;
  ::apollo::drivers::gnss::Heading heading_;  // For HEADING

  DISALLOW_COPY_AND_ASSIGN(NovatelParser);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
