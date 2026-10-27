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

#include "modules/drivers/gnss/parser/novatel/novatel_parser.h"

#include <cmath>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <type_traits>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/util/time_conversion.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/util/util.h"

namespace apollo {
namespace drivers {
namespace gnss {

using apollo::common::util::GpsToUnixSeconds;

namespace {

static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "NovAtel wire structures require a little-endian host");

template <typename T>
T ReadWire(std::string_view bytes) {
  static_assert(std::is_trivially_copyable_v<T>);
  T value;
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}

// Zero denotes a variable-size or unsupported log. Validate known fixed sizes
// before waiting for a payload, so a corrupt length cannot stall the stream.
size_t FixedPayloadSize(uint16_t id) {
  switch (id) {
    case novatel::BESTGNSSPOS:
    case novatel::BESTPOS:
    case novatel::PSRPOS:
      return sizeof(novatel::BestPos);
    case novatel::BESTGNSSVEL:
    case novatel::BESTVEL:
    case novatel::PSRVEL:
      return sizeof(novatel::BestVel);
    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
    case novatel::IMURATECORRIMUS:
      return sizeof(novatel::CorrImuData);
    case novatel::INSCOV:
    case novatel::INSCOVS:
      return sizeof(novatel::InsCov);
    case novatel::INSPVA:
    case novatel::INSPVAS:
      return sizeof(novatel::InsPva);
    case novatel::RAWIMU:
    case novatel::RAWIMUS:
      return sizeof(novatel::RawImu);
    case novatel::RAWIMUX:
    case novatel::RAWIMUSX:
      return sizeof(novatel::RawImuX);
    case novatel::INSPVAX:
      return sizeof(novatel::InsPvaX);
    case novatel::HEADING:
      return sizeof(novatel::Heading);
    case novatel::BDSEPHEMERIS:
      return sizeof(novatel::BDS_Ephemeris);
    case novatel::GPSEPHEMERIS:
      return sizeof(novatel::GPS_Ephemeris);
    case novatel::GLOEPHEMERIS:
      return sizeof(novatel::GLO_Ephemeris);
    default:
      return 0;
  }
}

bool ValidTime(uint32_t week, double seconds) {
  if (week == 0 || week > UINT16_MAX || !std::isfinite(seconds) ||
      seconds < 0 || seconds >= kSecondsPerWeek) {
    AERROR << "Invalid NovAtel GPS time: " << week << ", " << seconds;
    return false;
  }
  return true;
}

bool ValidStdDev(double value) { return std::isfinite(value) && value >= 0.0; }

bool ValidPosition(double latitude, double longitude, double height) {
  if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
      !std::isfinite(height) || std::abs(latitude) > 90.0 ||
      std::abs(longitude) > 180.0) {
    AERROR << "Invalid NovAtel position";
    return false;
  }
  return true;
}

bool ValidBestPosition(const novatel::BestPos& pos) {
  return ValidPosition(pos.latitude, pos.longitude, pos.height_msl) &&
         std::isfinite(pos.undulation) &&
         pos.datum_id == novatel::DatumId::WGS84 &&
         ValidStdDev(pos.latitude_std_dev) &&
         ValidStdDev(pos.longitude_std_dev) && ValidStdDev(pos.height_std_dev);
}

bool AllFinite(std::initializer_list<double> values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

std::optional<config::ImuType> WireImuType(uint8_t type) {
  // Do not cast OEM identifiers to the configuration enum: vendor extensions
  // share numeric values with different OEM7 IMUs.
  switch (type) {
    case 13:
      return config::IMAR_FSAS;
    case 26:
      return config::ISA100C;
    case 31:
      return config::ADIS16488;
    case 32:
      return config::STIM300;
    case 34:
      return config::ISA100;
    case 38:
      return config::ISA100_400HZ;
    case 39:
      return config::ISA100C_400HZ;
    case 41:
      return config::G320N;
    default:
      return std::nullopt;
  }
}

}  // namespace

NovatelParser::NovatelParser() : NovatelParser(config::Config()) {}

NovatelParser::NovatelParser(const config::Config& config) : Parser() {
  has_configured_imu_type_ = config.has_imu_type();
  if (config.has_imu_type()) {
    imu_type_ = config.imu_type();
  }
  if (config.has_novatel_config()) {
    imu_frame_mapping_ = config.novatel_config().imu_orientation();
    corrimu_rate_hz_ = config.novatel_config().corrimu_rate_hz();
  }
  if (1 != init_raw(&raw_)) {
    AFATAL << "memory allocation error for observation data structure.";
  }
}

NovatelParser::~NovatelParser() { free_raw(&raw_); }

bool NovatelParser::ProcessHeader() {
  static constexpr char SYNC_PATTERN[] = {static_cast<char>(novatel::SYNC_0),
                                          static_cast<char>(novatel::SYNC_1)};
  static constexpr std::string_view sync_pattern_view(SYNC_PATTERN, 2);

  while (true) {
    auto buffer_view = buffer_.Peek();
    if (buffer_view.length() < sizeof(novatel::ShortHeader)) {
      return false;
    }

    auto pos = buffer_view.find(sync_pattern_view);
    if (pos == std::string_view::npos) {
      // Keep a possible first sync byte spanning two transport reads.
      buffer_.Drain(buffer_view.length() - 1);
      return false;
    }

    if (pos > 0) {
      buffer_.Drain(pos);
      buffer_view = buffer_.Peek();
    }
    if (buffer_view.length() < 4) {
      return false;
    }

    size_t header_len = 0;
    if (static_cast<uint8_t>(buffer_view[2]) == novatel::SYNC_2_LONG_HEADER) {
      header_len = static_cast<uint8_t>(buffer_view[3]);
      if (header_len != sizeof(novatel::LongHeader)) {
        AERROR << "Unsupported NovAtel header length: " << header_len;
        buffer_.Drain(1);
        continue;
      }
    } else if (static_cast<uint8_t>(buffer_view[2]) ==
               novatel::SYNC_2_SHORT_HEADER) {
      header_len = sizeof(novatel::ShortHeader);
    }

    if (header_len == 0) {
      AERROR << "Invalid 3rd sync byte after a SYNC_0, SYNC_1 match. "
                "Discarding one byte and retrying.";
      buffer_.Drain(1);
      continue;
    }

    if (buffer_view.length() < header_len) {
      return false;
    }

    const uint16_t message_length =
        header_len == sizeof(novatel::LongHeader)
            ? ReadWire<novatel::LongHeader>(buffer_view).message_length
            : ReadWire<novatel::ShortHeader>(buffer_view).message_length;
    const uint16_t id = static_cast<uint8_t>(buffer_view[4]) |
                        (static_cast<uint8_t>(buffer_view[5]) << 8);
    const size_t expected_size = FixedPayloadSize(id);
    if ((expected_size != 0 && message_length != expected_size) ||
        (id == novatel::RANGE &&
         (message_length < 4 ||
          header_len + message_length + novatel::CRC_LENGTH > MAXRAWLEN))) {
      AERROR << "Invalid NovAtel payload length for " << id << ": "
             << message_length;
      buffer_.Drain(1);
      continue;
    }

    header_length_ = header_len;
    total_length_ = header_len + message_length + novatel::CRC_LENGTH;
    return true;
  }
}

std::optional<std::vector<Parser::ParsedMessage>>
NovatelParser::ProcessPayload() {
  if (buffer_.ReadableBytes() < total_length_) {
    return std::nullopt;
  }

  auto message_view = buffer_.Peek().substr(0, total_length_);

  if (!CheckCRC(message_view)) {
    AERROR << "Novatel message CRC check failed. Resynchronizing.";
    // A real sync can start inside a corrupt candidate header.
    buffer_.Drain(1);

    total_length_ = 0;
    header_length_ = 0;
    return std::vector<Parser::ParsedMessage>();
  }

  auto parsed_messages = PrepareMessage(message_view);

  buffer_.Drain(total_length_);

  total_length_ = 0;
  header_length_ = 0;

  return parsed_messages;
}

bool NovatelParser::CheckCRC(std::string_view message_view) {
  if (message_view.length() < novatel::CRC_LENGTH) {
    return false;
  }
  size_t data_len = message_view.length() - novatel::CRC_LENGTH;
  uint32_t expected_crc;
  std::memcpy(&expected_crc, message_view.data() + data_len,
              sizeof(expected_crc));
  const uint32_t actual_crc = crc32_block(
      reinterpret_cast<const uint8_t*>(message_view.data()), data_len);

  return actual_crc == expected_crc;
}

std::vector<Parser::ParsedMessage> NovatelParser::PrepareMessage(
    std::string_view message_view) {
  std::vector<Parser::ParsedMessage> messages;
  const auto header_view = message_view.substr(0, header_length_);
  const auto payload_view =
      message_view.substr(header_length_, message_view.size() - header_length_ -
                                              novatel::CRC_LENGTH);

  novatel::MessageId message_id;
  uint16_t gps_week;
  uint32_t gps_millisecs;

  if (header_view.length() == sizeof(novatel::LongHeader)) {
    const auto header = ReadWire<novatel::LongHeader>(header_view);
    message_id = header.message_id;
    gps_week = header.gps_week;
    gps_millisecs = header.gps_millisecs;
    if (header.message_type.response !=
            novatel::MessageType::ORIGINAL_MESSAGE ||
        header.message_type.format != novatel::MessageType::BINARY) {
      AERROR << "Ignoring NovAtel response or non-binary log";
      return messages;
    }
  } else if (header_view.length() == sizeof(novatel::ShortHeader)) {
    const auto header = ReadWire<novatel::ShortHeader>(header_view);
    message_id = header.message_id;
    gps_week = header.gps_week;
    gps_millisecs = header.gps_millisecs;
  } else {
    AERROR << "PrepareMessage called with invalid header length: "
           << header_view.length();
    return messages;
  }

  if (!ValidTime(gps_week, gps_millisecs * 1e-3)) {
    return messages;
  }
  const size_t payload_size = payload_view.size();
  const auto emit = [&messages](MessageType type, const auto& message) {
    messages.emplace_back(
        type, std::make_shared<std::decay_t<decltype(message)>>(message));
  };

  // ProcessHeader has validated every fixed-size payload. Copy wire bytes into
  // real objects instead of aliasing an arbitrarily aligned transport buffer.
  switch (message_id) {
    case novatel::BESTGNSSPOS: {
      const auto pos = ReadWire<novatel::BestPos>(payload_view);
      if (HandleGnssBestpos(&pos, gps_week, gps_millisecs)) {
        emit(MessageType::BEST_GNSS_POS, bestpos_);
      }
      if (HandleBestPos(&pos, gps_week, gps_millisecs)) {
        emit(MessageType::GNSS, gnss_);
      }
      break;
    }
    case novatel::BESTPOS:
    case novatel::PSRPOS: {
      const auto pos = ReadWire<novatel::BestPos>(payload_view);
      if (HandleBestPos(&pos, gps_week, gps_millisecs)) {
        emit(MessageType::GNSS, gnss_);
      }
      break;
    }
    case novatel::BESTGNSSVEL:
    case novatel::BESTVEL:
    case novatel::PSRVEL: {
      const auto vel = ReadWire<novatel::BestVel>(payload_view);
      if (HandleBestVel(&vel, gps_week, gps_millisecs)) {
        emit(MessageType::GNSS, gnss_);
      }
      break;
    }
    case novatel::CORRIMUDATA:
    case novatel::CORRIMUDATAS:
    case novatel::IMURATECORRIMUS: {
      const auto imu = ReadWire<novatel::CorrImuData>(payload_view);
      if (HandleCorrImuData(&imu, message_id)) {
        emit(MessageType::CORRECTED_IMU, corrimu_);
      }
      break;
    }
    case novatel::INSCOV:
    case novatel::INSCOVS: {
      const auto cov = ReadWire<novatel::InsCov>(payload_view);
      HandleInsCov(&cov);
      break;
    }
    case novatel::INSPVA:
    case novatel::INSPVAS: {
      const auto pva = ReadWire<novatel::InsPva>(payload_view);
      if (HandleInsPva(&pva)) {
        emit(MessageType::INS, ins_);
      }
      break;
    }
    case novatel::RAWIMUX:
    case novatel::RAWIMUSX: {
      const auto imu = ReadWire<novatel::RawImuX>(payload_view);
      if (HandleRawImuX(&imu)) {
        emit(MessageType::IMU, imu_);
      }
      break;
    }
    case novatel::RAWIMU:
    case novatel::RAWIMUS: {
      const auto imu = ReadWire<novatel::RawImu>(payload_view);
      if (HandleRawImu(&imu)) {
        emit(MessageType::IMU, imu_);
      }
      break;
    }
    case novatel::INSPVAX: {
      const auto pvax = ReadWire<novatel::InsPvaX>(payload_view);
      if (HandleInsPvax(&pvax, gps_week, gps_millisecs)) {
        emit(MessageType::INS_STAT, ins_stat_);
      }
      break;
    }
    case novatel::BDSEPHEMERIS: {
      const auto eph = ReadWire<novatel::BDS_Ephemeris>(payload_view);
      if (HandleBdsEph(&eph)) {
        emit(MessageType::BDSEPHEMERIDES, gnss_ephemeris_);
      }
      break;
    }
    case novatel::GPSEPHEMERIS: {
      const auto eph = ReadWire<novatel::GPS_Ephemeris>(payload_view);
      if (HandleGpsEph(&eph)) {
        emit(MessageType::GPSEPHEMERIDES, gnss_ephemeris_);
      }
      break;
    }
    case novatel::GLOEPHEMERIS: {
      const auto eph = ReadWire<novatel::GLO_Ephemeris>(payload_view);
      if (HandleGloEph(&eph)) {
        emit(MessageType::GLOEPHEMERIDES, gnss_ephemeris_);
      }
      break;
    }
    case novatel::RANGE:
      if (header_length_ != sizeof(novatel::LongHeader)) {
        AERROR << "RTKLIB requires a long-header RANGE log";
        break;
      }
      {
        uint32_t count;
        std::memcpy(&count, payload_view.data(), sizeof(count));
        if ((payload_size - 4) % 44 != 0 || count != (payload_size - 4) / 44) {
          AERROR << "Invalid RANGE observation count: " << count;
          break;
        }
      }
      if (DecodeGnssObservation(
              reinterpret_cast<const uint8_t*>(message_view.data()),
              reinterpret_cast<const uint8_t*>(message_view.data()) +
                  message_view.size())) {
        emit(MessageType::OBSERVATION, gnss_observation_);
      }
      break;

    case novatel::HEADING: {
      const auto heading = ReadWire<novatel::Heading>(payload_view);
      if (HandleHeading(&heading, gps_week, gps_millisecs)) {
        emit(MessageType::HEADING, heading_);
      }
      break;
    }
    default:
      ADEBUG << "Unsupported Novatel message ID: "
             << static_cast<int>(message_id)
             << ". Payload size: " << payload_size;
      break;
  }

  return messages;
}

bool NovatelParser::HandleGnssBestpos(const novatel::BestPos* pos,
                                      uint16_t gps_week,
                                      uint32_t gps_millisecs) {
  if (!ValidBestPosition(*pos) ||
      !SolutionStatus_IsValid(static_cast<int>(pos->solution_status)) ||
      !SolutionType_IsValid(static_cast<int>(pos->position_type))) {
    AERROR << "Invalid or unsupported BESTGNSSPOS solution";
    return false;
  }
  bestpos_.Clear();
  bestpos_.set_sol_status(
      static_cast<apollo::drivers::gnss::SolutionStatus>(pos->solution_status));
  bestpos_.set_sol_type(
      static_cast<apollo::drivers::gnss::SolutionType>(pos->position_type));
  bestpos_.set_latitude(pos->latitude);
  bestpos_.set_longitude(pos->longitude);
  bestpos_.set_height_msl(pos->height_msl);
  bestpos_.set_undulation(pos->undulation);
  bestpos_.set_datum_id(
      static_cast<apollo::drivers::gnss::DatumId>(pos->datum_id));
  bestpos_.set_latitude_std_dev(pos->latitude_std_dev);
  bestpos_.set_longitude_std_dev(pos->longitude_std_dev);
  bestpos_.set_height_std_dev(pos->height_std_dev);
  bestpos_.set_base_station_id(pos->base_station_id,
                               sizeof(pos->base_station_id));
  bestpos_.set_differential_age(pos->differential_age);
  bestpos_.set_solution_age(pos->solution_age);
  bestpos_.set_num_sats_tracked(pos->num_sats_tracked);
  bestpos_.set_num_sats_in_solution(pos->num_sats_in_solution);
  bestpos_.set_num_sats_l1(pos->num_sats_l1);
  bestpos_.set_num_sats_multi(pos->num_sats_multi);
  bestpos_.set_reserved(pos->reserved);
  bestpos_.set_extended_solution_status(pos->extended_solution_status);
  bestpos_.set_galileo_beidou_used_mask(pos->galileo_beidou_used_mask);
  bestpos_.set_gps_glonass_used_mask(pos->gps_glonass_used_mask);

  double seconds =
      static_cast<double>(gps_week) * kSecondsPerWeek + gps_millisecs * 1e-3;
  bestpos_.set_measurement_time(seconds);

  return true;
}

bool NovatelParser::HandleBestPos(const novatel::BestPos* pos,
                                  uint16_t gps_week, uint32_t gps_millisecs) {
  gnss_.Clear();
  if (pos->solution_status == novatel::SolutionStatus::SOL_COMPUTED) {
    if (ValidBestPosition(*pos)) {
      gnss_.mutable_position()->set_lon(pos->longitude);
      gnss_.mutable_position()->set_lat(pos->latitude);
      gnss_.mutable_position()->set_height(pos->height_msl + pos->undulation);
      gnss_.mutable_position_std_dev()->set_x(pos->longitude_std_dev);
      gnss_.mutable_position_std_dev()->set_y(pos->latitude_std_dev);
      gnss_.mutable_position_std_dev()->set_z(pos->height_std_dev);
    } else {
      AERROR << "Invalid BESTPOS position or unsupported datum";
    }
  }
  gnss_.set_num_sats(pos->num_sats_in_solution);
  if (solution_status_ != pos->solution_status) {
    solution_status_ = pos->solution_status;
    AINFO << "Solution status: " << static_cast<int>(solution_status_);
  }
  if (position_type_ != pos->position_type) {
    position_type_ = pos->position_type;
    AINFO << "Position type: " << static_cast<int>(position_type_);
  }
  gnss_.set_solution_status(static_cast<uint32_t>(pos->solution_status));
  if (pos->solution_status == novatel::SolutionStatus::SOL_COMPUTED) {
    gnss_.set_position_type(static_cast<uint32_t>(pos->position_type));
    switch (pos->position_type) {
      case novatel::SolutionType::SINGLE:
      case novatel::SolutionType::INS_PSRSP:
        gnss_.set_type(apollo::drivers::gnss::Gnss::SINGLE);
        break;
      case novatel::SolutionType::PSRDIFF:
      case novatel::SolutionType::WAAS:
      case novatel::SolutionType::INS_SBAS:
        gnss_.set_type(apollo::drivers::gnss::Gnss::PSRDIFF);
        break;
      case novatel::SolutionType::FLOATCONV:
      case novatel::SolutionType::L1_FLOAT:
      case novatel::SolutionType::IONOFREE_FLOAT:
      case novatel::SolutionType::NARROW_FLOAT:
      case novatel::SolutionType::RTK_DIRECT_INS:
      case novatel::SolutionType::INS_RTKFLOAT:
        gnss_.set_type(apollo::drivers::gnss::Gnss::RTK_FLOAT);
        break;
      case novatel::SolutionType::WIDELANE:
      case novatel::SolutionType::NARROWLANE:
      case novatel::SolutionType::L1_INT:
      case novatel::SolutionType::WIDE_INT:
      case novatel::SolutionType::NARROW_INT:
      case novatel::SolutionType::INS_RTKFIXED:
        gnss_.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
        break;
      case novatel::SolutionType::OMNISTAR:
      case novatel::SolutionType::INS_OMNISTAR:
      case novatel::SolutionType::INS_OMNISTAR_HP:
      case novatel::SolutionType::INS_OMNISTAR_XP:
      case novatel::SolutionType::OMNISTAR_HP:
      case novatel::SolutionType::OMNISTAR_XP:
      case novatel::SolutionType::PPP_CONVERGING:
      case novatel::SolutionType::PPP:
      case novatel::SolutionType::INS_PPP_CONVERGING:
      case novatel::SolutionType::INS_PPP:
        gnss_.set_type(apollo::drivers::gnss::Gnss::PPP);
        break;
      default:
        gnss_.set_type(apollo::drivers::gnss::Gnss::INVALID);
    }
  } else {
    gnss_.set_type(apollo::drivers::gnss::Gnss::INVALID);
    gnss_.set_position_type(0);
  }
  if (!gnss_.has_position()) {
    gnss_.set_type(Gnss::INVALID);
  }
  double seconds =
      static_cast<double>(gps_week) * kSecondsPerWeek + gps_millisecs * 1e-3;
  gnss_.set_measurement_time(seconds);
  return true;
}

bool NovatelParser::HandleBestVel(const novatel::BestVel* vel,
                                  uint16_t gps_week, uint32_t gps_millisecs) {
  gnss_.Clear();
  if (vel->solution_status != novatel::SolutionStatus::SOL_COMPUTED ||
      !std::isfinite(vel->horizontal_speed) ||
      !std::isfinite(vel->track_over_ground) ||
      !std::isfinite(vel->vertical_speed) || !std::isfinite(vel->latency) ||
      vel->latency < 0.0 || vel->horizontal_speed < 0.0) {
    AWARN << "Ignoring invalid NovAtel velocity solution";
    return false;
  }
  if (velocity_type_ != vel->velocity_type) {
    velocity_type_ = vel->velocity_type;
    AINFO << "Velocity type: " << static_cast<int>(velocity_type_);
  }
  gnss_.set_velocity_latency(vel->latency);
  double yaw = azimuth_deg_to_yaw_rad(vel->track_over_ground);
  gnss_.mutable_linear_velocity()->set_x(vel->horizontal_speed * cos(yaw));
  gnss_.mutable_linear_velocity()->set_y(vel->horizontal_speed * sin(yaw));
  gnss_.mutable_linear_velocity()->set_z(vel->vertical_speed);

  double seconds =
      static_cast<double>(gps_week) * kSecondsPerWeek + gps_millisecs * 1e-3;
  gnss_.set_measurement_time(seconds);
  return true;
}

bool NovatelParser::HandleCorrImuData(const novatel::CorrImuData* imu,
                                      novatel::MessageId message_id) {
  if (!ValidTime(imu->gps_week, imu->gps_seconds)) {
    return false;
  }
  double rate = corrimu_rate_hz_;
  if (message_id == novatel::IMURATECORRIMUS) {
    if (!detected_imu_type_ && !has_configured_imu_type_) {
      AERROR << "IMURATECORRIMUS requires a known IMU type";
      return false;
    }
    rate = novatel::GetImuParameter(detected_imu_type_.value_or(imu_type_))
               .sampling_rate_hz;
  }
  if (!std::isfinite(rate) || rate <= 0.0) {
    AERROR << "CORRIMUDATA(S) requires its LOG rate in "
              "novatel_config.corrimu_rate_hz";
    return false;
  }
  const double seconds =
      static_cast<double>(imu->gps_week) * kSecondsPerWeek + imu->gps_seconds;
  if (seconds <= corrimu_time_previous_) {
    AERROR << "Non-increasing corrected IMU timestamp";
    return false;
  }
  const double values[] = {imu->x_velocity_change, imu->y_velocity_change,
                           imu->z_velocity_change, imu->x_angle_change,
                           imu->y_angle_change,    imu->z_angle_change};
  for (double value : values) {
    if (!std::isfinite(value * rate)) {
      AERROR << "Non-finite corrected IMU measurement";
      return false;
    }
  }
  corrimu_.Clear();
  z_rot_90_ccw(imu->x_velocity_change * rate, imu->y_velocity_change * rate,
               imu->z_velocity_change * rate,
               corrimu_.mutable_linear_acceleration());
  z_rot_90_ccw(imu->x_angle_change * rate, imu->y_angle_change * rate,
               imu->z_angle_change * rate, corrimu_.mutable_angular_velocity());
  corrimu_.set_measurement_time(seconds);
  corrimu_time_previous_ = seconds;
  return true;
}

bool NovatelParser::HandleInsCov(const novatel::InsCov* cov) {
  if (!ValidTime(cov->gps_week, cov->gps_seconds)) {
    return false;
  }
  for (int i = 0; i < 9; ++i) {
    constexpr double limit = std::numeric_limits<float>::max();
    if (!std::isfinite(cov->position_covariance[i]) ||
        !std::isfinite(cov->attitude_covariance[i]) ||
        !std::isfinite(cov->velocity_covariance[i]) ||
        std::abs(cov->position_covariance[i]) > limit ||
        std::abs(cov->attitude_covariance[i]) > limit ||
        std::abs(cov->velocity_covariance[i]) > limit ||
        (i % 4 == 0 && (cov->position_covariance[i] < 0.0 ||
                        cov->attitude_covariance[i] < 0.0 ||
                        cov->velocity_covariance[i] < 0.0))) {
      AERROR << "Invalid or unrepresentable INS covariance";
      ins_cov_.reset();
      return false;
    }
  }
  ins_cov_ = *cov;
  return true;
}

bool NovatelParser::HandleInsPva(const novatel::InsPva* pva) {
  if (!ValidTime(pva->gps_week, pva->gps_seconds) ||
      !ValidPosition(pva->latitude, pva->longitude, pva->height)) {
    return false;
  }
  const double values[] = {pva->roll,          pva->pitch,
                           pva->azimuth,       pva->north_velocity,
                           pva->east_velocity, pva->up_velocity};
  for (double value : values) {
    if (!std::isfinite(value)) {
      AERROR << "Non-finite INS PVA measurement";
      return false;
    }
  }
  ins_.Clear();
  if (ins_status_ != pva->status) {
    ins_status_ = pva->status;
    AINFO << "INS status: " << static_cast<int>(ins_status_);
  }
  ins_.mutable_position()->set_lon(pva->longitude);
  ins_.mutable_position()->set_lat(pva->latitude);
  ins_.mutable_position()->set_height(pva->height);
  ins_.mutable_euler_angles()->set_x(pva->roll * kDegToRad);
  ins_.mutable_euler_angles()->set_y(-pva->pitch * kDegToRad);
  ins_.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(pva->azimuth));
  ins_.mutable_linear_velocity()->set_x(pva->east_velocity);
  ins_.mutable_linear_velocity()->set_y(pva->north_velocity);
  ins_.mutable_linear_velocity()->set_z(pva->up_velocity);

  switch (pva->status) {
    case novatel::InsStatus::ALIGNMENT_COMPLETE:
    case novatel::InsStatus::SOLUTION_GOOD:
      ins_.set_type(apollo::drivers::gnss::Ins::GOOD);
      break;
    case novatel::InsStatus::ALIGNING:
    case novatel::InsStatus::HIGH_VARIANCE:
    case novatel::InsStatus::SOLUTION_FREE:
      ins_.set_type(apollo::drivers::gnss::Ins::CONVERGING);
      break;
    default:
      ins_.set_type(apollo::drivers::gnss::Ins::INVALID);
  }

  double seconds =
      static_cast<double>(pva->gps_week) * kSecondsPerWeek + pva->gps_seconds;
  ins_.set_measurement_time(seconds);
  // Never label a covariance from another epoch as belonging to this PVA.
  if (ins_cov_ && ins_cov_->gps_week == pva->gps_week &&
      std::abs(ins_cov_->gps_seconds - pva->gps_seconds) < 1e-6) {
    constexpr int axes[] = {1, 0, 2};
    constexpr double signs[] = {1.0, -1.0, -1.0};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        const int source = axes[row] * 3 + axes[col];
        const int index = row * 3 + col;
        ins_.add_position_covariance(ins_cov_->position_covariance[index]);
        ins_.add_linear_velocity_covariance(
            ins_cov_->velocity_covariance[index]);
        ins_.add_euler_angles_covariance(signs[row] * signs[col] * kDegToRad *
                                         kDegToRad *
                                         ins_cov_->attitude_covariance[source]);
      }
    }
  }
  return true;
}

bool NovatelParser::HandleInsPvax(const novatel::InsPvaX* pvax,
                                  uint16_t gps_week, uint32_t gps_millisecs) {
  double seconds =
      static_cast<double>(gps_week) * kSecondsPerWeek + gps_millisecs * 1e-3;
  double unix_sec = GpsToUnixSeconds(seconds);
  ins_stat_.mutable_header()->set_timestamp_sec(unix_sec);
  ins_stat_.set_ins_status(pvax->ins_status);
  ins_stat_.set_pos_type(pvax->pos_type);
  return true;
}

bool NovatelParser::HandleRawImuX(const novatel::RawImuX* imu) {
  if (imu->imu_error != 0) {
    AERROR << "Rejecting IMU error/info flags: "
           << static_cast<int>(imu->imu_error)
           << ", status: " << imu->imuStatus;
    return false;
  }
  const auto type = WireImuType(imu->imu_type);
  if (!type || (has_configured_imu_type_ && *type != imu_type_)) {
    AERROR << "Unsupported or conflicting wire IMU type: "
           << static_cast<int>(imu->imu_type);
    return false;
  }
  novatel::RawImu raw;
  raw.gps_week = imu->gps_week;
  raw.gps_seconds = imu->gps_seconds;
  raw.imuStatus = imu->imuStatus;
  raw.z_velocity_change = imu->z_velocity_change;
  raw.y_velocity_change_neg = imu->y_velocity_change_neg;
  raw.x_velocity_change = imu->x_velocity_change;
  raw.z_angle_change = imu->z_angle_change;
  raw.y_angle_change_neg = imu->y_angle_change_neg;
  raw.x_angle_change = imu->x_angle_change;
  if (!DecodeRawImu(raw, *type)) {
    return false;
  }
  detected_imu_type_ = type;
  return true;
}

bool NovatelParser::HandleRawImu(const novatel::RawImu* imu) {
  if (!has_configured_imu_type_) {
    AERROR << "RAWIMU(S) requires an explicit imu_type";
    return false;
  }
  return DecodeRawImu(*imu, imu_type_);
}

bool NovatelParser::DecodeRawImu(const novatel::RawImu& imu,
                                 config::ImuType type) {
  const auto param = novatel::GetImuParameter(type);
  if (!ValidTime(imu.gps_week, imu.gps_seconds) ||
      param.sampling_rate_hz <= 0.0) {
    AERROR << "Invalid raw IMU time or unsupported type: " << type;
    return false;
  }
  if (imu_frame_mapping_ != 5 && imu_frame_mapping_ != 6) {
    AERROR << "Unsupported IMU frame mapping: " << imu_frame_mapping_;
    return false;
  }
  const double gyro_scale = param.gyro_scale * param.sampling_rate_hz;
  const double accel_scale = param.accel_scale * param.sampling_rate_hz;
  const double imu_measurement_span = 1.0 / param.sampling_rate_hz;
  const double time =
      static_cast<double>(imu.gps_week) * kSecondsPerWeek + imu.gps_seconds;
  if (time <= imu_measurement_time_previous_) {
    AERROR << "Non-increasing raw IMU timestamp: " << time;
    return false;
  }
  if (imu_measurement_time_previous_ > 0.0 &&
      fabs(time - imu_measurement_time_previous_ - imu_measurement_span) >
          1e-4) {
    AWARN << "Unexpected delay between two IMU measurements at: "
          << time - imu_measurement_time_previous_;
  }

  imu_.Clear();
  imu_.set_measurement_time(time);
  imu_.set_measurement_span(imu_measurement_span);
  switch (imu_frame_mapping_) {
    case 5:  // Default mapping.
      z_rot_90_ccw(
          imu.x_velocity_change * accel_scale,
          -static_cast<double>(imu.y_velocity_change_neg) * accel_scale,
          imu.z_velocity_change * accel_scale,
          imu_.mutable_linear_acceleration());
      z_rot_90_ccw(imu.x_angle_change * gyro_scale,
                   -static_cast<double>(imu.y_angle_change_neg) * gyro_scale,
                   imu.z_angle_change * gyro_scale,
                   imu_.mutable_angular_velocity());
      break;
    case 6:
      z_rot_90_ccw(
          -static_cast<double>(imu.y_velocity_change_neg) * accel_scale,
          imu.x_velocity_change * accel_scale,
          -static_cast<double>(imu.z_velocity_change) * accel_scale,
          imu_.mutable_linear_acceleration());
      z_rot_90_ccw(-static_cast<double>(imu.y_angle_change_neg) * gyro_scale,
                   imu.x_angle_change * gyro_scale,
                   -static_cast<double>(imu.z_angle_change) * gyro_scale,
                   imu_.mutable_angular_velocity());
      break;
    default:
      return false;
  }
  imu_measurement_time_previous_ = time;
  return true;
}

bool NovatelParser::HandleGpsEph(const novatel::GPS_Ephemeris* gps_emph) {
  if (!AllFinite({gps_emph->A, gps_emph->delta_A, gps_emph->M_0, gps_emph->ecc,
                  gps_emph->omega, gps_emph->cuc, gps_emph->cus, gps_emph->crc,
                  gps_emph->crs, gps_emph->cic, gps_emph->cis, gps_emph->I_0,
                  gps_emph->dot_I, gps_emph->omega_0, gps_emph->dot_omega,
                  gps_emph->tgd, gps_emph->af0, gps_emph->af1,
                  gps_emph->af2}) ||
      !ValidTime(gps_emph->week, gps_emph->toe) ||
      !ValidTime(gps_emph->week, gps_emph->toc) || gps_emph->A <= 0.0 ||
      gps_emph->ecc < 0.0 || gps_emph->ecc >= 1.0 ||
      !ValidStdDev(gps_emph->ura) ||
      std::sqrt(gps_emph->ura) > std::numeric_limits<uint32_t>::max()) {
    AERROR << "Invalid GPS ephemeris";
    return false;
  }
  gnss_ephemeris_.Clear();
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      gnss_ephemeris_.mutable_keppler_orbit();

  keppler_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::GPS_SYS);
  keppler_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::GPS_TIME);
  keppler_orbit->set_sat_prn(gps_emph->prn);
  keppler_orbit->set_week_num(gps_emph->week);
  keppler_orbit->set_af0(gps_emph->af0);
  keppler_orbit->set_af1(gps_emph->af1);
  keppler_orbit->set_af2(gps_emph->af2);
  keppler_orbit->set_iode(gps_emph->iode1);
  keppler_orbit->set_deltan(gps_emph->delta_A);
  keppler_orbit->set_m0(gps_emph->M_0);
  keppler_orbit->set_e(gps_emph->ecc);
  keppler_orbit->set_roota(sqrt(gps_emph->A));
  keppler_orbit->set_toe(gps_emph->toe);
  keppler_orbit->set_toc(gps_emph->toc);
  keppler_orbit->set_cic(gps_emph->cic);
  keppler_orbit->set_crc(gps_emph->crc);
  keppler_orbit->set_cis(gps_emph->cis);
  keppler_orbit->set_crs(gps_emph->crs);
  keppler_orbit->set_cuc(gps_emph->cuc);
  keppler_orbit->set_cus(gps_emph->cus);
  keppler_orbit->set_omega0(gps_emph->omega_0);
  keppler_orbit->set_omega(gps_emph->omega);
  keppler_orbit->set_i0(gps_emph->I_0);
  keppler_orbit->set_omegadot(gps_emph->dot_omega);
  keppler_orbit->set_idot(gps_emph->dot_I);
  keppler_orbit->set_accuracy(static_cast<unsigned int>(sqrt(gps_emph->ura)));
  keppler_orbit->set_health(gps_emph->health);
  keppler_orbit->set_tgd(gps_emph->tgd);
  keppler_orbit->set_iodc(gps_emph->iodc);
  return true;
}

bool NovatelParser::HandleBdsEph(const novatel::BDS_Ephemeris* bds_emph) {
  if (!AllFinite({bds_emph->a0, bds_emph->a1, bds_emph->a2, bds_emph->rootA,
                  bds_emph->ecc, bds_emph->omega, bds_emph->delta_N,
                  bds_emph->M0, bds_emph->omega0, bds_emph->rra,
                  bds_emph->inc_angle, bds_emph->idot, bds_emph->cuc,
                  bds_emph->cus, bds_emph->crc, bds_emph->crs, bds_emph->cic,
                  bds_emph->cis, bds_emph->tdg1}) ||
      bds_emph->rootA <= 0.0 || bds_emph->ecc < 0.0 || bds_emph->ecc >= 1.0 ||
      bds_emph->toe >= kSecondsPerWeek || bds_emph->toc >= kSecondsPerWeek ||
      !ValidStdDev(bds_emph->ura) ||
      bds_emph->ura > std::numeric_limits<uint32_t>::max()) {
    AERROR << "Invalid BDS ephemeris";
    return false;
  }
  gnss_ephemeris_.Clear();
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::BDS_SYS);

  apollo::drivers::gnss::KepplerOrbit* keppler_orbit =
      gnss_ephemeris_.mutable_keppler_orbit();

  keppler_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::BDS_SYS);
  keppler_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::BDS_TIME);
  keppler_orbit->set_sat_prn(bds_emph->satellite_id);
  keppler_orbit->set_week_num(bds_emph->week);
  keppler_orbit->set_af0(bds_emph->a0);
  keppler_orbit->set_af1(bds_emph->a1);
  keppler_orbit->set_af2(bds_emph->a2);
  keppler_orbit->set_iode(bds_emph->aode);
  keppler_orbit->set_deltan(bds_emph->delta_N);
  keppler_orbit->set_m0(bds_emph->M0);
  keppler_orbit->set_e(bds_emph->ecc);
  keppler_orbit->set_roota(bds_emph->rootA);
  keppler_orbit->set_toe(bds_emph->toe);
  keppler_orbit->set_toc(bds_emph->toc);
  keppler_orbit->set_cic(bds_emph->cic);
  keppler_orbit->set_crc(bds_emph->crc);
  keppler_orbit->set_cis(bds_emph->cis);
  keppler_orbit->set_crs(bds_emph->crs);
  keppler_orbit->set_cuc(bds_emph->cuc);
  keppler_orbit->set_cus(bds_emph->cus);
  keppler_orbit->set_omega0(bds_emph->omega0);
  keppler_orbit->set_omega(bds_emph->omega);
  keppler_orbit->set_i0(bds_emph->inc_angle);
  keppler_orbit->set_omegadot(bds_emph->rra);
  keppler_orbit->set_idot(bds_emph->idot);
  keppler_orbit->set_accuracy(static_cast<unsigned int>(bds_emph->ura));
  keppler_orbit->set_health(bds_emph->health1);
  keppler_orbit->set_tgd(bds_emph->tdg1);
  keppler_orbit->set_iodc(bds_emph->aodc);
  return true;
}

bool NovatelParser::HandleGloEph(const novatel::GLO_Ephemeris* glo_emph) {
  const double seconds = glo_emph->e_time * 1e-3;
  if (!ValidTime(glo_emph->e_week, seconds) || glo_emph->sloto <= 37 ||
      glo_emph->sloto > 37 + MAXPRNGLO || glo_emph->freqo > 20 ||
      glo_emph->Tk >= 86400 || glo_emph->t_offset >= 86400 ||
      !AllFinite({glo_emph->pos_x, glo_emph->pos_y, glo_emph->pos_z,
                  glo_emph->vel_x, glo_emph->vel_y, glo_emph->vel_z,
                  glo_emph->acc_x, glo_emph->acc_y, glo_emph->acc_z,
                  glo_emph->tau_n, glo_emph->gamma})) {
    AERROR << "Invalid GLONASS ephemeris";
    return false;
  }
  // Match RTKLIB decode_gloephemerisb and the RTCM3 producer: reference
  // week/toe and frame time are GPS time, not GLONASS seconds of day.
  double frame_seconds = static_cast<double>(glo_emph->Tk) -
                         glo_emph->t_offset +
                         std::floor(seconds / 86400.0) * 86400.0;
  if (frame_seconds < seconds - 43200.0) {
    frame_seconds += 86400.0;
  } else if (frame_seconds > seconds + 43200.0) {
    frame_seconds -= 86400.0;
  }
  frame_seconds = std::fmod(frame_seconds + kSecondsPerWeek, kSecondsPerWeek);
  gnss_ephemeris_.Clear();
  gnss_ephemeris_.set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);

  apollo::drivers::gnss::GlonassOrbit* glonass_orbit =
      gnss_ephemeris_.mutable_glonass_orbit();
  glonass_orbit->set_gnss_type(apollo::drivers::gnss::GnssType::GLO_SYS);
  glonass_orbit->set_gnss_time_type(
      apollo::drivers::gnss::GnssTimeType::GPS_TIME);
  glonass_orbit->set_slot_prn(glo_emph->sloto - 37);
  glonass_orbit->set_toe(seconds);
  glonass_orbit->set_frequency_no(static_cast<int>(glo_emph->freqo) - 7);
  glonass_orbit->set_week_num(glo_emph->e_week);
  glonass_orbit->set_week_second_s(seconds);
  glonass_orbit->set_tk(frame_seconds);
  glonass_orbit->set_clock_offset(-glo_emph->tau_n);
  glonass_orbit->set_clock_drift(glo_emph->gamma);

  if (glo_emph->health <= 3) {
    glonass_orbit->set_health(0);  // 0 means good.
  } else {
    glonass_orbit->set_health(1);  // 1 means bad.
  }
  glonass_orbit->set_position_x(glo_emph->pos_x);
  glonass_orbit->set_position_y(glo_emph->pos_y);
  glonass_orbit->set_position_z(glo_emph->pos_z);

  glonass_orbit->set_velocity_x(glo_emph->vel_x);
  glonass_orbit->set_velocity_y(glo_emph->vel_y);
  glonass_orbit->set_velocity_z(glo_emph->vel_z);

  glonass_orbit->set_accelerate_x(glo_emph->acc_x);
  glonass_orbit->set_accelerate_y(glo_emph->acc_y);
  glonass_orbit->set_accelerate_z(glo_emph->acc_z);

  glonass_orbit->set_infor_age(glo_emph->age);

  return true;
}

bool NovatelParser::HandleHeading(const novatel::Heading* heading,
                                  uint16_t gps_week, uint32_t gps_millisecs) {
  if (!std::isfinite(heading->length) || heading->length < 0.0 ||
      !std::isfinite(heading->heading) || !std::isfinite(heading->pitch) ||
      !ValidStdDev(heading->heading_std_dev) ||
      !ValidStdDev(heading->pitch_std_dev)) {
    AERROR << "Invalid NovAtel heading measurement";
    return false;
  }
  heading_.Clear();
  heading_.set_solution_status(static_cast<uint32_t>(heading->solution_status));
  heading_.set_position_type(static_cast<uint32_t>(heading->position_type));
  heading_.set_baseline_length(heading->length);
  heading_.set_heading(heading->heading);
  heading_.set_pitch(heading->pitch);
  heading_.set_reserved(heading->reserved);
  heading_.set_heading_std_dev(heading->heading_std_dev);
  heading_.set_pitch_std_dev(heading->pitch_std_dev);
  heading_.set_station_id(heading->station_id, sizeof(heading->station_id));
  heading_.set_satellite_tracked_number(heading->num_sats_tracked);
  heading_.set_satellite_soulution_number(heading->num_sats_in_solution);
  heading_.set_satellite_number_obs(heading->num_sats_ele);
  heading_.set_satellite_number_multi(heading->num_sats_l2);
  heading_.set_solution_source(heading->solution_source);
  heading_.set_extended_solution_status(heading->extended_solution_status);
  heading_.set_galileo_beidou_sig_mask(heading->galileo_beidou_sig_mask);
  heading_.set_gps_glonass_sig_mask(heading->gps_glonass_sig_mask);
  double seconds =
      static_cast<double>(gps_week) * kSecondsPerWeek + gps_millisecs * 1e-3;
  heading_.set_measurement_time(seconds);
  return true;
}

void NovatelParser::SetObservationTime() {
  int week = 0;
  double second = time2gpst(raw_.time, &week);
  gnss_observation_.set_gnss_time_type(apollo::drivers::gnss::GPS_TIME);
  gnss_observation_.set_gnss_week(week);
  gnss_observation_.set_gnss_second_s(second);
}

bool NovatelParser::DecodeGnssObservation(const uint8_t* obs_data,
                                          const uint8_t* obs_data_end) {
  raw_.nbyte = 0;
  // RTKLIB merges logs at the same epoch. Each RANGE output here is a frame
  // snapshot; retain lock history, but never satellites from an earlier log.
  raw_.obs.n = 0;
  while (obs_data < obs_data_end) {
    const int status = input_oem4(&raw_, *obs_data++);
    switch (status) {
      case 1:  // observation data
        if (raw_.obs.n == 0) {
          AWARN << "Obs is zero";
        }

        gnss_observation_.Clear();
        gnss_observation_.set_receiver_id(0);
        SetObservationTime();
        for (int i = 0; i < raw_.obs.n; ++i) {
          int prn = 0;
          int sys = 0;

          sys = satsys(raw_.obs.data[i].sat, &prn);

          apollo::drivers::gnss::GnssType gnss_type;
          if (!gnss_sys_type(sys, &gnss_type)) {
            continue;
          }

          SatelliteObservation* sat_obs = nullptr;
          for (int j = 0; j < NFREQ + NEXOBS; ++j) {
            if (raw_.obs.data[i].code[j] == CODE_NONE) {
              continue;
            }

            apollo::drivers::gnss::GnssBandID baud_id;
            if (!gnss_baud_id(gnss_type, j, &baud_id)) {
              continue;
            }

            if (sat_obs == nullptr) {
              sat_obs = gnss_observation_.add_sat_obs();
              sat_obs->set_sat_prn(prn);
              sat_obs->set_sat_sys(gnss_type);
            }
            auto band_obs = sat_obs->add_band_obs();
            if (raw_.obs.data[i].code[j] == CODE_L1C) {
              band_obs->set_pseudo_type(
                  apollo::drivers::gnss::PseudoType::CORSE_CODE);
            } else if (raw_.obs.data[i].code[j] == CODE_L1P) {
              band_obs->set_pseudo_type(
                  apollo::drivers::gnss::PseudoType::PRECISION_CODE);
            } else {
              ADEBUG << "Code " << raw_.obs.data[i].code[j] << ", in seq " << j
                     << ", gnss type " << static_cast<int>(gnss_type);
            }

            band_obs->set_band_id(baud_id);
            band_obs->set_pseudo_range(raw_.obs.data[i].P[j]);
            band_obs->set_carrier_phase(raw_.obs.data[i].L[j]);
            band_obs->set_loss_lock_index(raw_.obs.data[i].LLI[j]);
            band_obs->set_doppler(raw_.obs.data[i].D[j]);
            band_obs->set_snr(raw_.obs.data[i].SNR[j] * 0.25f);
          }
          if (sat_obs != nullptr) {
            sat_obs->set_band_obs_num(sat_obs->band_obs_size());
          }
        }
        gnss_observation_.set_sat_obs_num(gnss_observation_.sat_obs_size());
        return true;

      case -1:
        AERROR << "RTKLIB rejected a NovAtel RANGE frame";
        return false;
      default:
        break;
    }
  }
  return false;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
