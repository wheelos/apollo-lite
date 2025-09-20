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

#include "modules/drivers/gnss/parser/huace/huace_parser.h"

#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "absl/strings/escaping.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_split.h"

#include "cyber/common/log.h"
#include "modules/common/util/util.h"
#include "modules/drivers/gnss/util/util.h"

namespace {

using apollo::drivers::gnss::SolutionStatus;
using apollo::drivers::gnss::SolutionType;
using apollo::drivers::gnss::huace::GPCHCX;
using apollo::drivers::gnss::huace::SatelliteStatus;
using apollo::drivers::gnss::huace::SystemStatus;

SolutionStatus ToSolutionStatus(SystemStatus sys_status) {
  switch (sys_status) {
    case SystemStatus::INIT:
      // System is in initialization state, most common solution status is not
      // yet converged or waiting for data. PENDING means still computing.
      // COLD_START means just started.
      return SolutionStatus::COLD_START;
      // Alternatively, consider SolutionStatus::PENDING;

    case SystemStatus::GUIDANCE:
      // In guidance mode, if all is normal, a solution should be computed.
      // But in practice, there may be issues. Here we assume the "successful"
      // case.
      return SolutionStatus::SOL_COMPUTED;

    case SystemStatus::COMBINED:
      // In combined navigation mode, the solution is usually reliable and
      // computed. Here we also assume the "successful" case.
      return SolutionStatus::SOL_COMPUTED;

    case SystemStatus::INERTIAL:
      // Pure inertial mode, meaning GNSS signal may be lost or unavailable.
      // SolutionStatus does not have a direct "pure inertial dead reckoning"
      // state. If GNSS data is insufficient, could map to INSUFFICIENT_OBS.
      // More accurately, a new SolutionStatus value may be needed for "dead
      // reckoning, GNSS unavailable". For now, we assume a solution is still
      // "computed" (though accuracy may degrade). Another consideration: if
      // inertial solution quality degrades, COV_TRACE or VARIANCE may occur. To
      // avoid misleading, we choose a "computing but quality not guaranteed"
      // status.
      return SolutionStatus::
          SOL_COMPUTED;  // This mapping is debatable, as it does not reflect
                         // inertial drift. Ideally, SolutionStatus may need to
                         // be extended. For now, use SOL_COMPUTED to indicate a
                         // solution is output, but check other info for
                         // quality.

    default:
      // Catch all unhandled SystemStatus values.
      // This may indicate the system is in an unknown or unauthorized state.
      AWARN << "Unhandled SystemStatus value for SolutionStatus: "
            << static_cast<int>(sys_status);
      return SolutionStatus::UNAUTHORIZED;
  }
}

SolutionType ToSolutionType(SatelliteStatus sat_status) {
  switch (sat_status) {
    case SatelliteStatus::NO_POS_NO_ORIENT:
      // No position or attitude information, closest is NONE
      return SolutionType::NONE;

    case SatelliteStatus::SINGLE_POS_ORIENT:
    case SatelliteStatus::SINGLE_POS_NO_ORIENT:
      // Single point positioning, with or without attitude
      return SolutionType::SINGLE;

    case SatelliteStatus::PSEUDORANGE_DIFF_ORIENT:
    case SatelliteStatus::PSEUDORANGE_DIFF_NO_ORIENT:
      // Pseudorange differential positioning, with or without attitude
      return SolutionType::PSRDIFF;

    case SatelliteStatus::RTK_FLOAT_ORIENT:
      return SolutionType::INS_RTKFLOAT;
    case SatelliteStatus::RTK_FLOAT_NO_ORIENT:
      // RTK float solution, with or without attitude
      // SolutionType has several FLOAT types, here FLOATCONV is generic,
      // or if you know it's L1/IONOFREE/NARROW float, can be more precise.
      // But since SatelliteStatus just says "RTK_FLOAT", NARROW_FLOAT is a
      // reasonable generic representation. return SolutionType::FLOATCONV;
      // Alternatively: if focusing on RTK internal float, can use NARROW_FLOAT,
      // but FLOATCONV is more generic.
      return SolutionType::NARROW_FLOAT;

    case SatelliteStatus::RTK_STABLE_ORIENT:
      return SolutionType::INS_RTKFIXED;
    case SatelliteStatus::RTK_STABLE_NO_ORIENT:
      // RTK fixed solution, with or without attitude
      // SolutionType has several INT (Integer fixed) types, here NARROW_INT
      // represents RTK fixed solution, as RTK fixed is usually narrow-lane
      // fixed. L1_INT and WIDE_INT are also fixed solutions. If no more
      // detailed info, NARROW_INT is a common choice.
      return SolutionType::NARROW_INT;
      // Alternatively: for a more generic fixed solution, can use L1_INT or
      // WIDE_INT, or if there is a "best available fixed solution" concept, but
      // NARROW_INT usually means highest precision RTK fixed. return
      // SolutionType::L1_INT;

    case SatelliteStatus::COMBINED_PREDICTION:
      // This is a special type, may mean some form of prediction (e.g.,
      // inertial prediction) is fused. In SolutionType, PROPOGATED is closest
      // to "predicted". If "Combined Prediction" clearly involves INS, and you
      // want to map to INS-related types, may need to check deeper system
      // status. But from the name, PROPOGATED is most generic.
      return SolutionType::PROPOGATED;

    default:
      AWARN << "Unhandled SatelliteStatus value for SolutionType: "
            << static_cast<int>(sat_status);
      return SolutionType::NONE;
  }
}

template <typename T, typename ParserFunc>
bool ParseStandardField(const std::string& s, const char* field_name,
                        size_t field_index, T& target,
                        ParserFunc simple_absl_parse_func) {
  if (!simple_absl_parse_func(s, &target)) {
    AERROR << "Failed to parse '" << s << "' for field: " << field_name
           << " at index " << field_index << " (expected "
           << typeid(target).name() << ").";
    return false;
  }
  return true;
}

template <typename ParserFunc>
bool ParseStandardField(const std::string& s, const char* field_name,
                        size_t field_index, uint8_t& target,
                        ParserFunc simple_absl_parse_func) {
  uint8_t tmp;
  if (!simple_absl_parse_func(s, &tmp)) {
    AERROR << "Failed to parse '" << s
           << "' as unsigned int for field: " << field_name << " at index "
           << field_index << ".";
    return false;
  }
  target = tmp;
  return true;
}

template <typename ParserFunc>
bool ParseStandardField(const std::string& s, const char* field_name,
                        size_t field_index, std::string& target,
                        ParserFunc /* simple_absl_parse_func */) {
  // For std::string, simply assign the value.
  target = s;
  return true;
}

static const std::vector<apollo::drivers::gnss::HuaceParser::FieldParser>
    parsers = {
        {"gps_week",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "gps_week", 0, out_ptr->gps_week,
                                     [](const std::string& str, uint32_t* val) {
                                       return absl::SimpleAtoi<uint32_t>(str,
                                                                         val);
                                     });
         }},
        {"seconds_in_gps_week",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "seconds_in_gps_week", 1,
                                     out_ptr->seconds_in_gps_week,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"heading",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "heading", 2, out_ptr->heading,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"pitch",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "pitch", 3, out_ptr->pitch,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"roll",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "roll", 4, out_ptr->roll,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"gyro_x",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "gyro_x", 5, out_ptr->gyro_x,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"gyro_y",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "gyro_y", 6, out_ptr->gyro_y,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"gyro_z",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "gyro_z", 7, out_ptr->gyro_z,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"acc_x",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "acc_x", 8, out_ptr->acc_x,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"acc_y",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "acc_y", 9, out_ptr->acc_y,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"acc_z",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "acc_z", 10, out_ptr->acc_z,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"latitude",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(
               s, "latitude", 11, out_ptr->latitude,
               [](const std::string& str, double* val) {
                 // the string may be empty when the
                 // signal is weak
                 if (str.empty()) {
                   *val = std::numeric_limits<double>::quiet_NaN();
                   return true;
                 }
                 return absl::SimpleAtod(str, val);
               });
         }},
        {"longitude",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(
               s, "longitude", 12, out_ptr->longitude,
               [](const std::string& str, double* val) {
                 // the string may be empty when the
                 // signal is weak
                 if (str.empty()) {
                   *val = std::numeric_limits<double>::quiet_NaN();
                   return true;
                 }
                 return absl::SimpleAtod(str, val);
               });
         }},
        {"altitude",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(
               s, "altitude", 13, out_ptr->altitude,
               [](const std::string& str, double* val) {
                 // the string may be empty when the
                 // signal is weak
                 if (str.empty()) {
                   *val = std::numeric_limits<double>::quiet_NaN();
                   return true;
                 }
                 return absl::SimpleAtod(str, val);
               });
         }},
        {"velocity_east",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "velocity_east", 14,
                                     out_ptr->velocity_east,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"velocity_north",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "velocity_north", 15,
                                     out_ptr->velocity_north,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"velocity_up",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "velocity_up", 16, out_ptr->velocity_up,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"speed",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "speed", 17, out_ptr->speed,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"nsv1",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "nsv1", 18, out_ptr->nsv1,
                                     [](const std::string& str, uint32_t* val) {
                                       return absl::SimpleAtoi(str, val);
                                     });
         }},
        {"nsv2",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "nsv2", 19, out_ptr->nsv2,
                                     [](const std::string& str, uint32_t* val) {
                                       return absl::SimpleAtoi(str, val);
                                     });
         }},
        // Status field - special handling for bitfield (assuming
        // status.raw_value is uint8_t)
        {"status",
         [](const std::string& s, GPCHCX* out_ptr) {
           // Direct call to ParseStandardField for uint8_t, which handles
           // conversion and range check.
           return ParseStandardField(
               s, "status", 20, out_ptr->status.raw_value,
               [](const std::string& str, uint8_t* val) {
                 std::string result;
                 if (!absl::HexStringToBytes(str, &result)) {
                   AERROR << "Failed to decode hex string for field 'status'. "
                             "Raw input: "
                          << str;
                   return false;
                 }
                 *val = static_cast<uint8_t>(result[0]);
                 return true;
               });
         }},
        {"differential_age",
         [](const std::string& s, GPCHCX* out_ptr) {
           // THIS IS THE LINE THAT CAUSED THE ERROR
           return ParseStandardField(s, "differential_age", 21,
                                     out_ptr->differential_age,
                                     [](const std::string& str, uint32_t* val) {
                                       return absl::SimpleAtoi(str, val);
                                     });
         }},
        {"warning",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(
               s, "warning", 22, out_ptr->warning,
               [](const std::string& src, std::string* dest) {
                 *dest = src;
                 return true;
               });
         }},
        // --- Fields from GPCHCX structure (latitude_std to device_sn) ---
        {"latitude_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "latitude_std", 23,
                                     out_ptr->latitude_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"longitude_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "longitude_std", 24,
                                     out_ptr->longitude_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"altitude_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "altitude_std", 25,
                                     out_ptr->altitude_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"velocity_east_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "velocity_east_std", 26,
                                     out_ptr->velocity_east_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"velocity_north_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "velocity_north_std", 27,
                                     out_ptr->velocity_north_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"velocity_up_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "velocity_up_std", 28,
                                     out_ptr->velocity_up_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"roll_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "roll_std", 29, out_ptr->roll_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"pitch_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "pitch_std", 30, out_ptr->pitch_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"heading_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "heading_std", 31, out_ptr->heading_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        // Note: 'separator' is char, typically handled by skipping or specific
        // logic.
        // set a null parser for "separator" field, as it is a special field but
        // keep the parsers aligned with the fields
        {"separator", nullptr},
        {"speed_heading",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "speed_heading", 33,
                                     out_ptr->speed_heading,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"speed_heading_std",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "speed_heading_std", 34,
                                     out_ptr->speed_heading_std,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        // Antenna parameters (float types) - use ParseStandardField float
        // specialization
        {"antenna_x",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "antenna_x", 35, out_ptr->antenna_x,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"antenna_y",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "antenna_y", 36, out_ptr->antenna_y,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"antenna_z",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "antenna_z", 37, out_ptr->antenna_z,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"rotation_angle_x",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "rotation_angle_x", 38,
                                     out_ptr->rotation_angle_x,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"rotation_angle_y",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "rotation_angle_y", 39,
                                     out_ptr->rotation_angle_y,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"rotation_angle_z",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "rotation_angle_z", 40,
                                     out_ptr->rotation_angle_z,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"direction_angle",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "direction_angle", 41,
                                     out_ptr->direction_angle,
                                     [](const std::string& str, double* val) {
                                       return absl::SimpleAtod(str, val);
                                     });
         }},
        {"nsu1",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "nsu1", 42, out_ptr->nsu1,
                                     [](const std::string& str, uint32_t* val) {
                                       return absl::SimpleAtoi(str, val);
                                     });
         }},
        {"nsu2",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(s, "nsu2", 43, out_ptr->nsu2,
                                     [](const std::string& str, uint32_t* val) {
                                       return absl::SimpleAtoi(str, val);
                                     });
         }},
        // device_sn is now std::string, use its specialization of
        // ParseStandardField
        {"device_sn",
         [](const std::string& s, GPCHCX* out_ptr) {
           return ParseStandardField(
               s, "device_sn", 44, out_ptr->device_sn,
               [](const std::string& src, std::string* dest) {
                 *dest = src;
                 return true;
               });
         }},
};

}  // namespace

namespace apollo {
namespace drivers {
namespace gnss {

using apollo::common::Point3D;

const std::unordered_map<std::string, HuaceParser::FrameType>
    HuaceParser::FRAME_HEADER_MAP = {
        {"$GPCHC", FrameType::GPCHC},
        {"$GPCHCX", FrameType::GPCHCX},
        {"$GPGGA", FrameType::GAPPA}
        // Add other headers to frame type mappings here
};

// ProcessHeader implementation for text protocol: find header and skip garbage.
bool HuaceParser::ProcessHeader() {
  const auto buffer_view = buffer_.Peek();
  // Iterate through known headers to find the first match in the buffer
  for (const auto& pair : FRAME_HEADER_MAP) {
    auto pos = buffer_view.find(pair.first);
    if (pos != std::string::npos) {
      buffer_.Drain(pos);
      // Header found. Set the current frame type and header size.
      current_frame_type_ = pair.second;
      current_header_size_ = pair.first.size();
      AINFO_EVERY(100) << "Header found: " << pair.first << ", Frame Type: "
                       << static_cast<int>(current_frame_type_);
      return true;
    }
  }
  return false;
}

// Implementation for ParseHexByte
// Parses 2 hex characters from 'data' starting at 'start' into a single byte.
// Returns std::nullopt if characters are not valid hex digits or data is
// insufficient.
std::optional<uint8_t> HuaceParser::ParseHexByte(std::string_view hex_chars) {
  // A hex byte needs exactly 2 characters.
  if (hex_chars.length() != 2) {
    return std::nullopt;
  }

  auto to_val = [](char c) -> std::optional<uint8_t> {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return std::nullopt;
  };

  auto high = to_val(hex_chars[0]);
  auto low = to_val(hex_chars[1]);

  if (!high || !low) {
    AERROR << "ParseHexByte: Invalid hex characters found in view '"
           << hex_chars << "'";
    return std::nullopt;
  }

  // Combine the high and low nibbles.
  return (*high << 4) | *low;
}

std::optional<std::vector<Parser::ParsedMessage>>
HuaceParser::ProcessPayload() {
  // We are in PROCESS_PAYLOAD state, meaning ProcessHeader found a header
  // and the buffer currently starts with that header.
  // We need to find the frame terminator (\r\n) to get the full frame.

  auto terminator_pos_opt = buffer_.Find(huace::FRAME_TERMINATOR);
  if (!terminator_pos_opt) {
    // Terminator not found. Need more data.
    if (!buffer_.IsEmpty()) {
      ADEBUG << "Huace frame terminator not found. Need more data.";
    }
    return std::nullopt;
  }

  size_t terminator_pos = *terminator_pos_opt;
  // Terminator found. The complete frame includes header, payload, '*', CRC,
  // and terminator. Total frame length is terminator_pos +
  // huace::FRAME_TERMINATOR.size().
  size_t total_frame_length = terminator_pos + huace::FRAME_TERMINATOR.size();

  // --- Calculate payload and checksum bounds based on fixed format ---
  // Format: Header + Payload + '*' + CRC (2 hex chars) + "\r\n"
  // CRC characters are NUMA_CRC_LENGTH bytes long.
  // The '*' delimiter is 1 byte.
  // Terminator is huace::FRAME_TERMINATOR.size() bytes.
  auto frame_view = buffer_.Peek().substr(0, total_frame_length);

  ADEBUG << "frame_view: " << frame_view;

  // Minimum required length: Header + '*' + CRC + Terminator
  const size_t min_frame_size = current_header_size_ + 1 /* '*' */ +
                                huace::NUMA_CRC_LENGTH +
                                huace::FRAME_TERMINATOR.size();

  if (frame_view.length() < min_frame_size) {
    AWARN << "Frame data too short. Consuming malformed frame.";
    buffer_.Drain(total_frame_length);
    return std::vector<Parser::ParsedMessage>();
  }

  // Position of the '*' character (just before CRC hex chars and terminator)
  size_t checksum_delimiter_pos = terminator_pos - huace::NUMA_CRC_LENGTH - 1;

  // Position of the first CRC hex character
  size_t crc_chars_start_pos = terminator_pos - huace::NUMA_CRC_LENGTH;

  // Payload data is from 0 to the '*' delimiter, parser follows will check the
  // validation of header
  size_t payload_start_pos = 0;
  // Payload ends just before '*'
  size_t payload_end_pos = checksum_delimiter_pos;

  // Check if '*' is actually at the expected position
  if (frame_view[checksum_delimiter_pos] != huace::NMEA_CHECKSUM_DELIMITER) {
    AWARN << "Checksum delimiter not found at expected position. Consuming "
             "frame.";
    buffer_.Drain(total_frame_length);
    return std::vector<Parser::ParsedMessage>();
  }

  // Validate checksum.
  // the checksum calculation includes the header but not the 1st character `$`
  bool checksum_ok = IsChecksumValid(frame_view, 1, crc_chars_start_pos);

  if (!checksum_ok) {
    AWARN << "Checksum validation failed. Consuming frame.";
    buffer_.Drain(total_frame_length);
    return std::vector<Parser::ParsedMessage>();
  }

  // --- Parse payload using the view ---
  // The frame is now validated. Extract payload view and pass to parser.
  auto payload_view =
      frame_view.substr(payload_start_pos, payload_end_pos - payload_start_pos);

  // Parse the payload.
  std::vector<Parser::ParsedMessage> messages;
  switch (current_frame_type_) {
    case FrameType::GPCHC:
    case FrameType::GPCHCX:
      messages = ParseGPCHCX(payload_view);
      break;
    case FrameType::GAPPA:
      messages = ParseGAPPA(frame_view);
      break;
    default:
      AERROR << "Unknown frame type after checksum validation.";
      break;
  }

  // Consume the successfully processed frame from the buffer
  buffer_.Drain(total_frame_length);

  return messages;
}

bool ParseFieldsToStruct(const std::vector<std::string>& items,
                         huace::GPCHCX* out) {
  CHECK_NOTNULL(out);

  // check if the number of items matches the expected number
  if (items.size() > parsers.size() + 1) {  // +1 for the header
    AERROR << "Incorrect number of fields in message. Expected "
           << parsers.size() + 1 << " (including header), Got " << items.size();
    return false;
  }

  // parse the header
  const std::string& header_str = items[0];
  // Check if the header matches expected values
  if (header_str != huace::GPCHCX::header &&
      header_str != huace::GPCHC::header) {
    AERROR << "Invalid header: Expected '" << huace::GPCHCX::header << "' or '"
           << huace::GPCHC::header << "', Got '" << header_str << "'.";
    return false;
  }

  // Iterate through parsers and parse each field
  // this function used both for GPCHC and GPCHCX, but field number of GPCHC is
  // 24, less than parsers.size() + 1, so terminate early by items.size()
  for (size_t i = 0; i < parsers.size() && i < items.size(); ++i) {
    // Skip the header item (index 0) and parse the rest
    size_t current_item_index = i + 1;
    const std::string& field_string = items[current_item_index];
    const char* field_name = parsers[i].name;

    if (parsers[i].parser == nullptr) {
      // If parser is nullptr, we skip parsing and just log the field
      ADEBUG << "Skipping field: " << field_name << " at item index "
             << current_item_index << " (value: '" << field_string << "')";
      continue;
    }
    // access parser via index directly for performance, the parsers structure
    // must be aligned with the fields in the GPCHCX or GPCHC struct
    if (!parsers[i].parser(field_string, out)) {
      AERROR << "Parsing failed for field: " << field_name << " at item index "
             << current_item_index << " (string: '" << field_string << "')";
      return false;
    }
  }

  return true;
}

bool HuaceParser::IsChecksumValid(std::string_view frame_view,
                                  size_t payload_start,
                                  size_t crc_chars_start) {
  // Extract payload view for checksum calculation
  auto payload_view = frame_view.substr(
      payload_start, crc_chars_start - payload_start - 1 /* for '*' */);

  // Extract checksum hex characters view
  auto crc_hex_view =
      frame_view.substr(crc_chars_start, huace::NUMA_CRC_LENGTH);

  // Calculate checksum
  uint8_t calculated_checksum = 0;
  for (char c : payload_view) {
    calculated_checksum ^= static_cast<uint8_t>(c);
  }

  // Parse expected checksum (a helper is needed for this)
  auto expected_checksum_opt = ParseHexByte(crc_hex_view);
  if (!expected_checksum_opt) {
    AWARN << "Failed to parse checksum hex characters: " << crc_hex_view;
    return false;
  }

  if (calculated_checksum != *expected_checksum_opt) {
    AWARN << "Checksum mismatch. Calculated: " << std::hex
          << (int)calculated_checksum << ", Expected: " << std::hex
          << (int)(*expected_checksum_opt);
    return false;
  }

  return true;
}

std::vector<Parser::ParsedMessage> HuaceParser::ParseGPCHCX(
    std::string_view payload_view) {
  std::vector<Parser::ParsedMessage> parsed_messages;

  // 2. Use absl::StrSplit to split the frame string by comma
  std::vector<std::string> items = absl::StrSplit(payload_view, ',');

  // 3. Parse common and huace::GPCHCX specific fields
  huace::GPCHCX gpchcx;  // Use huace::GPCHCX struct for both
  if (!ParseFieldsToStruct(items, &gpchcx)) {
    // ParseFieldsToStruct logged the specific error
    // Data was consumed in ProcessPayload (assumed), return empty vector
    return parsed_messages;  // Return empty vector on parsing failure
  }

  // 3. Fill protobuf messages
  // Create protobuf messages on the stack first
  GnssBestPose bestpos;
  Imu imu;
  Ins ins;
  InsStat ins_stat;
  Heading heading;

  // Fill protobuf messages from the parsed struct
  FillGnssBestpos(gpchcx, &bestpos);
  FillImu(gpchcx, &imu);
  FillHeading(gpchcx, &heading);
  FillIns(gpchcx, &ins);
  // CORRECTED: Pass the actual status from the parsed struct
  FillInsStat(gpchcx, &ins_stat);

  parsed_messages.emplace_back(MessageType::BEST_GNSS_POS,
                               std::make_shared<GnssBestPose>(bestpos));
  parsed_messages.emplace_back(MessageType::IMU, std::make_shared<Imu>(imu));
  parsed_messages.emplace_back(MessageType::INS, std::make_shared<Ins>(ins));
  parsed_messages.emplace_back(MessageType::INS_STAT,
                               std::make_shared<InsStat>(ins_stat));
  parsed_messages.emplace_back(MessageType::HEADING,
                               std::make_shared<Heading>(heading));
  return parsed_messages;
}

// Implement ParseGAPPA here (for raw GPGGA passthrough)
std::vector<Parser::ParsedMessage> HuaceParser::ParseGAPPA(
    std::string_view frame_view) {
  std::vector<Parser::ParsedMessage> parsed_messages;

  // The variant expects a shared_ptr to a vector of uint8_t for raw data.
  // We construct that directly from the string_view's data.
  auto raw_ptr = std::make_shared<std::vector<uint8_t>>(frame_view.begin(),
                                                        frame_view.end());

  parsed_messages.emplace_back(MessageType::GPGGA, std::move(raw_ptr));
  return parsed_messages;
}

void HuaceParser::FillGnssBestpos(const huace::GPCHCX& gpchcx,
                                  GnssBestPose* bestpos) {
  // Note: Assuming gpchcx contains relevant fields populated by
  // ParseGPCHC/ParseGPCHCX even if they were not explicitly in the original
  // GPCHC definition (e.g., std deviations). This might require checking items
  // vector size in ParseGPCHC and assigning defaults if fields are missing.
  bestpos->set_measurement_time(gpchcx.gps_week * kSecondsPerWeek +
                                gpchcx.seconds_in_gps_week);
  // Use mapping helpers
  bestpos->set_sol_status(ToSolutionStatus(gpchcx.status.get_system_status()));
  bestpos->set_sol_type(ToSolutionType(gpchcx.status.get_satellite_status()));
  bestpos->set_latitude(gpchcx.latitude);
  bestpos->set_longitude(gpchcx.longitude);
  bestpos->set_height_msl(gpchcx.altitude);
  // Fill standard deviations - these might only be available in huace::GPCHCX,
  // need to check
  bestpos->set_latitude_std_dev(gpchcx.latitude_std);
  bestpos->set_longitude_std_dev(gpchcx.longitude_std);
  bestpos->set_height_std_dev(gpchcx.altitude_std);

  // Fill satellite counts
  bestpos->set_num_sats_tracked(gpchcx.nsv1 + gpchcx.nsv2);
  // TODO(All): add logic to fill other satellite counts if available
  // Fields like num_sats_in_solution, num_sats_l1, num_sats_multi might need
  // mapping from gpchcx or status code, check protocol spec.
  // bestpos->set_num_sats_in_solution(...);
}

void HuaceParser::FillImu(const huace::GPCHCX& gpchcx, Imu* imu) {
  imu->set_measurement_time(gpchcx.gps_week * kSecondsPerWeek +
                            gpchcx.seconds_in_gps_week);

  Point3D* linear_acceleration = imu->mutable_linear_acceleration();
  // Ensure correct coordinate transform (RFU to FLU)
  rfu_to_flu(gpchcx.acc_x * kAccelerationGravity,
             gpchcx.acc_y * kAccelerationGravity,
             gpchcx.acc_z * kAccelerationGravity, linear_acceleration);

  Point3D* angular_velocity = imu->mutable_angular_velocity();
  // Ensure correct coordinate transform (RFU to FLU)
  rfu_to_flu(gpchcx.gyro_x * kDegToRad, gpchcx.gyro_y * kDegToRad,
             gpchcx.gyro_z * kDegToRad, angular_velocity);
}

void HuaceParser::FillHeading(const huace::GPCHCX& gpchcx, Heading* heading) {
  heading->set_measurement_time(gpchcx.gps_week * kSecondsPerWeek +
                                gpchcx.seconds_in_gps_week);
  heading->set_solution_status(
      ToSolutionStatus(gpchcx.status.get_system_status()));
  heading->set_position_type(
      ToSolutionType(gpchcx.status.get_satellite_status()));
  heading->set_heading(gpchcx.heading);
  heading->set_pitch(gpchcx.pitch);
  // Fill standard deviations - might only be available in huace::GPCHCX
  heading->set_heading_std_dev(gpchcx.heading_std);
  heading->set_pitch_std_dev(gpchcx.pitch_std);

  // TODO(All): Add logic to fill fields of satellite number if available
  // Fields like station_id, satellite counts might need mapping
  // heading->set_station_id("0"); // Default or map from gpchcx?
  // heading->set_satellite_number_multi(...);
  // heading->set_satellite_solution_number(...);
}

void HuaceParser::FillIns(const huace::GPCHCX& gpchcx, Ins* ins) {
  // FIX: Use GPS time for Protobuf header timestamp as well
  double gps_time_sec =
      gpchcx.gps_week * kSecondsPerWeek + gpchcx.seconds_in_gps_week;
  ins->mutable_header()->set_timestamp_sec(gps_time_sec);  // Use sensor time

  ins->set_measurement_time(gps_time_sec);

  // Map SolutionType to Ins::Type
  SolutionType solution_type =
      ToSolutionType(gpchcx.status.get_satellite_status());
  switch (solution_type) {
    // Map relevant SolutionTypes to Ins::Type
    case SolutionType::INS_RTKFIXED:
    case SolutionType::NARROW_INT:
    case SolutionType::INS_RTKFLOAT:
    case SolutionType::NARROW_FLOAT:
      ins->set_type(Ins::GOOD);
      break;
    case SolutionType::SINGLE:
      ins->set_type(Ins::CONVERGING);
      break;
    case SolutionType::WIDELANE:       // Often implies float RTK
    case SolutionType::FLOATCONV:      // Explicit float RTK
      ins->set_type(Ins::CONVERGING);  // Or GOOD_FLOAT? Check Ins::Type options
      break;
    case SolutionType::RTK_DIRECT_INS:
      ins->set_type(Ins::GOOD);
      break;
    default:
      ins->set_type(Ins::INVALID);  // Handle unknown or invalid statuses
      break;
  }

  apollo::common::PointLLH* position = ins->mutable_position();
  position->set_lon(gpchcx.longitude);
  position->set_lat(gpchcx.latitude);
  position->set_height(gpchcx.altitude);

  Point3D* euler_angles = ins->mutable_euler_angles();
  // Ensure correct mapping and units (Roll/Pitch/Heading vs Euler angles)
  // Check coordinate system conversions (RFU to FLU, Azimuth to Yaw)
  euler_angles->set_x(gpchcx.roll * kDegToRad);
  // Check pitch sign convention based on Apollo FLU
  euler_angles->set_y(-gpchcx.pitch * kDegToRad);
  // Assuming heading is Azimuth 0-360 North=0, East=90
  // to Apollo Yaw
  euler_angles->set_z(azimuth_deg_to_yaw_rad(gpchcx.heading));

  Point3D* linear_velocity = ins->mutable_linear_velocity();
  // Check mapping of Ve/Vn/Vu to FLU velocity components (East, North, Up) ->
  // (X, Y, Z)
  linear_velocity->set_x(gpchcx.velocity_east);
  linear_velocity->set_y(gpchcx.velocity_north);
  linear_velocity->set_z(gpchcx.velocity_up);

  // Assuming these are RFU gyro rates
  Point3D* angular_velocity = ins->mutable_angular_velocity();
  rfu_to_flu(gpchcx.gyro_x * kDegToRad, gpchcx.gyro_y * kDegToRad,
             gpchcx.gyro_z * kDegToRad, angular_velocity);

  Point3D* linear_acceleration = ins->mutable_linear_acceleration();
  // Assuming these are RFU accelerations
  rfu_to_flu(gpchcx.acc_x * kAccelerationGravity,
             gpchcx.acc_y * kAccelerationGravity,
             gpchcx.acc_z * kAccelerationGravity, linear_acceleration);

  // Optional: Fill standard deviation fields if available in huace::GPCHCX
}

void HuaceParser::FillInsStat(const huace::GPCHCX& gpchcx, InsStat* ins_stat) {
  ins_stat->set_ins_status(
      static_cast<uint32_t>(gpchcx.status.get_system_status()));

  // IMPROVE: Map huace::Status to InsStat::GpsInfo and PosType
  // This requires knowing the mapping from Huace status codes to these enums.
  // Example (PLACEHOLDER - CHECK HUACE SPEC AND PROTO DEFINITIONS):
  /*
  switch(status) {
      case huace::Status::RTK_STABLE_ORIENT:
          ins_stat->set_gps_info(InsStat::GPS_FIX); // Or RTK_FIX?
          ins_stat->set_pos_type(InsStat::INS_RTK_FIXED);
          break;
      case huace::Status::RTK_FLOAT_ORIENT:
          ins_stat->set_gps_info(InsStat::GPS_FIX); // Or RTK_FLOAT?
          ins_stat->set_pos_type(InsStat::INS_RTK_FLOAT);
          break;
      case huace::Status::SINGLE_POS_ORIENT:
      case huace::Status::SINGLE_POS_NO_ORIENT:
          ins_stat->set_gps_info(InsStat::GPS_FIX); // Or GPS_SINGLE?
          ins_stat->set_pos_type(InsStat::INS_SINGLE);
          break;
      // ... handle other statuses ...
      default:
          ins_stat->set_gps_info(InsStat::NO_FIX);
          ins_stat->set_pos_type(InsStat::INS_INVALID);
          break;
  }
  */
  // If a direct mapping is not possible, logging the raw status code might be
  // sufficient depending on requirements.
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
