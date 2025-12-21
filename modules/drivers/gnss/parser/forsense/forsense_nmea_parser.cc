/******************************************************************************
 * Copyright 2025 Pride Leong. All Rights Reserved.
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
#include "modules/drivers/gnss/parser/forsense/forsense_nmea_parser.h"

#include <cmath>
#include <cstring>
#include <functional>
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
#include "modules/drivers/gnss/parser/forsense/forsense_messages.h"
#include "modules/drivers/gnss/util/util.h"

namespace apollo {
namespace drivers {
namespace gnss {

namespace {

using apollo::drivers::gnss::SolutionStatus;
using apollo::drivers::gnss::SolutionType;
using apollo::drivers::gnss::forsense::SatelliteStatus;
using apollo::drivers::gnss::forsense::SystemStatus;

struct GPYJFieldParser {
  std::string field_name;
  std::function<bool(const std::string&, forsense::GPYJ*)> parse_function;
};

struct GPATTFieldParser {
  std::string field_name;
  std::function<bool(const std::string&, forsense::GPATT*)> parse_function;
};

std::string clean_number_string(const std::string& input) {
  std::string output;

  if (input.empty()) {
    return "";
  }

  bool has_sign = input[0] == '+' || input[0] == '-';
  // Remove leading and trailing spaces
  size_t start = has_sign ? 1 : 0;
  while (start < input.size() && input[start] == ' ') {
    ++start;
  }
  size_t end = input.size();
  while (end > start && input[end - 1] == ' ') {
    --end;
  }
  output = input.substr(start, end - start);
  if (has_sign) {
    // Only prepend the sign if there is at least one digit/character after
    // trimming
    if (!output.empty()) {
      output = input[0] + output;
    } else {
      output = "";
    }
  }
  return output;
}

}  // namespace

static const std::
    vector<GPYJFieldParser>
        gpyj_field_parsers =
            {
                // field 1 gps_week
                {"gps_week",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtoi(str, &(msg->gps_week));
                   return ret;
                 }},
                // field 2 gps_time
                {"gps_time",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->gps_time));
                   return ret;
                 }},
                // field 3 heading
                {"heading",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->heading));
                   return ret;
                 }},
                // field 4 pitch
                {"pitch",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->pitch));
                   return ret;
                 }},
                // field 5 roll
                {"roll",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->roll));
                   return ret;
                 }},
                // field 6 gyro_x
                {"gyro_x",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->gyro_x));
                   return ret;
                 }},
                // field 7 gyro_y
                {"gyro_y",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->gyro_y));
                   return ret;
                 }},
                // field 8 gyro_z
                {"gyro_z",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->gyro_z));
                   return ret;
                 }},
                // field 9 acc_x
                {"acc_x",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->acc_x));
                   return ret;
                 }},
                // field 10 acc_y
                {"acc_y",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->acc_y));
                   return ret;
                 }},
                // field 11 acc_z
                {"acc_z",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->acc_z));
                   return ret;
                 }},
                // field 12 latitude
                {"latitude",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   // Clean the string to remove leading/trailing spaces
                   bool ret = absl::SimpleAtod(clean_number_string(str),
                                               &(msg->latitude));
                   return ret;
                 }},
                // field 13 longitude
                {"longitude",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   // Longitude may have leading spaces after sign
                   bool ret = absl::SimpleAtod(clean_number_string(str),
                                               &(msg->longitude));
                   return ret;
                 }},
                // field 14 altitude
                {"altitude",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->altitude));
                   return ret;
                 }},
                // field 15 velocity_east
                {"velocity_east",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->velocity_east));
                   return ret;
                 }},
                // field 16 velocity_north
                {"velocity_north",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->velocity_north));
                   return ret;
                 }},
                // field 17 velocity_up
                {"velocity_up",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->velocity_up));
                   return ret;
                 }},
                // field 18 speed
                {"speed",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtod(str, &(msg->speed));
                   return ret;
                 }},
                // field 19 nsv1
                {"nsv1",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtoi(str, &(msg->nsv1));
                   return ret;
                 }},
                // field 20 nsv2
                {"nsv2",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtoi(str, &(msg->nsv2));
                   return ret;
                 }},
                // field 21 status
                {"status",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   std::string result;
                   if (!absl::HexStringToBytes(str, &result)) {
                     AERROR << "Failed to parse status field: " << str;
                     return false;
                   }
                   msg->status.raw_value = static_cast<uint8_t>(result[0]);
                   return true;
                 }},
                // field 22 age
                {"age",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   bool ret = absl::SimpleAtoi(str, &(msg->age));
                   return ret;
                 }},
                // field 23 warning_cs
                {"warning_cs",
                 [](const std::string& str, forsense::GPYJ* msg) -> bool {
                   msg->warning_cs = str;
                   return true;
                 }}};

static const std::vector<GPATTFieldParser> gpatt_field_parsers = {
    {"time",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       return absl::SimpleAtod(str, &(msg->time));
     }},
    {"status",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       if (str.size() != 1) {
         AERROR << "Invalid status field length: " << str;
         return false;
       }
       msg->status = str[0];
       return true;
     }},
    {"roll_angle",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       return absl::SimpleAtod(str, &(msg->roll_angle));
     }},
    {"indicator_of_roll",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       if (str.size() != 1) {
         AERROR << "Invalid indicator_of_roll field length: " << str;
         return false;
       }
       msg->indicator_of_roll = str[0];
       return true;
     }},
    {"pitch_angle",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       return absl::SimpleAtod(str, &(msg->pitch_angle));
     }},
    {"indicator_of_pitch",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       if (str.size() != 1) {
         AERROR << "Invalid indicator_of_pitch field length: " << str;
         return false;
       }
       msg->indicator_of_pitch = str[0];
       return true;
     }},
    {"heading_angle",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       return absl::SimpleAtod(str, &(msg->heading_angle));
     }},
    {"roll_angle_uncertainty",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       return absl::SimpleAtod(str, &(msg->roll_angle_uncertainty));
     }},
    {"pitch_angle_uncertainty",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       return absl::SimpleAtod(str, &(msg->pitch_angle_uncertainty));
     }},
    {"heading_angle_uncertainty",
     [](const std::string& str, forsense::GPATT* msg) -> bool {
       return absl::SimpleAtod(str, &(msg->heading_angle_uncertainty));
     }}};

const std::unordered_map<std::string, ForsenseNmeaParser::FrameType>
    ForsenseNmeaParser::FRAME_HEADER_MAP = {
        {"$GPYJ", FrameType::GPYJ},
        {"$GPCHC", FrameType::GPCHC},
        {"$GPAAT", FrameType::GPATT},
        {"$GPGGA", FrameType::GPGGA}
        // Add other headers to frame type mappings here
};

// ProcessHeader implementation for text protocol: find header and skip garbage.
bool ForsenseNmeaParser::ProcessHeader() {
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

std::optional<std::vector<Parser::ParsedMessage>>
ForsenseNmeaParser::ProcessPayload() {
  // We are in PROCESS_PAYLOAD state, meaning ProcessHeader found a header
  // and the buffer currently starts with that header.
  // We need to find the frame terminator (\r\n) to get the full frame.
  auto terminator_pos_opt = buffer_.Find(forsense::FRAME_TERMINATOR);
  if (!terminator_pos_opt) {
    // Terminator not found. Need more data.
    if (!buffer_.IsEmpty()) {
      ADEBUG << "ForsenseNmeaParser::ProcessPayload: "
             << "Incomplete frame, waiting for more data.";
    }
    return std::nullopt;
  }

  size_t terminator_pos = *terminator_pos_opt;
  // Terminator found. The complete frame includes header, payload, '*', CRC,
  size_t total_frame_length =
      terminator_pos + forsense::FRAME_TERMINATOR.size();

  // --- Calculate payload and checksum bounds based on fixed format ---
  // Format: Header + Payload + '*' + CRC (2 hex chars) + "\r\n"
  // CRC characters are NMEA_CRC_LENGTH bytes long.
  // The '*' delimiter is 1 byte.
  auto frame_view = buffer_.Peek().substr(0, total_frame_length);
  ADEBUG << "ForsenseNmeaParser::ProcessPayload: "
         << "Processing frame: " << frame_view;

  const size_t min_frame_size = current_header_size_ + 1 +
                                forsense::NMEA_CRC_LENGTH +
                                forsense::FRAME_TERMINATOR.size();
  if (frame_view.size() < min_frame_size) {
    AWARN << "ForsenseNmeaParser::ProcessPayload: "
          << "Frame too short, expected at least " << min_frame_size
          << " bytes, got " << frame_view.size() << " bytes.";
    buffer_.Drain(total_frame_length);
    return std::vector<Parser::ParsedMessage>();
  }

  // Position of the '*' character (just before CRC hex chars and terminator)
  size_t checksum_delimiter_pos =
      terminator_pos - forsense::NMEA_CRC_LENGTH - 1;
  // Position of the first CRC hex character
  size_t crc_chars_start_pos = checksum_delimiter_pos + 1;

  // Payload data is from 0 to the '*' delimiter, parser follows will check the
  // validation of header
  size_t payload_start_pos = 0;
  // Payload ends just before '*'
  size_t payload_end_pos = checksum_delimiter_pos;

  // Check if '*' is actually at the expected position
  if (frame_view[checksum_delimiter_pos] != forsense::NMEA_CHECKSUM_DELIMITER) {
    AWARN << "ForsenseNmeaParser::ProcessPayload: "
          << "Invalid checksum delimiter at position "
          << checksum_delimiter_pos;
    buffer_.Drain(total_frame_length);
    return std::vector<Parser::ParsedMessage>();
  }

  // Validate checksum.
  // the checksum calculation includes the header, but not leading '$'
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
    case FrameType::GPYJ:
    case FrameType::GPCHC:
      messages = ParseGPYJ(payload_view);
      break;
    case FrameType::GPATT:
      messages = ParseGPATT(payload_view);
    case FrameType::GPGGA:
      messages = ParseGPGGA(frame_view);
      break;
    default:
      AERROR << "ForsenseNmeaParser::ProcessPayload: "
             << "Unknown frame type: " << static_cast<int>(current_frame_type_);
      break;
  }

  // Consume the successfully processed frame from the buffer
  buffer_.Drain(total_frame_length);

  return messages;
}

bool ForsenseNmeaParser::IsChecksumValid(std::string_view frame_view,
                                         size_t payload_start,
                                         size_t crc_chars_start) {
  // Extract payload view for checksum calculation
  // The payload is from payload_start to just before crc_chars_start - 1
  auto payload_view =
      frame_view.substr(payload_start, crc_chars_start - payload_start - 1);

  // Extract checksum hex characters view
  auto crc_hex_view =
      frame_view.substr(crc_chars_start, forsense::NMEA_CRC_LENGTH);

  // Calculate checksum
  uint8_t calculated_checksum = 0;
  for (char c : payload_view) {
    calculated_checksum ^= static_cast<uint8_t>(c);
  }

  // Parse expected checksum (a helper is needed for this)
  std::string expected_checksum_hex;
  if (!absl::HexStringToBytes(crc_hex_view, &expected_checksum_hex)) {
    AWARN << "Failed to parse checksum hex characters: " << crc_hex_view;
    return false;
  }

  if (calculated_checksum != static_cast<uint8_t>(expected_checksum_hex[0])) {
    AWARN << "Checksum mismatch. Calculated: " << std::hex
          << static_cast<int>(calculated_checksum)
          << ", Expected: " << crc_hex_view;
    return false;
  }

  return true;
}

std::vector<Parser::ParsedMessage> ForsenseNmeaParser::ParseGPYJ(
    std::string_view payload_view) {
  std::vector<Parser::ParsedMessage> parsed_messages;

  std::vector<std::string> items = absl::StrSplit(payload_view, ',');
  if (items.size() > gpyj_field_parsers.size() + 1) {
    AERROR << "ForsenseNmeaParser::ParseGPYJ: "
           << "Unexpected number of fields in GPYJ message: " << items.size();
    return parsed_messages;
  }
  forsense::GPYJ gpyj;
  for (size_t i = 0; i < gpyj_field_parsers.size() && i < items.size(); ++i) {
    // skip header
    size_t item_index = i + 1;
    const std::string& item = items[item_index];
    const char* field_name = gpyj_field_parsers[i].field_name.c_str();
    auto parser = gpyj_field_parsers[i].parse_function;
    if (parser == nullptr) {
      ADEBUG << "Skipping field " << field_name
             << " due to null parser function.";
      continue;
    }
    if (!parser(item, &gpyj)) {
      AERROR << "ForsenseNmeaParser::ParseGPYJ: "
             << "Failed to parse field " << field_name
             << " with value: " << item;
      return parsed_messages;  // Return empty on parsing error
    }
  }

  GnssBestPose bestpos;
  forsense::FillGnssBestpos(gpyj, &bestpos);
  Imu imu;
  forsense::FillImu(gpyj, &imu);
  Ins ins;
  forsense::FillIns(gpyj, &ins);
  InsStat ins_stat;
  forsense::FillInsStat(gpyj, &ins_stat);
  Heading heading;
  forsense::FillHeading(gpyj, &heading);

  // Fill protobuf messages from the parsed struct
  // CORRECTED: Pass the actual status from the parsed struct

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

std::vector<Parser::ParsedMessage> ForsenseNmeaParser::ParseGPATT(
    std::string_view frame_view) {
  std::vector<Parser::ParsedMessage> parsed_messages;
  // TODO(All): Implement ParseGPATT
  return parsed_messages;
}

// Implement ParseGAPPA here (for raw GPGGA passthrough)
// TODO(All): move to generic parser
std::vector<Parser::ParsedMessage> ForsenseNmeaParser::ParseGPGGA(
    std::string_view frame_view) {
  std::vector<Parser::ParsedMessage> parsed_messages;

  // The variant expects a shared_ptr to a vector of uint8_t for raw data.
  // We construct that directly from the string_view's data.
  auto raw_ptr = std::make_shared<std::vector<uint8_t>>(frame_view.begin(),
                                                        frame_view.end());

  parsed_messages.emplace_back(MessageType::GPGGA, std::move(raw_ptr));
  return parsed_messages;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
