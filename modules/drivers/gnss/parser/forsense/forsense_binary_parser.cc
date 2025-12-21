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

#include "modules/drivers/gnss/parser/forsense/forsense_binary_parser.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "cyber/common/macros.h"

namespace apollo {
namespace drivers {
namespace gnss {

namespace {
//

template <typename T>
T ConvertTo(const uint8_t* data, size_t length) {
  if (length != sizeof(T)) {
    throw std::runtime_error("Invalid length for conversion");
  }
  T value;
  std::memcpy(&value, data, sizeof(T));
  return value;
}

}  // namespace

bool ForsenseBinaryParser::ProcessHeader() {
  const auto buffer_view = buffer_.Peek();
  auto pos = buffer_view.find(forsense::BINARY_HEADER);
  if (pos == std::string::npos) {
    ADEBUG << "ForsenseBinaryParser: Header not found in buffer";
    return false;
  }
  buffer_.Drain(pos);
  AINFO_EVERY(100) << "ForsenseBinaryParser: Header found at position " << pos;
  return true;
}

std::optional<std::vector<Parser::ParsedMessage>>
ForsenseBinaryParser::ProcessPayload() {
  const auto buffer_view = buffer_.Peek();
  if (buffer_view.size() < forsense::BINARY_HEADER_LENGTH +
                               forsense::BINARY_FRAME_ID_LENGTH +
                               forsense::BINARY_FRAME_LENGTH_LENGTH) {
    ADEBUG << "ForsenseBinaryParser: Not enough data for payload";
    return std::vector<Parser::ParsedMessage>();
  }
  const uint8_t* buffer_data =
      reinterpret_cast<const uint8_t*>(buffer_view.data());
  auto frame_length =
      ConvertTo<uint16_t>(buffer_data + forsense::BINARY_HEADER_LENGTH +
                              forsense::BINARY_FRAME_ID_LENGTH,
                          forsense::BINARY_FRAME_LENGTH_LENGTH);
  auto total_frame_length = frame_length + forsense::BINARY_HEADER_LENGTH +
                            forsense::BINARY_FRAME_ID_LENGTH +
                            forsense::BINARY_FRAME_LENGTH_LENGTH +
                            forsense::BINARY_CRC_LENGTH;
  if (buffer_view.size() < total_frame_length) {
    ADEBUG << "ForsenseBinaryParser: Not enough data for complete frame";
    return std::nullopt;
  }

  // TODO(All): crc check

  auto frame_id =
      ConvertTo<uint16_t>(buffer_data + forsense::BINARY_HEADER_LENGTH,
                          forsense::BINARY_FRAME_ID_LENGTH);

  std::vector<Parser::ParsedMessage> messages;
  // TODO(All): const variable for frame_id
  if (frame_id == 0x0166) {
    messages = ParseIntegratedNavigationData(buffer_data, total_frame_length);
  } else if (frame_id == 0x0156) {
    messages = ParseAgData(buffer_data, total_frame_length);
  } else {
    AERROR << "ForsenseBinaryParser: Unsupported frame ID: " << frame_id;
  }
  buffer_.Drain(total_frame_length);

  return messages;
}

std::vector<Parser::ParsedMessage> ForsenseBinaryParser::ParseAgData(
    const uint8_t* data, size_t length) {
  std::vector<Parser::ParsedMessage> parsed_messages;
  // TODO(All): Implement parsing logic for AG data
  return parsed_messages;
}

std::vector<Parser::ParsedMessage>
ForsenseBinaryParser::ParseIntegratedNavigationData(const uint8_t* data,
                                                    size_t length) {
  std::vector<Parser::ParsedMessage> parsed_messages;

  forsense::GPYJ gpyj;

  gpyj.gps_time =
      ConvertTo<uint32_t>(data + 6, 4) / 1000.0;  // Convert ms to seconds
  gpyj.gps_week = ConvertTo<uint16_t>(data + 10, 2);
  gpyj.latitude = ConvertTo<int32_t>(data + 12, 4) / 1e7;  // Convert to degrees
  gpyj.longitude =
      ConvertTo<int32_t>(data + 16, 4) / 1e7;              // Convert to degrees
  gpyj.altitude = ConvertTo<int32_t>(data + 20, 4) / 1e3;  // Convert to meters
  gpyj.velocity_north = ConvertTo<float>(data + 24, 4);
  gpyj.velocity_east = ConvertTo<float>(data + 28, 4);
  gpyj.velocity_up = ConvertTo<float>(data + 32, 4);
  gpyj.roll = ConvertTo<float>(data + 36, 4);
  gpyj.pitch = ConvertTo<float>(data + 40, 4);
  gpyj.heading = ConvertTo<float>(data + 44, 4);
  // offset 48
  //    single antenna: AHRS headings
  //    double antenna: RTK headings
  //    G200: front gyro_z
  // offset 52
  //    avaliable if G200 connected, angle of front
  gpyj.acc_x = ConvertTo<float>(data + 56, 4);
  gpyj.acc_y = ConvertTo<float>(data + 60, 4);
  gpyj.acc_z = ConvertTo<float>(data + 64, 4);
  gpyj.gyro_x = ConvertTo<float>(data + 68, 4);
  gpyj.gyro_y = ConvertTo<float>(data + 72, 4);
  gpyj.gyro_z = ConvertTo<float>(data + 76, 4);
  // offset 80
  //   imu temperature
  gpyj.imu_temperature = ConvertTo<float>(data + 80, 4);
  gpyj.rtk_status = ConvertTo<uint8_t>(data + 84, 1);
  gpyj.nsv1 = ConvertTo<uint8_t>(data + 85, 1);
  gpyj.age = ConvertTo<uint8_t>(data + 86, 1);
  gpyj.orientation_status = ConvertTo<uint8_t>(data + 87, 1);
  gpyj.position_accuracy_factor = ConvertTo<uint16_t>(data + 88, 2);
  gpyj.combined_status = ConvertTo<uint16_t>(data + 90, 2);

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

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
