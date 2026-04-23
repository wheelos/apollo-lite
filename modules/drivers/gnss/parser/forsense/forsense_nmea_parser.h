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
#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"
#include "modules/drivers/gnss/proto/config.pb.h"

#include "modules/drivers/gnss/parser/forsense/forsense_messages.h"
#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

class ForsenseNmeaParser : public Parser {
 public:
  // list supported frame types for Forsense NMEA parser
  enum class FrameType {
    UNKNOWN,  // Unrecognized or initial state
    GPYJ,
    GPCHC,
    GPATT,
    GPGGA
  };
  ForsenseNmeaParser() = default;
  explicit ForsenseNmeaParser(const config::Config& config) {}

  // Virtual destructor as required by the base class.
  ~ForsenseNmeaParser() override = default;

  // Overridden methods from the base Parser class.
  // See base class for detailed contract description.
  bool ProcessHeader() override;
  std::optional<std::vector<Parser::ParsedMessage>> ProcessPayload() override;

 private:
  static const std::unordered_map<std::string, FrameType> FRAME_HEADER_MAP;

  bool IsChecksumValid(std::string_view frame_view, size_t payload_start,
                       size_t crc_chars_start);
  std::vector<Parser::ParsedMessage> ParseGPYJ(std::string_view payload_view);
  std::vector<Parser::ParsedMessage> ParseGPATT(std::string_view payload_view);
  std::vector<Parser::ParsedMessage> ParseGPGGA(std::string_view payload_view);

 private:
  // State variable storing the type of the frame currently being processed.
  FrameType current_frame_type_ = FrameType::UNKNOWN;

  // State variable to store the size of the header that was consumed by
  // ProcessHeader. Needed to correctly calculate total frame size after finding
  // terminator in ProcessPayload.
  size_t current_header_size_ = 0;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
