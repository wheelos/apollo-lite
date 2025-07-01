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

#include "modules/drivers/gnss/parser/huace/huace_messages.h"
#include "modules/drivers/gnss/parser/parser.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Concrete parser for Huace GNSS receiver data (e.g., GPCHC, GPCHCX, GAPPA).
class HuaceParser : public Parser {
 public:
  // Specific frame types handled by this parser.
  enum class FrameType {
    UNKNOWN,  // Unrecognized or initial state
    GPCHC,    // GPCHC frame type
    GPCHCX,   // GPCHCX frame type
    GAPPA     // GPGGA passthrough frame type
    // Add other Huace frame types here
  };

  struct FieldParser {
    const char* name;
    std::function<bool(const std::string&, huace::GPCHCX*)> parser;
  };

  HuaceParser() = default;
  explicit HuaceParser(const config::Config& config) {}

  // Virtual destructor as required by the base class.
  ~HuaceParser() override = default;

  // Overridden methods from the base Parser class.
  // See base class for detailed contract description.
  bool ProcessHeader() override;
  std::optional<std::vector<Parser::ParsedMessage>> ProcessPayload() override;

 private:
  // Static map associating frame header strings with their corresponding
  // FrameType. Defined in the .cpp file. Use string for easy comparison with
  // buffer content.
  static const std::unordered_map<std::string, FrameType> FRAME_HEADER_MAP;

  // Helper methods to parse specific frame types after a complete frame's data
  // is extracted. These methods handle field splitting, type conversion, and
  // internal error handling (logging, not throwing). They return a vector of
  // parsed messages for that frame, or an empty vector if parsing failed
  // internally.
  std::vector<Parser::ParsedMessage> ParseGPCHCX(std::string_view payload_view);
  std::vector<Parser::ParsedMessage> ParseGAPPA(std::string_view frame_view);

  // Helper method to parse common fields present in multiple GPCHCX-like
  // frames. Takes a vector of string fields and populates a GPCHCX struct.
  // Handles missing fields and invalid conversions internally via logging and
  // default values (not throwing).
  void ParseCommonGPCHCXFields(const std::vector<std::string>& items,
                               huace::GPCHCX& gpchcx);

  bool IsChecksumValid(std::string_view frame_view, size_t payload_start,
                       size_t crc_chars_start);

  std::optional<uint8_t> ParseHexByte(std::string_view hex_chars);

  // Helper methods to fill Protobuf messages from the parsed GPCHCX struct.
  // These methods take a const reference to the parsed data and a pointer to
  // the Protobuf message.
  void FillGnssBestpos(const huace::GPCHCX& gpchcx, GnssBestPose* bestpos);
  void FillImu(const huace::GPCHCX& gpchcx, Imu* imu);
  void FillHeading(const huace::GPCHCX& gpchcx, Heading* heading);
  void FillIns(const huace::GPCHCX& gpchcx, Ins* ins);
  void FillInsStat(const huace::GPCHCX& gpchcx, InsStat* ins_stat);

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
