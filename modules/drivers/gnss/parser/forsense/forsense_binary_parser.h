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

class ForsenseBinaryParser : public Parser {
 public:
  ForsenseBinaryParser() = default;
  explicit ForsenseBinaryParser(const config::Config& config) {}

  // Virtual destructor as required by the base class.
  ~ForsenseBinaryParser() override = default;

  // Overridden methods from the base Parser class.
  // See base class for detailed contract description.
  bool ProcessHeader() override;
  std::optional<std::vector<Parser::ParsedMessage>> ProcessPayload() override;

 private:
  std::vector<Parser::ParsedMessage> ParseAgData(const uint8_t* data,
                                                 size_t length);
  std::vector<Parser::ParsedMessage> ParseIntegratedNavigationData(
      const uint8_t* data, size_t length);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
