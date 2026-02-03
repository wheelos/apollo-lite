/******************************************************************************
 * Copyright 2025 The WheelOS Team. All Rights Reserved.
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

#include "modules/common_msgs/chassis_msgs/chassis_detail.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace agilex_hunter {

class Motioncommand111 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Motioncommand111();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 16, 'name': 'Speed_Command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-4.8|4.8]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  Motioncommand111* set_speed_command(double speed_command);

  // config detail: {'bit': 63, 'is_signed_var': True, 'len': 16, 'name': 'Steering_Command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-0.4|0.4]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
  Motioncommand111* set_steering_command(double steering_command);

 private:

  // config detail: {'bit': 15, 'is_signed_var': True, 'len': 16, 'name': 'Speed_Command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-4.8|4.8]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  void set_p_speed_command(uint8_t* data, double speed_command);

  // config detail: {'bit': 63, 'is_signed_var': True, 'len': 16, 'name': 'Steering_Command', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-0.4|0.4]', 'physical_unit': 'rad', 'precision': 0.001, 'type': 'double'}
  void set_p_steering_command(uint8_t* data, double steering_command);

  double speed_command(const std::uint8_t* bytes, const int32_t length) const;

  double steering_command(const std::uint8_t* bytes, const int32_t length) const;

 private:
  double speed_command_;
  double steering_command_;
};

}  // namespace agilex_hunter
}  // namespace canbus
}  // namespace apollo


