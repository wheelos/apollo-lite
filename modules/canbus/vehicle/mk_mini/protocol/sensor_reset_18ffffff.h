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
namespace mk_mini {

class Sensorreset18ffffff : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Sensorreset18ffffff();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'Close_candiag', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-128|127]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Sensorreset18ffffff* set_close_candiag(int close_candiag);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'Brake_reset', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Sensorreset18ffffff* set_brake_reset(bool brake_reset);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'steer_reset', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Sensorreset18ffffff* set_steer_reset(bool steer_reset);

 private:

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'Close_candiag', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-128|127]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_close_candiag(uint8_t* data, int close_candiag);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'Brake_reset', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_brake_reset(uint8_t* data, bool brake_reset);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'steer_reset', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_steer_reset(uint8_t* data, bool steer_reset);

  int close_candiag(const std::uint8_t* bytes, const int32_t length) const;

  bool brake_reset(const std::uint8_t* bytes, const int32_t length) const;

  bool steer_reset(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int close_candiag_;
  bool brake_reset_;
  bool steer_reset_;
};

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo


