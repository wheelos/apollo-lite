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

class Iocmd18c4d7d0 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Iocmd18c4d7d0();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_disCharge', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_discharge(bool io_cmd_discharge);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'io_cmd_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Iocmd18c4d7d0* set_io_cmd_check_bcc(int io_cmd_check_bcc);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'io_cmd_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Iocmd18c4d7d0* set_io_cmd_alive_cnt(int io_cmd_alive_cnt);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_speaker', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_speaker(bool io_cmd_speaker);

  // config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_fog_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_fog_lamp(bool io_cmd_fog_lamp);

  // config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_clearance_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_clearance_lamp(bool io_cmd_clearance_lamp);

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_braking_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_braking_lamp(bool io_cmd_braking_lamp);

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name': 'io_cmd_turn_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Iocmd18c4d7d0* set_io_cmd_turn_lamp(int io_cmd_turn_lamp);

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_upper_beam_headlamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_upper_beam_headlamp(bool io_cmd_upper_beam_headlamp);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_lower_beam_headlamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_lower_beam_headlamp(bool io_cmd_lower_beam_headlamp);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_enable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  Iocmd18c4d7d0* set_io_cmd_enable(bool io_cmd_enable);

 private:

  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_disCharge', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_discharge(uint8_t* data, bool io_cmd_discharge);

  // config detail: {'bit': 56, 'is_signed_var': False, 'len': 8, 'name': 'io_cmd_check_bcc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_io_cmd_check_bcc(uint8_t* data, int io_cmd_check_bcc);

  // config detail: {'bit': 52, 'is_signed_var': False, 'len': 4, 'name': 'io_cmd_alive_cnt', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_io_cmd_alive_cnt(uint8_t* data, int io_cmd_alive_cnt);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_speaker', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_speaker(uint8_t* data, bool io_cmd_speaker);

  // config detail: {'bit': 14, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_fog_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_fog_lamp(uint8_t* data, bool io_cmd_fog_lamp);

  // config detail: {'bit': 13, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_clearance_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_clearance_lamp(uint8_t* data, bool io_cmd_clearance_lamp);

  // config detail: {'bit': 12, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_braking_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_braking_lamp(uint8_t* data, bool io_cmd_braking_lamp);

  // config detail: {'bit': 10, 'is_signed_var': False, 'len': 2, 'name': 'io_cmd_turn_lamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_io_cmd_turn_lamp(uint8_t* data, int io_cmd_turn_lamp);

  // config detail: {'bit': 9, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_upper_beam_headlamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_upper_beam_headlamp(uint8_t* data, bool io_cmd_upper_beam_headlamp);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_lower_beam_headlamp', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_lower_beam_headlamp(uint8_t* data, bool io_cmd_lower_beam_headlamp);

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 1, 'name': 'io_cmd_enable', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'bool'}
  void set_p_io_cmd_enable(uint8_t* data, bool io_cmd_enable);

  bool io_cmd_discharge(const std::uint8_t* bytes, const int32_t length) const;

  int io_cmd_check_bcc(const std::uint8_t* bytes, const int32_t length) const;

  int io_cmd_alive_cnt(const std::uint8_t* bytes, const int32_t length) const;

  bool io_cmd_speaker(const std::uint8_t* bytes, const int32_t length) const;

  bool io_cmd_fog_lamp(const std::uint8_t* bytes, const int32_t length) const;

  bool io_cmd_clearance_lamp(const std::uint8_t* bytes, const int32_t length) const;

  bool io_cmd_braking_lamp(const std::uint8_t* bytes, const int32_t length) const;

  int io_cmd_turn_lamp(const std::uint8_t* bytes, const int32_t length) const;

  bool io_cmd_upper_beam_headlamp(const std::uint8_t* bytes, const int32_t length) const;

  bool io_cmd_lower_beam_headlamp(const std::uint8_t* bytes, const int32_t length) const;

  bool io_cmd_enable(const std::uint8_t* bytes, const int32_t length) const;

 private:
  bool io_cmd_discharge_;
  int io_cmd_check_bcc_;
  int io_cmd_alive_cnt_;
  bool io_cmd_speaker_;
  bool io_cmd_fog_lamp_;
  bool io_cmd_clearance_lamp_;
  bool io_cmd_braking_lamp_;
  int io_cmd_turn_lamp_;
  bool io_cmd_upper_beam_headlamp_;
  bool io_cmd_lower_beam_headlamp_;
  bool io_cmd_enable_;
};

}  // namespace mk_mini
}  // namespace canbus
}  // namespace apollo


