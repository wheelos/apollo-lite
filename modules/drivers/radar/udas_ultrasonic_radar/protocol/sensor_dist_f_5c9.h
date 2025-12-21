/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/radar/udas_ultrasonic_radar/proto/udas_ultrasonic_radar.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace udas_ultrasonic_radar {

class Sensordistf5c9
    : public ::apollo::drivers::canbus::ProtocolData<
          ::apollo::drivers::udas_ultrasonic_radar::Udas_ultrasonic_radar> {
 public:
  static const int32_t ID;
  Sensordistf5c9();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ::apollo::drivers::udas_ultrasonic_radar::Udas_ultrasonic_radar*
                 message) const override;

 private:
  // config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
  // 'Sensor_6_FF', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|510]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double sensor_6_ff(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
  // 'Sensor_5_FE', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|510]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double sensor_5_fe(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
  // 'Sensor_4_FD', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|510]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double sensor_4_fd(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
  // 'Sensor_3_FC', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|510]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double sensor_3_fc(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
  // 'Sensor_2_FB', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|510]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double sensor_2_fb(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
  // 'Sensor_1_FA', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[0|510]', 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
  double sensor_1_fa(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace udas_ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
