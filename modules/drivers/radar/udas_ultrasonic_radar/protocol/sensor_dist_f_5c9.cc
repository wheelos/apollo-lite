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

#include "modules/drivers/radar/udas_ultrasonic_radar/protocol/sensor_dist_f_5c9.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace udas_ultrasonic_radar {

using ::apollo::drivers::canbus::Byte;

Sensordistf5c9::Sensordistf5c9() {}
const int32_t Sensordistf5c9::ID = 0x5C9;

void Sensordistf5c9::Parse(
    const std::uint8_t* bytes, int32_t length,
    ::apollo::drivers::udas_ultrasonic_radar::Udas_ultrasonic_radar* message)
    const {
  message->mutable_sensor_dist_f_5c9()->set_sensor_6_ff(
      sensor_6_ff(bytes, length));
  message->mutable_sensor_dist_f_5c9()->set_sensor_5_fe(
      sensor_5_fe(bytes, length));
  message->mutable_sensor_dist_f_5c9()->set_sensor_4_fd(
      sensor_4_fd(bytes, length));
  message->mutable_sensor_dist_f_5c9()->set_sensor_3_fc(
      sensor_3_fc(bytes, length));
  message->mutable_sensor_dist_f_5c9()->set_sensor_2_fb(
      sensor_2_fb(bytes, length));
  message->mutable_sensor_dist_f_5c9()->set_sensor_1_fa(
      sensor_1_fa(bytes, length));
}

// config detail: {'bit': 40, 'is_signed_var': False, 'len': 8, 'name':
// 'sensor_6_ff', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|510]',
// 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Sensordistf5c9::sensor_6_ff(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 32, 'is_signed_var': False, 'len': 8, 'name':
// 'sensor_5_fe', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|510]',
// 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Sensordistf5c9::sensor_5_fe(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': False, 'len': 8, 'name':
// 'sensor_4_fd', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|510]',
// 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Sensordistf5c9::sensor_4_fd(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name':
// 'sensor_3_fc', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|510]',
// 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Sensordistf5c9::sensor_3_fc(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name':
// 'sensor_2_fb', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|510]',
// 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Sensordistf5c9::sensor_2_fb(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
// 'sensor_1_fa', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|510]',
// 'physical_unit': 'cm', 'precision': 2.0, 'type': 'double'}
double Sensordistf5c9::sensor_1_fa(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  double ret = x * 2.000000;
  return ret;
}
}  // namespace udas_ultrasonic_radar
}  // namespace drivers
}  // namespace apollo
