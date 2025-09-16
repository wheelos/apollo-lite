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

#include "modules/drivers/radar/yg_radar/protocol/radar_object_list_560.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace yg_radar {

using ::apollo::drivers::canbus::Byte;

Radarobjectlist560::Radarobjectlist560() {}
const int32_t Radarobjectlist560::ID = 0x560;

uint32_t Radarobjectlist560::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

void Radarobjectlist560::Parse(
    const std::uint8_t* bytes, int32_t length,
    ::apollo::drivers::yg_radar::Yg_radar* message) const {
  message->mutable_radar_object_list_560()->set_object_number(
      object_number(bytes, length));
  message->mutable_radar_object_list_560()->set_guardrail_distance(
      guardrail_distance(bytes, length));
  message->mutable_radar_object_list_560()->set_vehicle_speed(
      vehicle_speed(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name':
// 'object_number', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|32]',
// 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Radarobjectlist560::object_number(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 8, 'is_signed_var': True, 'len': 16, 'name':
// 'guardrail_distance', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-3276.8|3276.7]', 'physical_unit': 'm', 'precision': 0.1, 'type': 'double'}
double Radarobjectlist560::guardrail_distance(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 24, 'is_signed_var': True, 'len': 1, 'name':
// 'vehicle_speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-3276.8|3276.7]', 'physical_unit': 'km/h', 'precision': 0.1, 'type':
// 'bool'}
bool Radarobjectlist560::vehicle_speed(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 1);

  x <<= 31;
  x >>= 31;

  bool ret = x * 0.100000;
  return ret;
}
}  // namespace yg_radar
}  // namespace drivers
}  // namespace apollo
