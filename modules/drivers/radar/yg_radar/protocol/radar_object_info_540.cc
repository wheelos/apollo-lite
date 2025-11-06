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

#include "modules/drivers/radar/yg_radar/protocol/radar_object_info_540.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace yg_radar {

using ::apollo::drivers::canbus::Byte;

Radarobjectinfo540::Radarobjectinfo540() {}
const int32_t Radarobjectinfo540::ID = 0x540;

uint32_t Radarobjectinfo540::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Radarobjectinfo540::Parse(
    const std::uint8_t* bytes, int32_t length,
    ::apollo::drivers::yg_radar::Yg_radar* message) const {
  message->mutable_radar_object_info_540()->set_object_distance_lon(
      object_distance_lon(bytes, length));
  message->mutable_radar_object_info_540()->set_object_speed(
      object_speed(bytes, length));
  message->mutable_radar_object_info_540()->set_object_angle(
      object_angle(bytes, length));
  message->mutable_radar_object_info_540()->set_object_distance_lat(
      object_distance_lat(bytes, length));
  message->mutable_radar_object_info_540()->set_object_acceleration(
      object_acceleration(bytes, length));
  message->mutable_radar_object_info_540()->set_object_tracking_id(
      object_tracking_id(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 11, 'name':
// 'object_distance_lon', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-102.4|102.3]', 'physical_unit': 'm', 'precision': 0.1, 'type': 'double'}
double Radarobjectinfo540::object_distance_lon(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 3);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 21;
  x >>= 21;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 11, 'is_signed_var': True, 'len': 11, 'name':
// 'object_speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-102.4|102.3]', 'physical_unit': 'm/s', 'precision': 0.1, 'type': 'double'}
double Radarobjectinfo540::object_speed(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 6);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(3, 5);
  x <<= 5;
  x |= t;

  x <<= 21;
  x >>= 21;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 22, 'is_signed_var': True, 'len': 11, 'name':
// 'object_angle', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-102.4|102.3]', 'physical_unit': 'deg', 'precision': 0.1, 'type': 'double'}
double Radarobjectinfo540::object_angle(const std::uint8_t* bytes,
                                        int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(0, 1);

  Byte t1(bytes + 3);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 2);
  t = t2.get_byte(6, 2);
  x <<= 2;
  x |= t;

  x <<= 21;
  x >>= 21;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 33, 'is_signed_var': True, 'len': 11, 'name':
// 'object_distance_lat', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-102.4|102.3]', 'physical_unit': 'm', 'precision': 0.1, 'type': 'double'}
double Radarobjectinfo540::object_distance_lat(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(1, 7);
  x <<= 7;
  x |= t;

  x <<= 21;
  x >>= 21;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 44, 'is_signed_var': True, 'len': 8, 'name':
// 'object_acceleration', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-12.8|12.7]', 'physical_unit': 'm/s^2', 'precision': 0.1, 'type': 'double'}
double Radarobjectinfo540::object_acceleration(const std::uint8_t* bytes,
                                               int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 24;
  x >>= 24;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 52, 'is_signed_var': True, 'len': 12, 'name':
// 'object_tracking_id', 'offset': 0.0, 'order': 'intel', 'physical_range':
// '[-2048|2047]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Radarobjectinfo540::object_tracking_id(const std::uint8_t* bytes,
                                           int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  int ret = x;
  return ret;
}
}  // namespace yg_radar
}  // namespace drivers
}  // namespace apollo
