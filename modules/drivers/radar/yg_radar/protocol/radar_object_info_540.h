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

#include "modules/drivers/radar/yg_radar/proto/yg_radar.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace drivers {
namespace yg_radar {

class Radarobjectinfo540 : public ::apollo::drivers::canbus::ProtocolData<
                               ::apollo::drivers::yg_radar::Yg_radar> {
 public:
  static const int32_t ID;
  Radarobjectinfo540();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
             ::apollo::drivers::yg_radar::Yg_radar* message) const override;

 private:
  // config detail: {'bit': 0, 'is_signed_var': True, 'len': 11, 'name':
  // 'Object_Distance_Lon', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-102.4|102.3]', 'physical_unit': 'm', 'precision': 0.1, 'type': 'double'}
  double object_distance_lon(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 11, 'is_signed_var': True, 'len': 11, 'name':
  // 'Object_Speed', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-102.4|102.3]', 'physical_unit': 'm/s', 'precision': 0.1, 'type':
  // 'double'}
  double object_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 22, 'is_signed_var': True, 'len': 11, 'name':
  // 'Object_Angle', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-102.4|102.3]', 'physical_unit': 'deg', 'precision': 0.1, 'type':
  // 'double'}
  double object_angle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'bit': 33, 'is_signed_var': True, 'len': 11, 'name':
  // 'Object_Distance_Lat', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-102.4|102.3]', 'physical_unit': 'm', 'precision': 0.1, 'type': 'double'}
  double object_distance_lat(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 44, 'is_signed_var': True, 'len': 8, 'name':
  // 'Object_Acceleration', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-12.8|12.7]', 'physical_unit': 'm/s^2', 'precision': 0.1, 'type':
  // 'double'}
  double object_acceleration(const std::uint8_t* bytes,
                             const int32_t length) const;

  // config detail: {'bit': 52, 'is_signed_var': True, 'len': 12, 'name':
  // 'Object_Tracking_ID', 'offset': 0.0, 'order': 'intel', 'physical_range':
  // '[-2048|2047]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int object_tracking_id(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace yg_radar
}  // namespace drivers
}  // namespace apollo
