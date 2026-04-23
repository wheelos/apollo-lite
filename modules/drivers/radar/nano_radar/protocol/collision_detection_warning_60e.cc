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

#include "modules/drivers/radar/nano_radar/protocol/collision_detection_warning_60e.h"

#include "glog/logging.h"

#include "cyber/time/time.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/radar/nano_radar/protocol/const_vars.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::canbus::Byte;

CollisionDetectionWarning60E::CollisionDetectionWarning60E() {}
const uint32_t CollisionDetectionWarning60E::ID = 0x60E;

void CollisionDetectionWarning60E::Parse(const std::uint8_t* bytes,
                                         int32_t length,
                                         NanoRadar* nano_radar) const {
  int obj_id = object_id(bytes, length);
  auto warning = nano_radar->add_collision_detection_warnings();
  warning->set_obstacle_id(obj_id);
  warning->set_region_bit_field(region_bit_field(bytes, length));
  double timestamp = apollo::cyber::Time::Now().ToSecond();
  auto header = warning->mutable_header();
  header->CopyFrom(nano_radar->header());
  header->set_timestamp_sec(timestamp);
}

int CollisionDetectionWarning60E::object_id(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int CollisionDetectionWarning60E::region_bit_field(const std::uint8_t* bytes,
                                                   int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  double ret = x;
  return ret;
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
