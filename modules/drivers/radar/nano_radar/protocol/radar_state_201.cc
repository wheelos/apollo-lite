/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/radar/nano_radar/protocol/radar_state_201.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::canbus::Byte;

RadarState201::RadarState201() {}
const uint32_t RadarState201::ID = 0x201;

void RadarState201::Parse(const std::uint8_t* bytes, int32_t length,
                          NanoRadar* nano_radar) const {
  auto state = nano_radar->mutable_radar_state();
  state->set_max_distance(max_dist(bytes, length));
  state->set_output_type(output_type(bytes, length));
  state->set_rcs_threshold(rcs_threshold(bytes, length));
  state->set_radar_power(radar_power(bytes, length));
  state->set_send_quality(send_quality(bytes, length));
  state->set_send_ext_info(send_ext_info(bytes, length));
  state->set_nvm_read_status(nvm_read_status(bytes, length));
  state->set_nvm_write_status(nvm_write_status(bytes, length));
  state->set_sort_index(sort_index(bytes, length));
  state->set_ctrl_relay_cfg(ctrl_relay_cfg(bytes, length));
  state->set_motion_rx_state(motion_rx_state(bytes, length));
  state->set_can_baudrate(can_baudrate(bytes, length));
  state->set_interface_type(interface_type(bytes, length));
  state->set_lvds_select(lvds_select(bytes, length));
  state->set_calibration_enabled(calibration_enabled(bytes, length));
}

int RadarState201::max_dist(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  uint32_t t = t1.get_byte(6, 2);
  x <<= 2;
  x |= t;

  int ret = x * 2;
  return ret;
}

int RadarState201::radar_power(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes + 3);
  uint32_t x = t0.get_byte(0, 2);

  Byte t1(bytes + 4);
  uint32_t t = t1.get_byte(7, 1);
  x <<= 1;
  x |= t;

  int ret = x;
  return ret;
}

NanoRadarState_201::OutputType RadarState201::output_type(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(2, 2);

  switch (x) {
    case 0x0:
      return NanoRadarState_201::OUTPUT_TYPE_NONE;
    case 0x1:
      return NanoRadarState_201::OUTPUT_TYPE_OBJECTS;
    case 0x2:
      return NanoRadarState_201::OUTPUT_TYPE_CLUSTERS;
    default:
      return NanoRadarState_201::OUTPUT_TYPE_ERROR;
  }
}

NanoRadarState_201::RcsThreshold RadarState201::rcs_threshold(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(2, 3);

  switch (x) {
    case 0x0:
      return NanoRadarState_201::RCS_THRESHOLD_STANDARD;
    case 0x1:
      return NanoRadarState_201::RCS_THRESHOLD_HIGH_SENSITIVITY;
    default:
      return NanoRadarState_201::RCS_THRESHOLD_ERROR;
  }
}

bool RadarState201::send_quality(const std::uint8_t* bytes,
                                 int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(4, 1);

  bool ret = (x == 0x1);
  return ret;
}

bool RadarState201::send_ext_info(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(5, 1);

  bool ret = (x == 0x1);
  return ret;
}
bool RadarState201::nvm_read_status(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte t0(bytes + 0);
  uint32_t x = t0.get_byte(6, 1);

  bool ret = (x == 0x1);
  return ret;
}
bool RadarState201::nvm_write_status(const std::uint8_t* bytes,
                                     int32_t length) const {
  Byte t0(bytes + 0);
  uint32_t x = t0.get_byte(7, 1);

  bool ret = (x == 0x1);
  return ret;
}
NanoRadarState_201::SortIndex RadarState201::sort_index(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 4);
  uint32_t x = t0.get_byte(4, 3);

  switch (x) {
    case 0x0:
      return NanoRadarState_201::SORT_INDEX_NO_SORTING;
    case 0x1:
      return NanoRadarState_201::SORT_INDEX_SORT_BY_RANGE;
    case 0x2:
      return NanoRadarState_201::SORT_INDEX_SORT_BY_RCS;
    default:
      return NanoRadarState_201::SORT_INDEX_ERROR;
  }
}
bool RadarState201::ctrl_relay_cfg(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(1, 1);

  bool ret = (x == 0x1);
  return ret;
}
int RadarState201::motion_rx_state(const std::uint8_t* bytes,
                                   int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(6, 2);

  int ret = x;
  return ret;
}
NanoRadarState_201::CANBaudrate RadarState201::can_baudrate(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  uint32_t x = t0.get_byte(5, 3);

  switch (x) {
    case 0x0:
      return NanoRadarState_201::CAN_BAUDRATE_500K;
    case 0x1:
      return NanoRadarState_201::CAN_BAUDRATE_250K;
    case 0x2:
      return NanoRadarState_201::CAN_BAUDRATE_1M;
    default:
      return NanoRadarState_201::CAN_BAUDRATE_ERROR;
  }
}
int RadarState201::interface_type(const std::uint8_t* bytes,
                                  int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}
bool RadarState201::lvds_select(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(5, 1);

  bool ret = (x == 0x1);
  return ret;
}
NanoRadarState_201::CalibrationEnabled RadarState201::calibration_enabled(
    const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  uint32_t x = t0.get_byte(6, 2);

  switch (x) {
    case 0x1:
      return NanoRadarState_201::CALIBRATION_ENABLED_ENABLED;
    case 0x2:
      return NanoRadarState_201::CALIBRATION_ENABLED_INITIAL_RECOVERY;
    default:
      return NanoRadarState_201::CALIBRATION_ENABLED_ERROR;
  }
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
