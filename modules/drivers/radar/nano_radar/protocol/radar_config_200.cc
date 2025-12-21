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

#include "modules/drivers/radar/nano_radar/protocol/radar_config_200.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace nano_radar {

using apollo::drivers::NanoRadar;
using apollo::drivers::canbus::Byte;

const uint32_t RadarConfig200::ID = 0x200;

RadarConfig200::RadarConfig200() {}
RadarConfig200::~RadarConfig200() {}

uint32_t RadarConfig200::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void RadarConfig200::UpdateData(uint8_t* data) {
  set_max_distance_valid_p(data, radar_conf_.max_distance_valid());
  set_sensor_id_valid_p(data, radar_conf_.sensor_id_valid());
  set_radar_power_valid_p(data, radar_conf_.radar_power_valid());
  set_output_type_valid_p(data, radar_conf_.output_type_valid());
  set_send_quality_valid_p(data, radar_conf_.send_quality_valid());
  set_send_ext_info_valid_p(data, radar_conf_.send_ext_info_valid());
  set_sort_index_valid_p(data, radar_conf_.sort_index_valid());
  set_store_in_nvm_valid_p(data, radar_conf_.store_in_nvm_valid());
  set_rcs_threshold_valid_p(data, radar_conf_.rcs_threshold_valid());
  set_baudrate_valid_p(data, radar_conf_.baudrate_valid());
  set_interface_select_valid_p(data, radar_conf_.interface_select_valid());
  set_lvds_valid_p(data, radar_conf_.lvds_valid());
  set_calibration_valid_p(data, radar_conf_.calibration_valid());

  set_max_distance_p(data, static_cast<uint16_t>(radar_conf_.max_distance()));
  set_sensor_id_p(data, static_cast<uint8_t>(radar_conf_.sensor_id()));
  set_output_type_p(data, radar_conf_.output_type());
  set_radar_power_p(data, static_cast<uint8_t>(radar_conf_.radar_power()));
  set_send_ext_info_p(data, radar_conf_.send_ext_info());
  set_send_quality_p(data, radar_conf_.send_quality());
  set_sort_index_p(data, radar_conf_.sort_index());
  set_store_in_nvm_p(data, static_cast<uint8_t>(radar_conf_.store_in_nvm()));
  set_rcs_threshold_p(data, radar_conf_.rcs_threshold());
  set_baudrate_p(data, radar_conf_.baudrate());
  set_interface_type_p(data, radar_conf_.interface_type());
  set_lvds_select_p(data, radar_conf_.lvds_select());
  set_calibration_enabled_p(data, radar_conf_.calibration_enabled());
}

/**
 * @brief reset the private variables
 */
void RadarConfig200::Reset() {
  radar_conf_.set_max_distance_valid(false);
  radar_conf_.set_sensor_id_valid(false);
  radar_conf_.set_radar_power_valid(false);
  radar_conf_.set_output_type_valid(true);
  radar_conf_.set_send_quality_valid(true);
  radar_conf_.set_send_ext_info_valid(true);
  radar_conf_.set_sort_index_valid(false);
  radar_conf_.set_store_in_nvm_valid(false);
  radar_conf_.set_rcs_threshold_valid(true);
  radar_conf_.set_baudrate_valid(true);
  radar_conf_.set_interface_select_valid(false);
  radar_conf_.set_lvds_valid(false);
  radar_conf_.set_calibration_valid(false);

  radar_conf_.set_max_distance(125);
  radar_conf_.set_sensor_id(0);
  radar_conf_.set_output_type(NanoRadarState_201::OUTPUT_TYPE_NONE);
  radar_conf_.set_radar_power(0);
  radar_conf_.set_send_ext_info(1);
  radar_conf_.set_send_quality(1);
  radar_conf_.set_sort_index(NanoRadarState_201::SORT_INDEX_NO_SORTING);
  radar_conf_.set_store_in_nvm(0);
  radar_conf_.set_rcs_threshold(NanoRadarState_201::RCS_THRESHOLD_STANDARD);
  radar_conf_.set_baudrate(NanoRadarState_201::CAN_BAUDRATE_500K);
  radar_conf_.set_interface_type(0);
  radar_conf_.set_lvds_select(false);
  radar_conf_.set_calibration_enabled(
      NanoRadarState_201::CALIBRATION_ENABLED_ENABLED);
}

RadarConf RadarConfig200::radar_conf() { return radar_conf_; }

RadarConfig200* RadarConfig200::set_radar_conf(RadarConf radar_conf) {
  radar_conf_.CopyFrom(radar_conf);
  return this;
}

RadarConfig200* RadarConfig200::set_max_distance_valid(bool valid) {
  radar_conf_.set_max_distance_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_sensor_id_valid(bool valid) {
  radar_conf_.set_sensor_id_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_radar_power_valid(bool valid) {
  radar_conf_.set_radar_power_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_output_type_valid(bool valid) {
  radar_conf_.set_output_type_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_send_quality_valid(bool valid) {
  radar_conf_.set_send_quality_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_send_ext_info_valid(bool valid) {
  radar_conf_.set_send_ext_info_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_sort_index_valid(bool valid) {
  radar_conf_.set_sort_index_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_store_in_nvm_valid(bool valid) {
  radar_conf_.set_store_in_nvm_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_rcs_threshold_valid(bool valid) {
  radar_conf_.set_rcs_threshold_valid(valid);
  return this;
}

RadarConfig200* RadarConfig200::set_max_distance(uint16_t data) {
  radar_conf_.set_max_distance(data);
  return this;
}

RadarConfig200* RadarConfig200::set_sensor_id(uint8_t data) {
  radar_conf_.set_sensor_id(data);
  return this;
}

RadarConfig200* RadarConfig200::set_output_type(
    NanoRadarState_201::OutputType type) {
  radar_conf_.set_output_type(type);
  return this;
}

RadarConfig200* RadarConfig200::set_radar_power(uint8_t data) {
  radar_conf_.set_radar_power(data);
  return this;
}

RadarConfig200* RadarConfig200::set_send_ext_info(uint8_t data) {
  radar_conf_.set_send_ext_info(data);
  return this;
}

RadarConfig200* RadarConfig200::set_send_quality(uint8_t data) {
  radar_conf_.set_send_quality(data);
  return this;
}

RadarConfig200* RadarConfig200::set_sort_index(
    NanoRadarState_201::SortIndex data) {
  radar_conf_.set_sort_index(data);
  return this;
}

RadarConfig200* RadarConfig200::set_store_in_nvm(uint8_t data) {
  radar_conf_.set_store_in_nvm(data);
  return this;
}

RadarConfig200* RadarConfig200::set_rcs_threshold(
    NanoRadarState_201::RcsThreshold rcs_theshold) {
  radar_conf_.set_rcs_threshold(rcs_theshold);
  return this;
}

RadarConfig200* RadarConfig200::set_interface_select_valid(bool valid) {
  radar_conf_.set_interface_select_valid(valid);
  return this;
}
RadarConfig200* RadarConfig200::set_lvds_valid(bool valid) {
  radar_conf_.set_lvds_valid(valid);
  return this;
}
RadarConfig200* RadarConfig200::set_calibration_valid(bool valid) {
  radar_conf_.set_calibration_valid(valid);
  return this;
}
RadarConfig200* RadarConfig200::set_interface_type(uint8_t data) {
  radar_conf_.set_interface_type(data);
  return this;
}
RadarConfig200* RadarConfig200::set_lvds_select(bool data) {
  radar_conf_.set_lvds_select(data);
  return this;
}
RadarConfig200* RadarConfig200::set_calibration_enabled(
    NanoRadarState_201::CalibrationEnabled data) {
  radar_conf_.set_calibration_enabled(data);
  return this;
}

void RadarConfig200::set_max_distance_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

void RadarConfig200::set_sensor_id_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_bit_1(1);
  } else {
    frame.set_bit_0(1);
  }
}

void RadarConfig200::set_radar_power_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 2, 1);
  } else {
    frame.set_value(0, 2, 1);
  }
}

void RadarConfig200::set_output_type_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 3, 1);
  } else {
    frame.set_value(0, 3, 1);
  }
}

void RadarConfig200::set_send_quality_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 4, 1);
  } else {
    frame.set_value(0, 4, 1);
  }
}

void RadarConfig200::set_send_ext_info_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 5, 1);
  } else {
    frame.set_value(0, 5, 1);
  }
}

void RadarConfig200::set_sort_index_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 6, 1);
  } else {
    frame.set_value(0, 6, 1);
  }
}

void RadarConfig200::set_store_in_nvm_valid_p(uint8_t* data, bool valid) {
  Byte frame(data);
  if (valid) {
    frame.set_value(1, 7, 1);
  } else {
    frame.set_value(0, 7, 1);
  }
}

void RadarConfig200::set_rcs_threshold_valid_p(uint8_t* data, bool valid) {
  Byte frame(data + 6);
  if (valid) {
    frame.set_bit_1(0);
  } else {
    frame.set_bit_0(0);
  }
}

// void RadarConfig200::set_max_distance_p(uint8_t* data, uint16_t value) {
//   value /= 2;
//   uint8_t low = static_cast<uint8_t>(value >> 2);
//   Byte frame_low(data + 1);
//   frame_low.set_value(low, 0, 8);

//   uint8_t high = static_cast<uint8_t>(value << 6);
//   high &= 0xc0;
//   Byte frame_high(data + 2);
//   frame_high.set_value(high, 0, 8);
// }

void RadarConfig200::set_max_distance_p(uint8_t* data, uint16_t value) {
  uint16_t x = value / 2;
  uint8_t t = 0;

  t = x & 0x3;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 6, 2);
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

void RadarConfig200::set_sensor_id_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 4);
  frame.set_value(value, 0, 3);
}

void RadarConfig200::set_output_type_p(uint8_t* data,
                                       NanoRadarState_201::OutputType type) {
  Byte frame(data + 4);
  uint8_t value = static_cast<uint8_t>(type);
  frame.set_value(value, 3, 2);
}

void RadarConfig200::set_radar_power_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 4);
  frame.set_value(value, 5, 3);
}

void RadarConfig200::set_send_ext_info_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 3, 1);
}

void RadarConfig200::set_send_quality_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 2, 1);
}

void RadarConfig200::set_sort_index_p(
    uint8_t* data, NanoRadarState_201::SortIndex sort_index) {
  Byte frame(data + 5);
  uint8_t value = static_cast<uint8_t>(sort_index);
  frame.set_value(value, 4, 3);
}

void RadarConfig200::set_store_in_nvm_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 5);
  frame.set_value(value, 7, 1);
}

void RadarConfig200::set_rcs_threshold_p(
    uint8_t* data, NanoRadarState_201::RcsThreshold rcs_threshold) {
  Byte frame(data + 6);
  uint8_t value = static_cast<uint8_t>(rcs_threshold);
  frame.set_value(value, 1, 3);
}

RadarConfig200* RadarConfig200::set_baudrate_valid(bool valid) {
  radar_conf_.set_baudrate_valid(valid);
  return this;
}

void RadarConfig200::set_baudrate_valid_p(uint8_t* data, bool valid) {
  Byte frame(data + 7);
  if (valid) {
    frame.set_value(1, 4, 1);
  } else {
    frame.set_value(0, 4, 1);
  }
}

RadarConfig200* RadarConfig200::set_baudrate(
    NanoRadarState_201::CANBaudrate data) {
  radar_conf_.set_baudrate(data);
  return this;
}

void RadarConfig200::set_baudrate_p(uint8_t* data,
                                    NanoRadarState_201::CANBaudrate baudrate) {
  Byte frame(data + 7);
  uint8_t value = static_cast<uint8_t>(baudrate);
  frame.set_value(value, 5, 3);
}
void RadarConfig200::set_interface_select_valid_p(uint8_t* data, bool valid) {
  Byte frame(data + 6);
  if (valid) {
    frame.set_value(1, 6, 1);
  } else {
    frame.set_value(0, 6, 1);
  }
}
void RadarConfig200::set_lvds_valid_p(uint8_t* data, bool valid) {
  Byte frame(data + 6);
  if (valid) {
    frame.set_value(1, 7, 1);
  } else {
    frame.set_value(0, 7, 1);
  }
}
void RadarConfig200::set_calibration_valid_p(uint8_t* data, bool valid) {
  Byte frame(data + 7);
  if (valid) {
    frame.set_value(1, 3, 1);
  } else {
    frame.set_value(0, 3, 1);
  }
}
void RadarConfig200::set_interface_type_p(uint8_t* data, uint8_t value) {
  Byte frame(data + 6);
  frame.set_value(value, 4, 2);
}
void RadarConfig200::set_lvds_select_p(uint8_t* data, bool value) {
  Byte frame(data + 7);
  if (value) {
    frame.set_value(1, 0, 1);
  } else {
    frame.set_value(0, 0, 1);
  }
}
void RadarConfig200::set_calibration_enabled_p(
    uint8_t* data, NanoRadarState_201::CalibrationEnabled calibration_enabled) {
  Byte frame(data + 7);
  uint8_t value = static_cast<uint8_t>(calibration_enabled);
  frame.set_value(value, 2, 2);
}

}  // namespace nano_radar
}  // namespace drivers
}  // namespace apollo
