// Copyright 2024 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2024-12-30
//  Author: daohu527

#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace apollo {
namespace drivers {
namespace gnss {
namespace huace {

// (Assuming FRAME_TERMINATOR is already defined as "\r\n")
constexpr std::string_view FRAME_TERMINATOR = "\r\n";
const size_t NUMA_CRC_LENGTH = 2;
const uint8_t NMEA_CHECKSUM_DELIMITER = '*';

enum class SystemStatus : uint8_t {
  INIT = 0x00,
  GUIDANCE = 0x01,
  COMBINED = 0x02,
  INERTIAL = 0x03
};

enum class SatelliteStatus : uint8_t {
  NO_POS_NO_ORIENT = 0x00,
  SINGLE_POS_ORIENT = 0x01,
  PSEUDORANGE_DIFF_ORIENT = 0x02,
  COMBINED_PREDICTION = 0x03,
  RTK_STABLE_ORIENT = 0x04,
  RTK_FLOAT_ORIENT = 0x05,
  SINGLE_POS_NO_ORIENT = 0x06,
  PSEUDORANGE_DIFF_NO_ORIENT = 0x07,
  RTK_STABLE_NO_ORIENT = 0x08,
  RTK_FLOAT_NO_ORIENT = 0x09
};

struct Status {
  uint8_t raw_value = 0;

  SatelliteStatus get_satellite_status() const {
    return static_cast<SatelliteStatus>((raw_value >> 4) & 0x0F);
  }

  SystemStatus get_system_status() const {
    return static_cast<SystemStatus>(raw_value & 0x0F);
  }

  void set_status(SatelliteStatus sat_status, SystemStatus sys_status) {
    raw_value = (static_cast<uint8_t>(sat_status) << 4) |
                static_cast<uint8_t>(sys_status);
  }
};

struct GPCHCBase {
  // GPS week number since 1980-1-6
  uint32_t gps_week;
  // Seconds since the start of the current GPS week
  double seconds_in_gps_week;
  // Heading angle (0 to 359.99) in degrees
  double heading;
  // Pitch angle (-90 to 90) in degrees
  double pitch;
  // Roll angle (-180 to 180) in degrees
  double roll;
  // Gyroscope X-axis
  double gyro_x;
  // Gyroscope Y-axis
  double gyro_y;
  // Gyroscope Z-axis
  double gyro_z;
  // Accelerometer X-axis
  double acc_x;
  // Accelerometer Y-axis
  double acc_y;
  // Accelerometer Z-axis
  double acc_z;
  // Latitude (-90° to 90°) in degrees
  double latitude;
  // Longitude (-180° to 180°) in degrees
  double longitude;
  // Altitude in meters
  double altitude;
  // Eastward velocity in m/s
  double velocity_east;
  // Northward velocity in m/s
  double velocity_north;
  // Upward velocity in m/s
  double velocity_up;
  // Vehicle speed in m/s
  double speed;
  // Number of satellites for main antenna
  uint32_t nsv1;
  // Number of satellites for secondary antenna
  uint32_t nsv2;
  Status status;
  uint32_t differential_age = 0;
  // Warning flags:
  std::string warning;
};

struct GPCHC : public GPCHCBase {
  // GPCHC protocol header, default value "$GPCHC"
  static constexpr char header[] = "$GPCHC";
};

struct GPCHCX : public GPCHCBase {
  // GPCHCX protocol header, default value "$GPCHCX"
  static constexpr char header[] = "$GPCHCX";

  // Latitude standard deviation, unit (meters)
  double latitude_std = 0.0;
  // Longitude standard deviation, unit (meters)
  double longitude_std = 0.0;
  // Altitude standard deviation, unit (meters)
  double altitude_std = 0.0;
  // Eastward velocity standard deviation, unit (m/s)
  double velocity_east_std = 0.0;
  // Northward velocity standard deviation, unit (m/s)
  double velocity_north_std = 0.0;
  // Upward velocity standard deviation, unit (m/s)
  double velocity_up_std = 0.0;

  // Roll angle standard deviation, unit (degrees)
  double roll_std = 0.0;
  // Pitch angle standard deviation, unit (degrees)
  double pitch_std = 0.0;
  // Heading angle standard deviation, unit (degrees)
  double heading_std = 0.0;

  char separator = 'X';

  // Speed heading (0 to 359.99) in degrees (accurate to two decimal places)
  double speed_heading = 0.0;
  // Speed heading standard deviation, unit (degrees), accurate to two decimal
  // places
  double speed_heading_std = 0.0;
  // Antenna position X-axis lever arm relative to the device, in vehicle
  // coordinate system, unit (meters), accurate to two decimal places
  double antenna_x = 0.0f;
  // Antenna position Y-axis lever arm relative to the device, in vehicle
  // coordinate system, unit (meters), accurate to two decimal places
  double antenna_y = 0.0f;
  // Antenna position Z-axis lever arm relative to the device, in vehicle
  // coordinate system, unit (meters), accurate to two decimal places
  double antenna_z = 0.0f;
  // Rotation Euler angle from device coordinate system to vehicle coordinate
  // system, X-axis angle, unit (degrees), accurate to two decimal places
  double rotation_angle_x = 0.0f;
  // Rotation Euler angle from device coordinate system to vehicle coordinate
  // system, Y-axis angle, unit (degrees), accurate to two decimal places
  double rotation_angle_y = 0.0f;
  // Rotation Euler angle from device coordinate system to vehicle coordinate
  // system, Z-axis angle, unit (degrees), accurate to two decimal places
  double rotation_angle_z = 0.0f;
  // Rotation angle from vehicle heading to GNSS heading direction, along
  // vehicle coordinate system Z-axis, unit (degrees), accurate to two decimal
  // places
  double direction_angle = 0.0f;
  // Number of satellites used for main antenna
  uint32_t nsu1;
  // Number of satellites used for secondary antenna
  uint32_t nsu2;
  // Device serial number (e.g., 6 chars + null terminator)
  std::string device_sn;
};

}  // namespace huace
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
