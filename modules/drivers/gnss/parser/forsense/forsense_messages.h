/******************************************************************************
 * Copyright 2025 Pride Leong. All Rights Reserved.
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
#include <cstdint>
#include <string>

#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace forsense {

constexpr std::string_view BINARY_HEADER = "\xaa\x55";
const size_t BINARY_HEADER_LENGTH = 2;
const size_t BINARY_FRAME_ID_LENGTH = 2;
const size_t BINARY_FRAME_LENGTH_LENGTH = 2;
const size_t BINARY_CRC_LENGTH = 4;
constexpr std::string_view FRAME_TERMINATOR = "\r\n";
const size_t NMEA_CRC_LENGTH = 2;
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

struct GPATT {
  static constexpr char header[] = "$GNATT";
  // format hhmmss.sss
  double time;
  char status;
  double roll_angle;
  char indicator_of_roll;
  double pitch_angle;
  char indicator_of_pitch;
  double heading_angle;
  double roll_angle_uncertainty;
  double pitch_angle_uncertainty;
  double heading_angle_uncertainty;
};

// compatible with $GPCHC and $GPCHCX
struct GPYJ {
  static constexpr char header[] = "$GPYJ";
  // GPS week number since 1980-1-6
  uint32_t gps_week;
  // Seconds since the start of the current GPS week
  double gps_time;
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
  uint32_t nsv1 = 0;
  // Number of satellites for secondary antenna
  uint32_t nsv2 = 0;
  // status for nmea format
  Status status;
  uint32_t age = 0;
  // Warning flags:
  std::string warning_cs;

  // additional fields for $GPCHCX format
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

  // additional fields for binary integrated navigation data
  // IMU temperature in degrees Celsius
  double imu_temperature = 0.0;
  uint8_t rtk_status;
  uint8_t orientation_status;
  uint32_t position_accuracy_factor;
  // bit0: 1: rtk data valid, 0: rtk data invalid
  // bit1: 1: pps signal valid, 0: pps signal invalid
  // bit2: 1: integrated navigation initialized, 0: not initialized
  // bit3: 1: steer of front wheel valid, 0: not valid
  // bit4: 1: integrated navigation converged, 0: not converged
  // bit5: 1: front gyro valid, 0: not valid
  // bit6: 1: steering motor valid, 0: not valid
  // bit7, bit8:
  //  01(bit7=1, bit8=0): vehicle forwarding
  //  10(bit7=0, bit8=1): vehicle backward
  //  00(bit7=0, bit8=0): invalid
  uint16_t combined_status;
};

void FillGnssBestpos(const forsense::GPYJ& gpyj, GnssBestPose* bestpos);
void FillImu(const forsense::GPYJ& gpyj, Imu* imu);
void FillHeading(const forsense::GPYJ& gpyj, Heading* heading);
void FillIns(const forsense::GPYJ& gpyj, Ins* ins);
void FillInsStat(const forsense::GPYJ& gpyj, InsStat* ins_stat);

}  // namespace forsense
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
