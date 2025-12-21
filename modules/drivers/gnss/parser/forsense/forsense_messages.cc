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
#include "modules/drivers/gnss/parser/forsense/forsense_messages.h"

#include <cstdint>
#include <string>
#include <vector>

#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/util.h"
#include "modules/drivers/gnss/util/util.h"

namespace apollo {
namespace drivers {
namespace gnss {
namespace forsense {

using apollo::drivers::gnss::SolutionStatus;
using apollo::drivers::gnss::SolutionType;

SolutionStatus ToSolutionStatus(SystemStatus sys_status) {
  switch (sys_status) {
    case SystemStatus::INIT:
      // System is in initialization state, most common solution status is not
      // yet converged or waiting for data. PENDING means still computing.
      // COLD_START means just started.
      return SolutionStatus::COLD_START;
      // Alternatively, consider SolutionStatus::PENDING;

    case SystemStatus::GUIDANCE:
      // In guidance mode, if all is normal, a solution should be computed.
      // But in practice, there may be issues. Here we assume the "successful"
      // case.
      return SolutionStatus::SOL_COMPUTED;

    case SystemStatus::COMBINED:
      // In combined navigation mode, the solution is usually reliable and
      // computed. Here we also assume the "successful" case.
      return SolutionStatus::SOL_COMPUTED;

    case SystemStatus::INERTIAL:
      // Pure inertial mode, meaning GNSS signal may be lost or unavailable.
      // SolutionStatus does not have a direct "pure inertial dead reckoning"
      // state. If GNSS data is insufficient, could map to INSUFFICIENT_OBS.
      // More accurately, a new SolutionStatus value may be needed for "dead
      // reckoning, GNSS unavailable". For now, we assume a solution is still
      // "computed" (though accuracy may degrade). Another consideration: if
      // inertial solution quality degrades, COV_TRACE or VARIANCE may occur. To
      // avoid misleading, we choose a "computing but quality not guaranteed"
      // status.
      return SolutionStatus::
          SOL_COMPUTED;  // This mapping is debatable, as it does not reflect
                         // inertial drift. Ideally, SolutionStatus may need to
                         // be extended. For now, use SOL_COMPUTED to indicate a
                         // solution is output, but check other info for
                         // quality.

    default:
      // Catch all unhandled SystemStatus values.
      // This may indicate the system is in an unknown or unauthorized state.
      AWARN << "Unhandled SystemStatus value for SolutionStatus: "
            << static_cast<int>(sys_status);
      return SolutionStatus::UNAUTHORIZED;
  }
}

SolutionType ToSolutionType(SatelliteStatus sat_status) {
  switch (sat_status) {
    case SatelliteStatus::NO_POS_NO_ORIENT:
      // No position or attitude information, closest is NONE
      return SolutionType::NONE;

    case SatelliteStatus::SINGLE_POS_ORIENT:
    case SatelliteStatus::SINGLE_POS_NO_ORIENT:
      // Single point positioning, with or without attitude
      return SolutionType::SINGLE;

    case SatelliteStatus::PSEUDORANGE_DIFF_ORIENT:
    case SatelliteStatus::PSEUDORANGE_DIFF_NO_ORIENT:
      // Pseudorange differential positioning, with or without attitude
      return SolutionType::PSRDIFF;

    case SatelliteStatus::RTK_FLOAT_ORIENT:
      return SolutionType::INS_RTKFLOAT;
    case SatelliteStatus::RTK_FLOAT_NO_ORIENT:
      // RTK float solution, with or without attitude
      // SolutionType has several FLOAT types, here FLOATCONV is generic,
      // or if you know it's L1/IONOFREE/NARROW float, can be more precise.
      // But since SatelliteStatus just says "RTK_FLOAT", NARROW_FLOAT is a
      // reasonable generic representation. return SolutionType::FLOATCONV;
      // Alternatively: if focusing on RTK internal float, can use NARROW_FLOAT,
      // but FLOATCONV is more generic.
      return SolutionType::NARROW_FLOAT;

    case SatelliteStatus::RTK_STABLE_ORIENT:
      return SolutionType::INS_RTKFIXED;
    case SatelliteStatus::RTK_STABLE_NO_ORIENT:
      // RTK fixed solution, with or without attitude
      // SolutionType has several INT (Integer fixed) types, here NARROW_INT
      // represents RTK fixed solution, as RTK fixed is usually narrow-lane
      // fixed. L1_INT and WIDE_INT are also fixed solutions. If no more
      // detailed info, NARROW_INT is a common choice.
      return SolutionType::NARROW_INT;
      // Alternatively: for a more generic fixed solution, can use L1_INT or
      // WIDE_INT, or if there is a "best available fixed solution" concept, but
      // NARROW_INT usually means highest precision RTK fixed. return
      // SolutionType::L1_INT;

    case SatelliteStatus::COMBINED_PREDICTION:
      // This is a special type, may mean some form of prediction (e.g.,
      // inertial prediction) is fused. In SolutionType, PROPOGATED is closest
      // to "predicted". If "Combined Prediction" clearly involves INS, and you
      // want to map to INS-related types, may need to check deeper system
      // status. But from the name, PROPOGATED is most generic.
      return SolutionType::PROPOGATED;

    default:
      AWARN << "Unhandled SatelliteStatus value for SolutionType: "
            << static_cast<int>(sat_status);
      return SolutionType::NONE;
  }
}

void FillGnssBestpos(const forsense::GPYJ& gpyj, GnssBestPose* bestpos) {
  bestpos->set_measurement_time(gpyj.gps_week * kSecondsPerWeek +
                                gpyj.gps_time);
  // Use mapping helpers
  bestpos->set_sol_status(ToSolutionStatus(gpyj.status.get_system_status()));
  bestpos->set_sol_type(ToSolutionType(gpyj.status.get_satellite_status()));
  bestpos->set_latitude(gpyj.latitude);
  bestpos->set_longitude(gpyj.longitude);
  bestpos->set_height_msl(gpyj.altitude);

  // bestpos->set_latitude_std_dev(gpyj.latitude_std);
  // bestpos->set_longitude_std_dev(gpyj.longitude_std);
  // bestpos->set_height_std_dev(gpyj.altitude_std);

  // Fill satellite counts
  bestpos->set_num_sats_tracked(gpyj.nsv1 + gpyj.nsv2);
}

void FillImu(const forsense::GPYJ& gpyj, Imu* imu) {
  imu->set_measurement_time(gpyj.gps_week * kSecondsPerWeek + gpyj.gps_time);

  apollo::common::Point3D* linear_acceleration =
      imu->mutable_linear_acceleration();
  // Ensure correct coordinate transform (RFU to FLU)
  z_rot_90_ccw(gpyj.acc_x * kAccelerationGravity,
               gpyj.acc_y * kAccelerationGravity,
               gpyj.acc_z * kAccelerationGravity, linear_acceleration);

  apollo::common::Point3D* angular_velocity = imu->mutable_angular_velocity();
  // Ensure correct coordinate transform (RFU to FLU)
  z_rot_90_ccw(gpyj.gyro_x * kDegToRad, gpyj.gyro_y * kDegToRad,
               gpyj.gyro_z * kDegToRad, angular_velocity);
}

void FillHeading(const forsense::GPYJ& gpyj, Heading* heading) {
  heading->set_measurement_time(gpyj.gps_week * kSecondsPerWeek +
                                gpyj.gps_time);
  heading->set_solution_status(
      ToSolutionStatus(gpyj.status.get_system_status()));
  heading->set_position_type(
      ToSolutionType(gpyj.status.get_satellite_status()));
  heading->set_heading(gpyj.heading);
  heading->set_pitch(gpyj.pitch);

  // heading->set_heading_std_dev(gpyj.heading_std);
  // heading->set_pitch_std_dev(gpyj.pitch_std);
}

void FillIns(const forsense::GPYJ& gpyj, Ins* ins) {
  // FIX: Use GPS time for Protobuf header timestamp as well
  double gps_time_sec = gpyj.gps_week * kSecondsPerWeek + gpyj.gps_time;
  ins->mutable_header()->set_timestamp_sec(gps_time_sec);  // Use sensor time

  ins->set_measurement_time(gps_time_sec);

  // Map SolutionType to Ins::Type
  SolutionType solution_type =
      ToSolutionType(gpyj.status.get_satellite_status());
  switch (solution_type) {
    // Map relevant SolutionTypes to Ins::Type
    case SolutionType::INS_RTKFIXED:
    case SolutionType::NARROW_INT:
    case SolutionType::INS_RTKFLOAT:
    case SolutionType::NARROW_FLOAT:
      ins->set_type(Ins::GOOD);
      break;
    case SolutionType::SINGLE:
      ins->set_type(Ins::CONVERGING);
      break;
    case SolutionType::WIDELANE:       // Often implies float RTK
    case SolutionType::FLOATCONV:      // Explicit float RTK
      ins->set_type(Ins::CONVERGING);  // Or GOOD_FLOAT? Check Ins::Type options
      break;
    case SolutionType::RTK_DIRECT_INS:
      ins->set_type(Ins::GOOD);
      break;
    default:
      ins->set_type(Ins::INVALID);  // Handle unknown or invalid statuses
      break;
  }

  apollo::common::PointLLH* position = ins->mutable_position();
  position->set_lon(gpyj.longitude);
  position->set_lat(gpyj.latitude);
  position->set_height(gpyj.altitude);

  apollo::common::Point3D* euler_angles = ins->mutable_euler_angles();
  // Ensure correct mapping and units (Roll/Pitch/Heading vs Euler angles)
  // Check coordinate system conversions (RFU to FLU, Azimuth to Yaw)
  euler_angles->set_x(gpyj.roll * kDegToRad);
  // Check pitch sign convention based on Apollo FLU
  euler_angles->set_y(-gpyj.pitch * kDegToRad);
  // Assuming heading is Azimuth 0-360 North=0, East=90
  // to Apollo Yaw
  euler_angles->set_z(azimuth_deg_to_yaw_rad(gpyj.heading));

  apollo::common::Point3D* linear_velocity = ins->mutable_linear_velocity();
  // Check mapping of Ve/Vn/Vu to FLU velocity components (East, North, Up) ->
  // (X, Y, Z)
  linear_velocity->set_x(gpyj.velocity_east);
  linear_velocity->set_y(gpyj.velocity_north);
  linear_velocity->set_z(gpyj.velocity_up);

  apollo::common::Point3D* angular_velocity = ins->mutable_angular_velocity();
  z_rot_90_ccw(gpyj.gyro_x * kDegToRad, gpyj.gyro_y * kDegToRad,
               gpyj.gyro_z * kDegToRad, angular_velocity);

  apollo::common::Point3D* linear_acceleration =
      ins->mutable_linear_acceleration();
  z_rot_90_ccw(gpyj.acc_x * kAccelerationGravity,
               gpyj.acc_y * kAccelerationGravity,
               gpyj.acc_z * kAccelerationGravity, linear_acceleration);
}

void FillInsStat(const forsense::GPYJ& gpyj, InsStat* ins_stat) {
  ins_stat->set_ins_status(
      static_cast<uint32_t>(gpyj.status.get_system_status()));
}

}  // namespace forsense
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
