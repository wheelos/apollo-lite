// Copyright 2026 WheelOS. All Rights Reserved.
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

//  Created Date: 2026-09-06
//  Author: daohu527

#include "modules/localization_health/reason_aggregator.h"

namespace apollo {
namespace localization {

namespace {

// Ranked priority of reasons (safety impact order):
// 1. Time, Comm & Data Integrity
// 2. Numerical & Estimator Failure
// 3. Local Localization & Kinematics
// 4. Global Localization & Map
// 5. Single Sensor Faults
// 6. Recovery Process Info
const HealthReason kPriorityReasons[] = {
    // 1. Time, Comm & Data Integrity
    REASON_ASSESSMENT_TIMEOUT,
    REASON_POSE_TIMEOUT,
    REASON_TIMESTAMP_REGRESSION,
    REASON_DATA_TOO_OLD,
    REASON_SEQUENCE_ERROR,
    REASON_SESSION_CHANGED,
    REASON_TIME_SYNC_ERROR,

    // 2. Numerical & Estimator Failure
    REASON_NON_FINITE_OUTPUT,
    REASON_INVALID_QUATERNION,
    REASON_INVALID_COVARIANCE,
    REASON_KINEMATIC_VIOLATION,
    REASON_UNDECLARED_POSE_JUMP,
    REASON_ESTIMATOR_NOT_RUNNING,
    REASON_ESTIMATOR_NOT_CONVERGED,

    // 3. Local Localization
    REASON_LOCAL_POSE_INVALID,
    REASON_LIO_DIVERGED,
    REASON_LIO_DEGENERATED,
    REASON_IMU_SATURATION,
    REASON_WHEEL_SLIP,
    REASON_INSUFFICIENT_FEATURES,

    // 4. Global Localization & Map
    REASON_GLOBAL_POSE_INVALID,
    REASON_MAP_MATCH_FAILED,
    REASON_MAP_MISMATCH,
    REASON_MAP_STALE,
    REASON_LANE_LEVEL_UNAVAILABLE,

    // 5. Single Sensor Faults
    REASON_GNSS_UNAVAILABLE,
    REASON_GNSS_MULTIPATH,

    // 6. Recovery Process Info
    REASON_RELOCALIZATION_FAILED,
    REASON_RELOCALIZATION_TIMEOUT,
    REASON_RELOCALIZATION_AMBIGUOUS,
    REASON_RELOCALIZATION_REQUIRED,
    REASON_GLOBAL_ALIGNMENT_IN_PROGRESS,
    REASON_RECOVERY_VERIFICATION_FAILED,
};

}  // namespace

void ReasonAggregator::Reset() {
  active_reasons_ = 0;
  latched_reasons_ = 0;
  primary_reason_ = REASON_NONE;
}

void ReasonAggregator::Update(uint64_t independent_reasons,
                              uint64_t algorithm_faults) {
  active_reasons_ = independent_reasons | algorithm_faults;

  // Latch severe reasons during the session
  for (const auto& reason : kPriorityReasons) {
    if (IsHardFault(reason) && HasReason(active_reasons_, reason)) {
      latched_reasons_ |= ReasonToBit(reason);
    }
  }
  if (HasReason(active_reasons_, REASON_KINEMATIC_VIOLATION)) {
    latched_reasons_ |= ReasonToBit(REASON_KINEMATIC_VIOLATION);
  }
  if (HasReason(active_reasons_, REASON_INVALID_COVARIANCE)) {
    latched_reasons_ |= ReasonToBit(REASON_INVALID_COVARIANCE);
  }
  if (HasReason(active_reasons_, REASON_RELOCALIZATION_FAILED)) {
    latched_reasons_ |= ReasonToBit(REASON_RELOCALIZATION_FAILED);
  }

  primary_reason_ = SelectPrimaryReason(active_reasons_);
}

int ReasonAggregator::ReasonToBitIndex(HealthReason reason) {
  switch (reason) {
    case REASON_ASSESSMENT_TIMEOUT:
      return 0;
    case REASON_POSE_TIMEOUT:
      return 1;
    case REASON_TIMESTAMP_REGRESSION:
      return 2;
    case REASON_DATA_TOO_OLD:
      return 3;
    case REASON_SEQUENCE_ERROR:
      return 4;
    case REASON_SESSION_CHANGED:
      return 5;
    case REASON_TIME_SYNC_ERROR:
      return 6;
    case REASON_NON_FINITE_OUTPUT:
      return 7;
    case REASON_INVALID_QUATERNION:
      return 8;
    case REASON_INVALID_COVARIANCE:
      return 9;
    case REASON_KINEMATIC_VIOLATION:
      return 10;
    case REASON_UNDECLARED_POSE_JUMP:
      return 11;
    case REASON_ESTIMATOR_NOT_RUNNING:
      return 12;
    case REASON_ESTIMATOR_NOT_CONVERGED:
      return 13;
    case REASON_LOCAL_POSE_INVALID:
      return 14;
    case REASON_LIO_DIVERGED:
      return 15;
    case REASON_LIO_DEGENERATED:
      return 16;
    case REASON_IMU_SATURATION:
      return 17;
    case REASON_WHEEL_SLIP:
      return 18;
    case REASON_INSUFFICIENT_FEATURES:
      return 19;
    case REASON_GNSS_UNAVAILABLE:
      return 20;
    case REASON_GNSS_MULTIPATH:
      return 21;
    case REASON_GLOBAL_POSE_INVALID:
      return 22;
    case REASON_MAP_MATCH_FAILED:
      return 23;
    case REASON_MAP_MISMATCH:
      return 24;
    case REASON_MAP_STALE:
      return 25;
    case REASON_LANE_LEVEL_UNAVAILABLE:
      return 26;
    case REASON_RELOCALIZATION_REQUIRED:
      return 27;
    case REASON_RELOCALIZATION_AMBIGUOUS:
      return 28;
    case REASON_RELOCALIZATION_TIMEOUT:
      return 29;
    case REASON_RELOCALIZATION_FAILED:
      return 30;
    case REASON_GLOBAL_ALIGNMENT_IN_PROGRESS:
      return 31;
    case REASON_RECOVERY_VERIFICATION_FAILED:
      return 32;
    default:
      return -1;
  }
}

HealthReason ReasonAggregator::BitIndexToReason(int bit_index) {
  switch (bit_index) {
    case 0:
      return REASON_ASSESSMENT_TIMEOUT;
    case 1:
      return REASON_POSE_TIMEOUT;
    case 2:
      return REASON_TIMESTAMP_REGRESSION;
    case 3:
      return REASON_DATA_TOO_OLD;
    case 4:
      return REASON_SEQUENCE_ERROR;
    case 5:
      return REASON_SESSION_CHANGED;
    case 6:
      return REASON_TIME_SYNC_ERROR;
    case 7:
      return REASON_NON_FINITE_OUTPUT;
    case 8:
      return REASON_INVALID_QUATERNION;
    case 9:
      return REASON_INVALID_COVARIANCE;
    case 10:
      return REASON_KINEMATIC_VIOLATION;
    case 11:
      return REASON_UNDECLARED_POSE_JUMP;
    case 12:
      return REASON_ESTIMATOR_NOT_RUNNING;
    case 13:
      return REASON_ESTIMATOR_NOT_CONVERGED;
    case 14:
      return REASON_LOCAL_POSE_INVALID;
    case 15:
      return REASON_LIO_DIVERGED;
    case 16:
      return REASON_LIO_DEGENERATED;
    case 17:
      return REASON_IMU_SATURATION;
    case 18:
      return REASON_WHEEL_SLIP;
    case 19:
      return REASON_INSUFFICIENT_FEATURES;
    case 20:
      return REASON_GNSS_UNAVAILABLE;
    case 21:
      return REASON_GNSS_MULTIPATH;
    case 22:
      return REASON_GLOBAL_POSE_INVALID;
    case 23:
      return REASON_MAP_MATCH_FAILED;
    case 24:
      return REASON_MAP_MISMATCH;
    case 25:
      return REASON_MAP_STALE;
    case 26:
      return REASON_LANE_LEVEL_UNAVAILABLE;
    case 27:
      return REASON_RELOCALIZATION_REQUIRED;
    case 28:
      return REASON_RELOCALIZATION_AMBIGUOUS;
    case 29:
      return REASON_RELOCALIZATION_TIMEOUT;
    case 30:
      return REASON_RELOCALIZATION_FAILED;
    case 31:
      return REASON_GLOBAL_ALIGNMENT_IN_PROGRESS;
    case 32:
      return REASON_RECOVERY_VERIFICATION_FAILED;
    default:
      return REASON_NONE;
  }
}

uint64_t ReasonAggregator::ReasonToBit(HealthReason reason) {
  int index = ReasonToBitIndex(reason);
  if (index < 0 || index >= 64) {
    return 0ULL;
  }
  return 1ULL << index;
}

bool ReasonAggregator::HasReason(uint64_t reason_mask, HealthReason reason) {
  uint64_t bit = ReasonToBit(reason);
  return (reason_mask & bit) != 0;
}

void ReasonAggregator::AddReason(uint64_t* reason_mask, HealthReason reason) {
  if (reason_mask) {
    *reason_mask |= ReasonToBit(reason);
  }
}

bool ReasonAggregator::IsHardFault(HealthReason reason) {
  switch (reason) {
    case REASON_ASSESSMENT_TIMEOUT:
    case REASON_POSE_TIMEOUT:
    case REASON_TIMESTAMP_REGRESSION:
    case REASON_DATA_TOO_OLD:
    case REASON_NON_FINITE_OUTPUT:
    case REASON_INVALID_QUATERNION:
    case REASON_UNDECLARED_POSE_JUMP:
    case REASON_ESTIMATOR_NOT_RUNNING:
    case REASON_LOCAL_POSE_INVALID:
    case REASON_LIO_DIVERGED:
    case REASON_SESSION_CHANGED:
      return true;
    default:
      return false;
  }
}

HealthReason ReasonAggregator::SelectPrimaryReason(uint64_t active_reasons) {
  for (const auto& reason : kPriorityReasons) {
    if (HasReason(active_reasons, reason)) {
      return reason;
    }
  }
  return REASON_NONE;
}

}  // namespace localization
}  // namespace apollo
