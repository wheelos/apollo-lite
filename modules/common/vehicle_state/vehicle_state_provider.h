/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#pragma once

#include <string>

#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "wheelos_msgs/chassis_msgs/chassis.pb.h"
#include "wheelos_msgs/localization_msgs/localization.pb.h"

#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/vehicle_description.h"

namespace apollo {
namespace common {

// VehicleStateProvider is the only owner of the aggregation boundary:
// - Update() receives one localization/chassis input cycle.
// - Motion and operating state are constructed independently, aligned, and
//   merged into one committed VehicleState snapshot.
// - state() is the only public state accessor. Consumers must not reconstruct
//   state from Localization or Chassis.
class VehicleStateProvider {
 public:
  Status Update(const localization::LocalizationEstimate& localization,
                const canbus::Chassis& chassis);

  // Returns the latest successfully committed state snapshot.
  // The returned object is read-only through this API. Its contents may be
  // replaced by a later successful Update() call. Callers that retain a state
  // across update cycles must make a copy.
  const VehicleState& state() const;

  // Returns true after at least one complete state snapshot has been
  // successfully committed. A failed later Update() does not invalidate the
  // previously committed snapshot.
  bool HasValidState() const;

  ~VehicleStateProvider() = default;

 private:
  bool ConstructMotionState(
      const localization::LocalizationEstimate& localization,
      VehicleMotionState* motion_state) const;

  bool ConstructOperatingState(const canbus::Chassis& chassis, double timestamp,
                               VehicleOperatingState* operating_state) const;

  bool MergeStates(const VehicleMotionState& motion_state,
                   const VehicleOperatingState& operating_state,
                   VehicleState* state) const;

  // Internal state layers. Only the merged VehicleState crosses the public
  // module boundary.
  VehicleMotionState motion_state_snapshot_;
  VehicleOperatingState operating_state_snapshot_;
  VehicleState state_snapshot_;
  bool has_valid_state_ = false;
};

}  // namespace common
}  // namespace apollo
