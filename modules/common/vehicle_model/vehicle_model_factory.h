// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include <memory>

#include "modules/common/status/status.h"
#include "modules/common/vehicle_model/proto/vehicle_model_config.pb.h"
#include "modules/common/vehicle_model/vehicle_model_input.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/common/vehicle_state/vehicle_description.h"

namespace apollo {
namespace common {

// Internal model contract. Implementations receive and return states at
// canonical_reference_point(). They must not retain either state or input.
class VehicleModelImplementation {
 public:
  virtual ~VehicleModelImplementation() = default;

  virtual VehicleReferencePoint canonical_reference_point() const = 0;

  virtual Status Predict(double predicted_time_horizon,
                         const VehicleState& canonical_state,
                         const VehicleModelInput& input,
                         VehicleState* predicted_canonical_state) const = 0;

  virtual Status PredictWithHeldCurvature(
      double predicted_time_horizon, const VehicleState& canonical_state,
      VehicleState* predicted_canonical_state) const = 0;
};

// Front-wheel-steered kinematic bicycle model. The rear axle is the canonical
// point, matching the no-slip bicycle equations.
class AckermannKinematicModel final : public VehicleModelImplementation {
 public:
  VehicleReferencePoint canonical_reference_point() const override {
    return REAR_AXLE_CENTER;
  }

  Status Predict(double predicted_time_horizon,
                 const VehicleState& canonical_state,
                 const VehicleModelInput& input,
                 VehicleState* predicted_canonical_state) const override;

  Status PredictWithHeldCurvature(
      double predicted_time_horizon, const VehicleState& canonical_state,
      VehicleState* predicted_canonical_state) const override;

 private:
  friend class VehicleModelFactory;
  AckermannKinematicModel(double dt, double wheel_base,
                          double max_road_wheel_angle, double max_acceleration,
                          double max_deceleration);

  double dt_;
  double wheel_base_;
  double max_road_wheel_angle_;
  double max_acceleration_;
  double max_deceleration_;
};

// Four-wheel-steering kinematic bicycle model. The center of mass is the
// canonical point so the model can propagate the CG lateral velocity implied
// by front and rear steering.
class FourWheelSteeringKinematicModel final
    : public VehicleModelImplementation {
 public:
  VehicleReferencePoint canonical_reference_point() const override {
    return CENTER_OF_MASS;
  }

  Status Predict(double predicted_time_horizon,
                 const VehicleState& canonical_state,
                 const VehicleModelInput& input,
                 VehicleState* predicted_canonical_state) const override;

  Status PredictWithHeldCurvature(
      double predicted_time_horizon, const VehicleState& canonical_state,
      VehicleState* predicted_canonical_state) const override;

 private:
  friend class VehicleModelFactory;
  FourWheelSteeringKinematicModel(double dt, double wheel_base,
                                  double center_of_mass_offset,
                                  double max_road_wheel_angle,
                                  double max_acceleration,
                                  double max_deceleration);

  double dt_;
  double wheel_base_;
  double distance_cg_to_rear_axle_;
  double distance_cg_to_front_axle_;
  double max_road_wheel_angle_;
  double max_acceleration_;
  double max_deceleration_;
};

class VehicleModelFactory {
 public:
  static Status Create(const VehicleModelConfig& config,
                       const VehicleDescription& description,
                       std::unique_ptr<VehicleModelImplementation>* model);
};

}  // namespace common
}  // namespace apollo
