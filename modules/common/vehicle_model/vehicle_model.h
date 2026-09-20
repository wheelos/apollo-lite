/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/common/vehicle_model/proto/vehicle_model_config.pb.h"
#include "modules/common/vehicle_model/vehicle_model_input.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"
#include "modules/common/vehicle_state/reference_point_transformer.h"
#include "modules/common/vehicle_state/vehicle_description.h"
#include "modules/common/vehicle_state/vehicle_geometry_model.h"

namespace apollo {
namespace common {

class VehicleModelImplementation;

// Stateless configured prediction facade.
//
// Pipeline:
//   input VehicleState at its declared reference point
//     -> transform to implementation canonical_reference_point()
//     -> Ackermann or 4WS state propagation
//     -> transform to the requested output reference point
//
// The facade owns immutable model/configuration objects only. It never stores
// the current vehicle state. VehicleState remains the system semantic state;
// VehicleModelInput contains actuator/model inputs for a prediction.
class VehicleModel {
 public:
  ~VehicleModel();

  VehicleModel(const VehicleModel&) = delete;
  VehicleModel& operator=(const VehicleModel&) = delete;

  // Creates an independent model instance. description and config must refer
  // to the same physical vehicle.
  static Status Create(const VehicleModelConfig& config,
                       const VehicleDescription& description,
                       std::unique_ptr<VehicleModel>* vehicle_model);

  // Loads config_file and uses the process vehicle description.
  static Status CreateFromFile(const std::string& config_file,
                               std::unique_ptr<VehicleModel>* vehicle_model);

  ReferencePoint canonical_reference_point() const;

  // Returns the geometry model whose trajectory-point anchor
  // (trajectory_reference_point()) matches this model's
  // canonical_reference_point(). Consumers that build vehicle footprints
  // from PathPoint/TrajectoryPoint produced by this VehicleModel must use
  // this instance instead of constructing their own, so that switching the
  // configured vehicle model (e.g. Ackermann -> four-wheel-steering) never
  // requires changes outside modules/common/vehicle_model /
  // modules/common/vehicle_state.
  const VehicleGeometryModel& geometry_model() const { return geometry_model_; }

  // Predicts with explicit actuator/model input. The input state may use any
  // supported reference point. The output is expressed at
  // target_reference_point.
  Status Predict(double predicted_time_horizon,
                 const VehicleState& current_state,
                 const VehicleModelInput& input,
                 ReferencePoint target_reference_point,
                 VehicleState* predicted_state) const;

  // Extrapolates the state while holding its current curvature and
  // longitudinal acceleration. This is intended for short timestamp alignment
  // and trajectory stitching, not for command-response prediction.
  Status PredictWithHeldCurvature(double predicted_time_horizon,
                                  const VehicleState& current_state,
                                  ReferencePoint target_reference_point,
                                  VehicleState* predicted_state) const;

  Status PredictPositionWithHeldCurvature(
      double predicted_time_horizon, const VehicleState& current_state,
      math::Vec2d* predicted_position) const;

 private:
  VehicleModel(std::unique_ptr<VehicleModelImplementation> implementation,
               const VehicleDescription& description);

  Status PredictCanonical(double predicted_time_horizon,
                          const VehicleState& current_state,
                          const VehicleModelInput* input,
                          ReferencePoint target_reference_point,
                          VehicleState* predicted_state) const;

  std::unique_ptr<VehicleModelImplementation> implementation_;
  ReferencePointTransformer transformer_;
  VehicleGeometryModel geometry_model_;
};

}  // namespace common
}  // namespace apollo
