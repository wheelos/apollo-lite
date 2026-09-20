/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "modules/common/vehicle_model/vehicle_model.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/strings/str_cat.h"

#include "cyber/common/file.h"
#include "modules/common/vehicle_model/vehicle_model_factory.h"

namespace apollo {
namespace common {

VehicleModel::VehicleModel(
    std::unique_ptr<VehicleModelImplementation> implementation,
    const VehicleDescription& description)
    : implementation_(std::move(implementation)),
      transformer_(description),
      geometry_model_(description, implementation_->canonical_reference_point()) {
}

VehicleModel::~VehicleModel() = default;

Status VehicleModel::Create(const VehicleModelConfig& config,
                            const VehicleDescription& description,
                            std::unique_ptr<VehicleModel>* vehicle_model) {
  if (vehicle_model == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "vehicle model output is null");
  }
  vehicle_model->reset();

  std::unique_ptr<VehicleModelImplementation> implementation;
  auto status =
      VehicleModelFactory::Create(config, description, &implementation);
  if (!status.ok()) {
    return status;
  }
  *vehicle_model = std::unique_ptr<VehicleModel>(new VehicleModel(
      std::move(implementation), description));
  return Status::OK();
}

Status VehicleModel::CreateFromFile(
    const std::string& config_file,
    std::unique_ptr<VehicleModel>* vehicle_model) {
  VehicleModelConfig config;
  if (!cyber::common::GetProtoFromFile(config_file, &config)) {
    return Status(
        ErrorCode::PLANNING_ERROR,
        absl::StrCat("Failed to load vehicle model config file ", config_file));
  }
  return Create(config, VehicleDescription(), vehicle_model);
}

ReferencePoint VehicleModel::canonical_reference_point() const {
  return implementation_->canonical_reference_point();
}

Status VehicleModel::Predict(const double predicted_time_horizon,
                             const VehicleState& current_state,
                             const VehicleModelInput& input,
                             const ReferencePoint target_reference_point,
                             VehicleState* predicted_state) const {
  return PredictCanonical(predicted_time_horizon, current_state, &input,
                          target_reference_point, predicted_state);
}

Status VehicleModel::PredictWithHeldCurvature(
    const double predicted_time_horizon, const VehicleState& current_state,
    const ReferencePoint target_reference_point,
    VehicleState* predicted_state) const {
  return PredictCanonical(predicted_time_horizon, current_state, nullptr,
                          target_reference_point, predicted_state);
}

Status VehicleModel::PredictCanonical(
    const double predicted_time_horizon, const VehicleState& current_state,
    const VehicleModelInput* input, const ReferencePoint target_reference_point,
    VehicleState* predicted_state) const {
  if (predicted_state == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "predicted state is null");
  }
  if (predicted_time_horizon < 0.0) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "prediction horizon must be nonnegative");
  }

  VehicleState canonical_state;
  auto status =
      transformer_.TransformState(current_state, canonical_reference_point(),
                                  &canonical_state);
  if (!status.ok()) {
    return status;
  }

  VehicleState predicted_canonical_state;
  if (input == nullptr) {
    status = implementation_->PredictWithHeldCurvature(
        predicted_time_horizon, canonical_state, &predicted_canonical_state);
  } else {
    status = implementation_->Predict(predicted_time_horizon, canonical_state,
                                      *input, &predicted_canonical_state);
  }
  if (!status.ok()) {
    return status;
  }

  return transformer_.TransformState(predicted_canonical_state,
                                     target_reference_point, predicted_state);
}

Status VehicleModel::PredictPositionWithHeldCurvature(
    const double predicted_time_horizon, const VehicleState& current_state,
    math::Vec2d* predicted_position) const {
  if (predicted_position == nullptr) {
    return Status(ErrorCode::PLANNING_ERROR, "predicted position is null");
  }
  VehicleState predicted_state;
  const auto status = PredictWithHeldCurvature(
      predicted_time_horizon, current_state, current_state.reference_point(),
      &predicted_state);
  if (!status.ok()) {
    return status;
  }
  *predicted_position = math::Vec2d(predicted_state.x(), predicted_state.y());
  return Status::OK();
}

}  // namespace common
}  // namespace apollo
