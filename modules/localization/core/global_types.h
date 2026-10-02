// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "modules/localization/core/types.h"

namespace apollo {
namespace localization {
namespace unified {

struct GlobalConfig {
  std::string clock_id = "unix";
  std::string map_id;
  std::string map_version;
  std::string calibration_id;
  double history_duration = 3.0;
  double max_interpolation_gap = 0.05;
  double max_observation_age = 2.0;
  double local_timeout = 0.2;
  double global_timeout = 2.0;
  double future_tolerance = 0.02;
  double innovation_gate = 22.458;
  double max_position_std = 2.0;
  double max_attitude_std = 0.2;
  uint32_t verification_frames = 3;
  double verification_position = 0.5;
  double verification_angle = 0.1;
  double position_drift_variance_rate = 0.01;
  double attitude_drift_variance_rate = 0.0001;
  bool enable_fixed_lag_graph = false;
  bool motion_error_model_qualified = false;
  double graph_solve_budget_ms = 50.0;
  double graph_robust_threshold = 2.5;
  uint32_t graph_max_states = 1000;
  uint32_t graph_max_observations = 2000;
  uint32_t graph_max_iterations = 5;
  double graph_max_linearization_angle = 0.2;
};

struct GlobalObservation {
  enum class Kind { POSE, POSITION, PROJECTED_POSE };
  Kind kind = Kind::POSE;
  Stamp stamp;
  Epoch epoch;
  std::string source;
  std::string map_id;
  std::string map_version;
  std::string calibration_id;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Matrix6d covariance = Matrix6d::Zero();
  // POSITION measures this body-fixed point, not an invented base orientation.
  Eigen::Vector3d point_in_base = Eigen::Vector3d::Zero();
  // Rows act on [translation_map (m), left rotation_map (rad)].
  Eigen::MatrixXd projection;
  Eigen::MatrixXd projected_covariance;
  bool quality_valid = false;
  bool georeferenced = false;
  bool recovery = false;
  bool ambiguous = false;
  bool independent_of_local = false;
};

struct GlobalState {
  Epoch epoch;
  Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
  Matrix6d covariance = Matrix6d::Zero();
  double last_observation = 0.0;
  double last_full_observation = 0.0;
  double last_georeference = 0.0;
  double covariance_time = 0.0;
  uint64_t correction_id = 0;
  bool valid = false;
  bool georeferenced = false;
  bool verifying = false;
};

struct GlobalEvidence {
  GlobalObservation observation;
  LocalState local;
};

Matrix6d TransformCovarianceBound(const Eigen::Isometry3d& map_base,
                                  const LocalState& local,
                                  const Matrix6d& observation_covariance);
Matrix6d GlobalPoseCovarianceBound(const GlobalState& global,
                                   const LocalState& local);

}  // namespace unified
}  // namespace localization
}  // namespace apollo
