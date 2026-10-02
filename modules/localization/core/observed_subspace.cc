// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/observed_subspace.h"

#include <cmath>

#include "Eigen/Cholesky"
#include "Eigen/SVD"

namespace apollo {
namespace localization {
namespace unified {

Result CanonicalizeObservedSubspace(const Eigen::MatrixXd& projection,
                                   const Eigen::MatrixXd& covariance,
                                   double rotation_length,
                                   Eigen::MatrixXd* canonical_projection,
                                   Eigen::MatrixXd* canonical_covariance) {
  const int rank = projection.rows();
  if (canonical_projection == nullptr || canonical_covariance == nullptr ||
      canonical_projection == canonical_covariance || rank < 1 || rank > 6 ||
      projection.cols() != 6 || covariance.rows() != rank ||
      covariance.cols() != rank || !projection.allFinite() ||
      !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-9) ||
      Eigen::LLT<Eigen::MatrixXd>(covariance).info() != Eigen::Success ||
      !std::isfinite(rotation_length) || rotation_length <= 0.0) {
    return {Reason::INVALID_INPUT, "Invalid observed subspace/noise/metric"};
  }
  Matrix6d scale = Matrix6d::Identity();
  scale.bottomRightCorner<3, 3>() *= rotation_length;
  Matrix6d inverse_scale = Matrix6d::Identity();
  inverse_scale.bottomRightCorner<3, 3>() /= rotation_length;
  const Eigen::MatrixXd measured = projection * inverse_scale;
  if (!measured.allFinite()) {
    return {Reason::INVALID_INPUT, "Observed subspace scaling overflow"};
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      measured, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success ||
      svd.singularValues()(0) <= 0.0 ||
      svd.singularValues()(rank - 1) <= 1e-12 * svd.singularValues()(0)) {
    return {Reason::DEGENERATE, "Observed basis does not have declared row rank"};
  }
  const Eigen::MatrixXd span = svd.matrixV().leftCols(rank);
  const Matrix6d projector = span * span.transpose();
  Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(rank, 6);
  int row = 0;
  for (int axis = 0; axis < 6 && row < rank; ++axis) {
    Eigen::Matrix<double, 6, 1> direction = projector.col(axis);
    // Reorthogonalization avoids cancellation for nearly parallel projections.
    for (int pass = 0; pass < 2; ++pass) {
      for (int previous = 0; previous < row; ++previous) {
        direction -= basis.row(previous).dot(direction) *
                     basis.row(previous).transpose();
      }
    }
    if (direction.norm() <= 1e-7) {
      continue;
    }
    basis.row(row++) = direction.normalized().transpose();
  }
  if (row != rank) {
    return {Reason::DEGENERATE, "Cannot construct stable canonical subspace"};
  }
  const Eigen::MatrixXd change =
      basis * span * svd.singularValues().cwiseInverse().asDiagonal() *
      svd.matrixU().transpose();
  const Eigen::MatrixXd output_projection = basis * scale;
  Eigen::MatrixXd output_covariance = change * covariance * change.transpose();
  output_covariance =
      (0.5 * (output_covariance + output_covariance.transpose())).eval();
  if (!output_projection.allFinite() || !output_covariance.allFinite() ||
      Eigen::LLT<Eigen::MatrixXd>(output_covariance).info() != Eigen::Success) {
    return {Reason::INVALID_INPUT, "Canonical observed covariance is invalid"};
  }
  *canonical_projection = output_projection;
  *canonical_covariance = output_covariance;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
