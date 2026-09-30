#pragma once

#include <cmath>

#include "Eigen/Eigenvalues"

#include "modules/perception/base/object.h"

namespace apollo {
namespace perception {
namespace fusion {

inline bool ValidPlanarCovariance(const Eigen::Matrix2d& covariance) {
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-6)) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  return solver.info() == Eigen::Success &&
         solver.eigenvalues().minCoeff() >= 0.0;
}

inline bool ValidPlanarObservation(const base::Object& object) {
  if (!object.center.allFinite() || !object.velocity.allFinite() ||
      !object.direction.allFinite() ||
      !object.size.allFinite() || (object.size.array() < 0).any() ||
      !object.center_uncertainty.allFinite() ||
      object.center_uncertainty(2, 2) < 0 ||
      !object.velocity_uncertainty.allFinite() ||
      object.velocity_uncertainty(2, 2) < 0 ||
      !object.acceleration_uncertainty.allFinite() ||
      object.acceleration_uncertainty(2, 2) < 0 ||
      !std::isfinite(object.theta) || !std::isfinite(object.confidence) ||
      object.confidence < 0 || object.confidence > 1 ||
      !ValidPlanarCovariance(
          object.center_uncertainty.topLeftCorner<2, 2>().cast<double>()) ||
      (object.velocity_converged &&
       !ValidPlanarCovariance(
           object.velocity_uncertainty.topLeftCorner<2, 2>().cast<double>()))) {
    return false;
  }
  for (float probability : object.type_probs) {
    if (!std::isfinite(probability) || probability < 0 || probability > 1) {
      return false;
    }
  }
  for (const auto& point : object.polygon) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      return false;
  }
  return true;
}

inline bool ValidCameraObservation(const base::Object& object) {
  const auto& camera = object.camera_supplement;
  if (!camera.local_center.allFinite() ||
      !std::isfinite(camera.box.xmin) || !std::isfinite(camera.box.xmax) ||
      !std::isfinite(camera.box.ymin) || !std::isfinite(camera.box.ymax)) {
    return false;
  }
  return camera.local_center.z() > 0 ||
      (camera.box.xmax > camera.box.xmin && camera.box.ymax > camera.box.ymin);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
