/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/fusion/lib/data_association/hm_data_association/hm_tracks_objects_match.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "Eigen/Cholesky"
#include "cyber/common/log.h"
#include "modules/perception/fusion/base/observation_validation.h"
#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/common/camera_util.h"

namespace apollo {
namespace perception {
namespace fusion {
namespace {

bool KnownType(base::ObjectType type) {
  return type == base::ObjectType::VEHICLE ||
         type == base::ObjectType::PEDESTRIAN ||
         type == base::ObjectType::BICYCLE;
}

}  // namespace

bool HMTrackersObjectsAssociation::Init() {
  if (!std::isfinite(params_.max_center_distance) ||
      params_.max_center_distance <= 0 ||
      !std::isfinite(params_.max_mahalanobis_distance) ||
      params_.max_mahalanobis_distance <= 0 ||
      !std::isfinite(params_.covariance_floor) ||
      params_.covariance_floor <= 0 ||
      !std::isfinite(params_.class_penalty) ||
      params_.class_penalty < 0 || params_.class_penalty >= 1 ||
      !std::isfinite(params_.min_camera_similarity) ||
      params_.min_camera_similarity <= 0 || params_.min_camera_similarity > 1 ||
      !std::isfinite(params_.max_velocity_difference) ||
      params_.max_velocity_difference <= 0) {
    AERROR << "Invalid fusion association configuration.";
    return false;
  }
  for (double penalty : {params_.velocity_penalty, params_.size_penalty,
                          params_.heading_penalty}) {
    if (!std::isfinite(penalty) || penalty < 0 || penalty >= 1) {
      AERROR << "Invalid association attribute penalty.";
      return false;
    }
  }
  return true;
}

bool HMTrackersObjectsAssociation::Evaluate(
    const TrackPtr& track, const SensorObjectPtr& measurement,
    double* loss, bool* accepted) {
  *loss = 1.0;
  *accepted = false;
  const auto object = measurement->GetBaseObject();
  const auto estimate = track->GetFusedObject()->GetBaseObject();
  if (!ValidPlanarObservation(*object) || !ValidPlanarObservation(*estimate)) {
    AERROR << "Invalid association state or observation.";
    return false;
  }
  if (IsCamera(measurement) && object->camera_supplement.local_center.z() <= 0) {
    double similarity = 0.0;
    auto* data = SensorDataManager::Instance();
    Eigen::Affine3d pose;
    if (!ValidCameraObservation(*object) ||
        !data->GetPose(measurement->GetSensorId(),
                       measurement->GetTimestamp(), &pose) ||
        !ProjectedBoxSimilarity(estimate, object->camera_supplement.box, pose,
            data->GetCameraIntrinsic(measurement->GetSensorId()), &similarity)) {
      AERROR << "Failed camera association projection.";
      return false;
    }
    *loss = 1.0 - similarity;
    *accepted = similarity >= params_.min_camera_similarity;
    return true;
  }
  const Eigen::Vector2d residual =
      object->center.head<2>() - estimate->center.head<2>();
  const double distance = residual.norm();
  if (distance >= params_.max_center_distance) return true;
  Eigen::Matrix2d covariance =
      object->center_uncertainty.topLeftCorner<2, 2>().cast<double>() +
      estimate->center_uncertainty.topLeftCorner<2, 2>().cast<double>();
  covariance.diagonal().array() += params_.covariance_floor;
  Eigen::LDLT<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.isPositive()) {
    AERROR << "Invalid association innovation covariance.";
    return false;
  }
  const double mahalanobis = residual.dot(solver.solve(residual));
  if (!std::isfinite(mahalanobis)) {
    AERROR << "Non-finite association innovation.";
    return false;
  }
  if (mahalanobis > params_.max_mahalanobis_distance) return true;

  double geometry_loss = distance / params_.max_center_distance;
  double attribute_loss = 0;
  if (object->velocity_converged && estimate->velocity_converged) {
    const double difference =
        (object->velocity.head<2>() - estimate->velocity.head<2>()).norm();
    if (difference > params_.max_velocity_difference) return true;
    attribute_loss += params_.velocity_penalty *
                      difference / params_.max_velocity_difference;
  }
  if ((object->size.array() > 0).all() &&
      (estimate->size.array() > 0).all()) {
    const double size_difference = (
        (object->size - estimate->size).cwiseAbs().array() /
        object->size.cwiseMax(estimate->size).array()).mean();
    attribute_loss += params_.size_penalty * size_difference;
    attribute_loss += params_.heading_penalty *
        (1 - std::cos(2 * (object->theta - estimate->theta))) * 0.5;
  }
  const bool incompatible = KnownType(object->type) && KnownType(estimate->type) &&
                            object->type != estimate->type;
  *loss = std::min(1.0, geometry_loss + attribute_loss +
      (incompatible ? params_.class_penalty : 0.0));
  *accepted = *loss < 1.0;
  return true;
}

bool HMTrackersObjectsAssociation::Associate(
    const AssociationOptions& options, SensorFramePtr sensor_measurements,
    ScenePtr scene, AssociationResult* result) {
  if (!sensor_measurements || !scene || !result) {
    AERROR << "Association requires measurements, a scene and output.";
    return false;
  }
  *result = AssociationResult();
  const auto& objects = sensor_measurements->GetForegroundObjects();
  const auto& tracks = scene->GetForegroundTracks();
  result->track_association_loss.assign(tracks.size(), 1.0);
  result->track_miss_similarity.assign(tracks.size(), 0.0);
  result->track2measurements_dist.assign(tracks.size(), 1.0);
  result->measurement2track_dist.assign(objects.size(), 1.0);
  if (tracks.empty() || objects.empty()) {
    for (size_t i = 0; i < tracks.size(); ++i) result->unassigned_tracks.push_back(i);
    for (size_t i = 0; i < objects.size(); ++i) {
      result->unassigned_measurements.push_back(i);
    }
    return true;
  }
  auto* costs = optimizer_.mutable_global_costs();
  costs->Resize(tracks.size(), objects.size());
  std::vector<std::vector<double>> losses(
      tracks.size(), std::vector<double>(objects.size(), 1.0));
  for (size_t row = 0; row < tracks.size(); ++row) {
    for (size_t col = 0; col < objects.size(); ++col) {
      if (!tracks[row] || !objects[col]) {
        AERROR << "Null association candidate.";
        return false;
      }
      bool accepted = false;
      if (!Evaluate(tracks[row], objects[col], &losses[row][col], &accepted)) {
        return false;
      }
      double assignment_cost = accepted ? losses[row][col] : 2.0;
      const auto previous =
          tracks[row]->GetSensorObject(objects[col]->GetSensorId());
      if (accepted && previous && objects[col]->GetBaseObject()->track_id >= 0 &&
          previous->GetBaseObject()->track_id ==
              objects[col]->GetBaseObject()->track_id) {
        // Local identity is a tie preference, never a gate bypass.
        assignment_cost *= 0.99;
      }
      (*costs)(row, col) = static_cast<float>(assignment_cost);
      if (accepted) {
        result->track_miss_similarity[row] =
            std::max(result->track_miss_similarity[row], 1.0 - losses[row][col]);
      }
    }
  }
  optimizer_.Match(1.0f, 2.0f,
                  common::GatedHungarianMatcher<float>::OptimizeFlag::OPTMIN,
                  &result->assignments, &result->unassigned_tracks,
                  &result->unassigned_measurements);
  for (const auto& assignment : result->assignments) {
    const double loss = losses[assignment.first][assignment.second];
    result->track_association_loss[assignment.first] = loss;
    result->track2measurements_dist[assignment.first] = loss;
    result->measurement2track_dist[assignment.second] = loss;
  }
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
