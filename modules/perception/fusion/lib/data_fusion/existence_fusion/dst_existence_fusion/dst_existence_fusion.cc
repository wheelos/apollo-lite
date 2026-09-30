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
#include "modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"

#include <cmath>

#include "cyber/common/file.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/fusion/base/sensor_data_manager.h"
#include "modules/perception/fusion/common/camera_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

const char* DstExistenceFusion::name_ = "DstExistenceFusion";
const char* DstExistenceFusion::toic_name_ = "DstToicFusion";
ExistenceDstMaps DstExistenceFusion::existence_dst_maps_;
ToicDstMaps DstExistenceFusion::toic_dst_maps_;
DstExistenceFusionOptions DstExistenceFusion::options_;

DstExistenceFusion::DstExistenceFusion(TrackPtr track)
    : BaseExistenceFusion(track),
      fused_toic_(toic_name_),
      fused_existence_(name_),
      last_evidence_timestamp_(track->GetLastObservationTimestamp()) {}

bool DstExistenceFusion::Init() {
  BaseInitOptions options;
  if (!GetFusionInitOptions(name_, &options)) return false;
  const auto root = cyber::common::GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);
  DstExistenceFusionConfig config;
  if (!cyber::common::GetProtoFromFile(
      cyber::common::GetAbsolutePath(root, options.conf_file), &config)) {
    AERROR << "Failed to read fusion existence policy.";
    return false;
  }
  options_ = DstExistenceFusionOptions();
  options_.evidence_half_life = config.evidence_half_life();
  options_.lidar_reliability = config.lidar_reliability();
  options_.radar_reliability = config.radar_reliability();
  options_.camera_reliability = config.camera_reliability();
  options_.unknown_type_discount = config.unknown_type_discount();
  options_.camera_toic_weight = config.camera_toic_weight();
  if (!std::isfinite(options_.evidence_half_life) ||
      options_.evidence_half_life <= 0) {
    AERROR << "Invalid existence-evidence half life.";
    return false;
  }
  for (double reliability : {options_.lidar_reliability,
      options_.radar_reliability, options_.camera_reliability,
      options_.unknown_type_discount, options_.camera_toic_weight}) {
    if (!std::isfinite(reliability) || reliability < 0 || reliability > 1) {
      AERROR << "Existence reliability must be in [0, 1].";
      return false;
    }
  }
  for (const auto& item : config.camera_valid_dist()) {
    if (item.camera_name().empty() || !std::isfinite(item.valid_dist()) ||
        item.valid_dist() <= 0) {
      AERROR << "Invalid camera visibility configuration.";
      return false;
    }
    options_.camera_max_valid_dist_[item.camera_name()] = item.valid_dist();
  }
  for (const auto& policy : config.miss_evidence_policy()) {
    if (policy.sensor_name().empty() || !std::isfinite(policy.min_range()) ||
        !std::isfinite(policy.max_range()) ||
        !std::isfinite(policy.horizontal_half_fov_degrees()) ||
        policy.min_range() < 0 || policy.max_range() <= policy.min_range() ||
        policy.horizontal_half_fov_degrees() <= 0 ||
        policy.horizontal_half_fov_degrees() > 180) {
      AERROR << "Invalid missing-observation visibility policy.";
      return false;
    }
    options_.miss_evidence_policy[policy.sensor_name()] = policy;
  }
  auto* manager = DstManager::Instance();
  return (manager->IsAppAdded(name_) ||
          manager->AddApp(name_, existence_dst_maps_.fod_subsets_,
                          existence_dst_maps_.subset_names_)) &&
         (manager->IsAppAdded(toic_name_) ||
          manager->AddApp(toic_name_, toic_dst_maps_.fod_subsets_,
                          toic_dst_maps_.subset_names_));
}

bool DstExistenceFusion::AdvanceEvidence(double timestamp) {
  if (!std::isfinite(timestamp) || timestamp < last_evidence_timestamp_) {
    AERROR << "Invalid existence-evidence timestamp.";
    return false;
  }
  const double weight = std::exp2(
      -(timestamp - last_evidence_timestamp_) / options_.evidence_half_life);
  fused_existence_ = fused_existence_ * weight;
  fused_toic_ = fused_toic_ * weight;
  last_evidence_timestamp_ = timestamp;
  return true;
}

bool DstExistenceFusion::DistanceDecay(const SensorObjectPtr& measurement,
                                     double* decay) {
  if (IsRadar(measurement)) {
    *decay = measurement->GetBaseObject()->confidence;
    return true;
  }
  Eigen::Affine3d pose;
  if (!SensorDataManager::Instance()->GetPose(
      measurement->GetSensorId(), measurement->GetTimestamp(), &pose)) {
    AERROR << "Missing observation pose for existence evidence.";
    return false;
  }
  const Eigen::Vector3d local =
      pose.inverse() * measurement->GetBaseObject()->center;
  if (!local.allFinite()) {
    AERROR << "Invalid existence observation transform.";
    return false;
  }
  *decay = (local.norm() > 60 ? 0.8 : 1.0) *
           measurement->GetBaseObject()->confidence;
  return true;
}

bool DstExistenceFusion::Visibility(const std::string& sensor, double timestamp,
                                   double* visibility) {
  *visibility = 0;
  auto* data = SensorDataManager::Instance();
  Eigen::Affine3d pose;
  if (!data->GetPose(sensor, timestamp, &pose)) {
    AERROR << "Missing processed sensor pose for visibility evidence: " << sensor;
    return false;
  }
  if (common::SensorManager::Instance()->IsCamera(sensor)) {
    const auto model = data->GetCameraIntrinsic(sensor);
    if (!model) {
      AERROR << "Missing camera visibility model: " << sensor;
      return false;
    }
    const auto limit = options_.camera_max_valid_dist_.find(sensor);
    if (limit == options_.camera_max_valid_dist_.end()) {
      AWARN << "Camera negative evidence disabled without a range policy: "
            << sensor;
      return true;
    }
    const auto lidar = track_ref_->GetLatestLidarObject();
    const auto radar = track_ref_->GetLatestRadarObject();
    if (lidar || radar) {
      *visibility = ObjectInCameraView(lidar ? lidar : radar, model, pose,
          timestamp, limit->second, lidar != nullptr, false);
    }
  } else {
    const auto policy = options_.miss_evidence_policy.find(sensor);
    if (policy == options_.miss_evidence_policy.end()) {
      ADEBUG << "Negative existence evidence disabled without coverage: "
             << sensor;
      return true;
    }
    const Eigen::Vector3d local =
        pose.inverse() * track_ref_->GetFusedObject()->GetBaseObject()->center;
    const double range = local.norm();
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    const double angle = std::fabs(std::atan2(local.y(), local.x())) *
                         kRadiansToDegrees;
    if (range >= policy->second.min_range() &&
        range <= policy->second.max_range() &&
        angle <= policy->second.horizontal_half_fov_degrees()) {
      *visibility = 1;
    }
  }
  if (!std::isfinite(*visibility) || *visibility < 0 || *visibility > 1) {
    AERROR << "Invalid fusion visibility evidence.";
    return false;
  }
  return true;
}

bool DstExistenceFusion::UpdateWithMeasurement(
    const SensorObjectPtr measurement, double timestamp, double loss) {
  if (!measurement || !std::isfinite(loss) || loss < 0 || loss > 1 ||
      !AdvanceEvidence(timestamp)) {
    AERROR << "Invalid matched existence evidence.";
    return false;
  }
  double decay = 0;
  if (!DistanceDecay(measurement, &decay)) return false;
  const double mass = GetExistReliability(measurement) * decay;
  Dst evidence(name_);
  if (!evidence.SetBba({{ExistenceDstMaps::EXIST, mass},
      {ExistenceDstMaps::EXISTUNKNOWN, 1 - mass}}) ||
      !fused_existence_.TryCombine(evidence * (1 - loss), &fused_existence_)) {
    return false;
  }
  if (IsCamera(measurement)) {
    double visibility = 0;
    if (!Visibility(measurement->GetSensorId(), timestamp, &visibility)) {
      return false;
    }
    Dst toic(toic_name_);
    if (!toic.SetBba({{ToicDstMaps::TOIC, 1 - loss},
        {ToicDstMaps::TOICUNKNOWN, loss}}) ||
        !fused_toic_.TryCombine(toic * visibility * options_.camera_toic_weight,
                               &fused_toic_)) {
      return false;
    }
  }
  UpdateExistenceState();
  return true;
}

bool DstExistenceFusion::UpdateWithoutMeasurement(
    const std::string& sensor, double measurement_timestamp, double timestamp,
    double similarity) {
  if (!std::isfinite(similarity) || similarity < 0 || similarity > 1 ||
      !AdvanceEvidence(timestamp)) {
    AERROR << "Invalid missed existence evidence.";
    return false;
  }
  double visibility = 0;
  if (!Visibility(sensor, measurement_timestamp, &visibility)) return false;
  const double mass = GetUnexistReliability(sensor) * visibility *
                      (1 - similarity);
  Dst evidence(name_);
  if (!evidence.SetBba({{ExistenceDstMaps::NEXIST, mass},
      {ExistenceDstMaps::EXISTUNKNOWN, 1 - mass}}) ||
      !fused_existence_.TryCombine(evidence, &fused_existence_)) {
    return false;
  }
  if (common::SensorManager::Instance()->IsCamera(sensor)) {
    Dst toic(toic_name_);
    if (!toic.SetBba({{ToicDstMaps::NTOIC, 1 - similarity},
        {ToicDstMaps::TOICUNKNOWN, similarity}}) ||
        !fused_toic_.TryCombine(toic * visibility, &fused_toic_)) {
      return false;
    }
  }
  UpdateExistenceState();
  return true;
}

double DstExistenceFusion::GetExistReliability(
    const SensorObjectPtr measurement) {
  const auto type = measurement->GetBaseObject()->type;
  const double discount = type == base::ObjectType::UNKNOWN ||
      type == base::ObjectType::UNKNOWN_MOVABLE
      ? options_.unknown_type_discount : 1.0;
  return GetUnexistReliability(measurement->GetSensorId()) * discount;
}

double DstExistenceFusion::GetUnexistReliability(const std::string& sensor) {
  auto* manager = common::SensorManager::Instance();
  if (manager->IsCamera(sensor)) return options_.camera_reliability;
  if (manager->IsLidar(sensor)) return options_.lidar_reliability;
  return options_.radar_reliability;
}

std::string DstExistenceFusion::Name() const { return name_; }

double DstExistenceFusion::GetExistenceProbability() const {
  fused_existence_.ComputeProbability();
  return fused_existence_.GetProbabilityVec()[
      DstManager::Instance()->FodSubsetToInd(name_, ExistenceDstMaps::EXIST)];
}

double DstExistenceFusion::GetToicProbability() const {
  fused_toic_.ComputeProbability();
  return fused_toic_.GetProbabilityVec()[
      DstManager::Instance()->FodSubsetToInd(toic_name_, ToicDstMaps::TOIC)];
}

void DstExistenceFusion::UpdateExistenceState() {
  const double toic = GetToicProbability();
  track_ref_->SetToicProb(toic);
  track_ref_->SetExistenceProb(GetExistenceProbability());
  toic_score_ = track_ref_->GetLidarObjects().empty() &&
      !track_ref_->GetRadarObjects().empty() ? 0.5 + (toic - 0.5) * 0.6 : 0.5;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
