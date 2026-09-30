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
#include "modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"

#include <cmath>

#include "modules/perception/pipeline/proto/plugin/pbf_gatekeeper_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

PbfGatekeeper::PbfGatekeeper(const PluginConfig& config) {
  name_ = "PbfGatekeeper";
  if (!Init(config)) AERROR << "Failed to initialize fusion publication policy.";
}

bool PbfGatekeeper::Init() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("PbfGatekeeper", &options)) return false;
  const std::string root = cyber::common::GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);
  PluginConfig plugin;
  if (!cyber::common::GetProtoFromFile(
          cyber::common::GetAbsolutePath(root, options.conf_file),
          plugin.mutable_pbf_gatekeeper_config())) {
    AERROR << "Failed to load fusion publication policy.";
    return false;
  }
  return Init(plugin);
}

bool PbfGatekeeper::Init(const PluginConfig& plugin) {
  initialized_ = false;
  const auto& config = plugin.pbf_gatekeeper_config();
  params_.publish_if_has_lidar = config.publish_if_has_lidar();
  params_.publish_if_has_radar = config.publish_if_has_radar();
  params_.publish_if_has_camera = config.publish_if_has_camera();
  params_.use_camera_3d = config.use_camera_3d();
  params_.min_radar_confident_distance = config.min_radar_confident_distance();
  params_.min_camera_publish_distance = config.min_camera_publish_distance();
  params_.existence_threshold = config.existence_threshold();
  params_.radar_existence_threshold = config.radar_existence_threshold();
  params_.use_track_time_pub_strategy = config.use_track_time_pub_strategy();
  params_.pub_track_time_thresh = config.pub_track_time_thresh();
  blocked_publish_sensors_.clear();
  blocked_publish_sensors_.insert(config.blocked_publish_sensors().begin(),
                                  config.blocked_publish_sensors().end());
  if (params_.pub_track_time_thresh < 0 ||
      !std::isfinite(params_.min_radar_confident_distance) ||
      params_.min_radar_confident_distance < 0 ||
      !std::isfinite(params_.min_camera_publish_distance) ||
      params_.min_camera_publish_distance < 0 ||
      !std::isfinite(params_.existence_threshold) ||
      params_.existence_threshold < 0 || params_.existence_threshold > 1 ||
      !std::isfinite(params_.radar_existence_threshold) ||
      params_.radar_existence_threshold < 0 ||
      params_.radar_existence_threshold > 1) {
    AERROR << "Invalid fusion publication policy.";
    return false;
  }
  initialized_ = true;
  enable_ = plugin.enabled();
  return true;
}

bool PbfGatekeeper::SourceAllowed(const SensorObjectConstPtr& object) const {
  return object && blocked_publish_sensors_.count(object->GetSensorId()) == 0;
}

bool PbfGatekeeper::AbleToPublish(const TrackPtr& track) {
  return Decide(track).publish;
}

PublicationDecision PbfGatekeeper::Decide(const TrackPtr& track) const {
  if (!initialized_) return {false, PublicationReason::kUninitialized};
  if (!track) return {false, PublicationReason::kInvalidTrack};
  if (!track->IsAlive()) return {false, PublicationReason::kExpired};
  if (params_.use_track_time_pub_strategy &&
      track->GetTrackedTimes() <=
          static_cast<size_t>(params_.pub_track_time_thresh)) {
    return {false, PublicationReason::kUnconfirmed};
  }
  if (LidarAbleToPublish(track) || RadarAbleToPublish(track) ||
      CameraAbleToPublish(track)) {
    return {true, PublicationReason::kAccepted};
  }
  return {false, PublicationReason::kNoEligibleSource};
}

bool PbfGatekeeper::LidarAbleToPublish(const TrackPtr& track) const {
  if (!params_.publish_if_has_lidar) return false;
  for (const auto& item : track->GetLidarObjects()) {
    if (SourceAllowed(item.second)) return true;
  }
  return false;
}

bool PbfGatekeeper::RadarAbleToPublish(const TrackPtr& track) const {
  if (!params_.publish_if_has_radar ||
      track->GetExistenceProb() < params_.radar_existence_threshold) {
    return false;
  }
  for (const auto& item : track->GetRadarObjects()) {
    if (SourceAllowed(item.second) &&
        item.second->GetBaseObject()->radar_supplement.range >=
            params_.min_radar_confident_distance) {
      return true;
    }
  }
  return false;
}

bool PbfGatekeeper::CameraAbleToPublish(const TrackPtr& track) const {
  if (!params_.publish_if_has_camera || !params_.use_camera_3d ||
      track->GetExistenceProb() < params_.existence_threshold) {
    return false;
  }
  for (const auto& item : track->GetCameraObjects()) {
    if (!SourceAllowed(item.second)) continue;
    const auto& object = item.second->GetBaseObject();
    if (object->camera_supplement.local_center.z() > 0 &&
        (object->sub_type == base::ObjectSubType::TRAFFICCONE ||
        object->camera_supplement.local_center.norm() >=
            params_.min_camera_publish_distance)) {
      return true;
    }
  }
  return false;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
