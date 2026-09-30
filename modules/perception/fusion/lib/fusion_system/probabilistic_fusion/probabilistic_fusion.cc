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
#include "modules/perception/fusion/lib/fusion_system/probabilistic_fusion/probabilistic_fusion.h"

#include <map>
#include <cmath>
#include <utility>

#include "cyber/common/file.h"
#include "modules/common/util/perf_util.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/fusion/base/track_pool_types.h"
#include "modules/perception/fusion/base/object_snapshot.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/hm_tracks_objects_match.h"
#include "modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker/pbf_tracker.h"
#include "modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

namespace {

bool ValidFusionConfig(const ProbabilisticFusionConfig& config) {
  return config.max_cached_frame_num() > 0 &&
      std::isfinite(config.max_lidar_invisible_period()) &&
      config.max_lidar_invisible_period() > 0 &&
      std::isfinite(config.max_radar_invisible_period()) &&
      config.max_radar_invisible_period() > 0 &&
      std::isfinite(config.max_camera_invisible_period()) &&
      config.max_camera_invisible_period() > 0 &&
      std::isfinite(config.max_prediction_age()) &&
      config.max_prediction_age() > 0;
}

AssociationParams AssociationConfig(const ProbabilisticFusionConfig& config) {
  AssociationParams params;
  params.max_center_distance = config.association_max_center_distance();
  params.max_mahalanobis_distance = config.association_max_mahalanobis_distance();
  params.covariance_floor = config.association_covariance_floor();
  params.class_penalty = config.association_class_penalty();
  params.min_camera_similarity = config.association_min_camera_similarity();
  params.max_velocity_difference = config.association_max_velocity_difference();
  params.velocity_penalty = config.association_velocity_penalty();
  params.size_penalty = config.association_size_penalty();
  params.heading_penalty = config.association_heading_penalty();
  return params;
}

}  // namespace

bool ProbabilisticFusion::Init(const FusionInitOptions& init_options) {
  faulted_ = false;
  if (!SensorDataManager::Instance()->Init()) return false;
  main_sensor_ = init_options.main_sensor;

  BaseInitOptions options;
  if (!GetFusionInitOptions("ProbabilisticFusion", &options)) {
    return false;
  }

  std::string work_root_config = GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(work_root_config, options.conf_file);
  ProbabilisticFusionConfig params;

  if (!cyber::common::GetProtoFromFile(config, &params)) {
    AERROR << "Read config failed: " << config;
    return false;
  }
  if (!ValidFusionConfig(params) || main_sensor_.empty()) {
    AERROR << "Invalid legacy fusion configuration.";
    return false;
  }
  params_.prohibition_sensors.clear();
  probabilistic_fusion_config_ = params;
  trackers_.clear();
  legacy_scheduler_ = FrameScheduler(
      static_cast<size_t>(params.max_cached_frame_num()));
  params_.use_lidar = params.use_lidar();
  params_.use_radar = params.use_radar();
  params_.use_camera = params.use_camera();
  params_.tracker_method = params.tracker_method();
  if (params_.tracker_method != "PbfTracker") {
    AERROR << "Unknown tracker method: " << params_.tracker_method;
    return false;
  }
  params_.data_association_method = params.data_association_method();
  params_.gate_keeper_method = params.gate_keeper_method();
  for (int i = 0; i < params.prohibition_sensors_size(); ++i) {
    params_.prohibition_sensors.push_back(params.prohibition_sensors(i));
  }

  // static member initialization from PB config
  Track::SetMaxLidarInvisiblePeriod(params.max_lidar_invisible_period());
  Track::SetMaxRadarInvisiblePeriod(params.max_radar_invisible_period());
  Track::SetMaxCameraInvisiblePeriod(params.max_camera_invisible_period());
  Sensor::SetMaxCachedFrameNumber(params.max_cached_frame_num());

  scenes_.reset(new Scene());
  if (params_.data_association_method == "HMAssociation") {
    matcher_.reset(new HMTrackersObjectsAssociation(AssociationConfig(params)));
  } else {
    AERROR << "Unknown association method: " << params_.data_association_method;
    return false;
  }
  if (!matcher_->Init()) {
    AERROR << "Failed to init matcher.";
    return false;
  }

  if (params_.gate_keeper_method == "PbfGatekeeper") {
    gate_keeper_.reset(new PbfGatekeeper());
  } else {
    AERROR << "Unknown gate keeper method: " << params_.gate_keeper_method;
    return false;
  }
  if (!gate_keeper_->Init()) {
    AERROR << "Failed to init gatekeeper.";
    return false;
  }

  bool state = DstTypeFusion::Init() && DstExistenceFusion::Init() &&
               PbfTracker::InitParams();

  return state;
}

bool ProbabilisticFusion::Init(const StageConfig& stage_config) {
  faulted_ = false;
  if (!SensorDataManager::Instance()->Init()) return false;
  if (!Initialize(stage_config)) {
    return false;
  }

  probabilistic_fusion_config_ = stage_config.probabilistic_fusion_config();
  if (!ValidFusionConfig(probabilistic_fusion_config_)) {
    AERROR << "Invalid fusion lifecycle or cache configuration.";
    return false;
  }
  trackers_.clear();
  params_.prohibition_sensors.clear();
  params_.use_lidar = probabilistic_fusion_config_.use_lidar();
  params_.use_radar = probabilistic_fusion_config_.use_radar();
  params_.use_camera = probabilistic_fusion_config_.use_camera();

  params_.tracker_method = probabilistic_fusion_config_.tracker_method();
  if (params_.tracker_method != "PbfTracker") {
    AERROR << "Unknown tracker method: " << params_.tracker_method;
    return false;
  }
  params_.data_association_method =
      probabilistic_fusion_config_.data_association_method();
  for (const auto& prohibition_sensor :
       probabilistic_fusion_config_.prohibition_sensors()) {
    params_.prohibition_sensors.push_back(prohibition_sensor);
  }

  // static member initialization from PB config
  Track::SetMaxLidarInvisiblePeriod(
      probabilistic_fusion_config_.max_lidar_invisible_period());
  Track::SetMaxRadarInvisiblePeriod(
      probabilistic_fusion_config_.max_radar_invisible_period());
  Track::SetMaxCameraInvisiblePeriod(
      probabilistic_fusion_config_.max_camera_invisible_period());
  Sensor::SetMaxCachedFrameNumber(
      probabilistic_fusion_config_.max_cached_frame_num());

  scenes_.reset(new Scene());
  if (params_.data_association_method == "HMAssociation") {
    matcher_.reset(new HMTrackersObjectsAssociation(
        AssociationConfig(probabilistic_fusion_config_)));
  } else {
    AERROR << "Unknown association method: " << params_.data_association_method;
    return false;
  }
  if (!matcher_->Init()) {
    AERROR << "Failed to init matcher.";
    return false;
  }

  bool state = DstTypeFusion::Init() && DstExistenceFusion::Init() &&
               PbfTracker::InitParams();

  return state;
}

bool ProbabilisticFusion::Process(DataFrame* data_frame) {
  if (data_frame == nullptr) return false;

  FusionFrame* fusion_frame = data_frame->fusion_frame;
  if (fusion_frame == nullptr) return false;
  if (!fusion_frame->ready) return true;
  if (faulted_) {
    AERROR << "Fusion state is faulted; reinitialization is required.";
    return false;
  }
  if (std::fabs(fusion_frame->max_prediction_age -
      probabilistic_fusion_config_.max_prediction_age()) > 1e-9) {
    AERROR << "Scheduler and fusion prediction-age limits must agree.";
    return false;
  }
  if (!fusion_frame->frame || !scenes_) {
    AERROR << "Fusion cycle requires a timestamp and initialized scene.";
    return false;
  }

  // 3. perform fusion on related frames
  for (const auto& frame : fusion_frame->sensor_frames) {
    if (!FuseFrame(frame)) {
      faulted_ = true;
      return false;
    }
  }

  if (!PredictTracks(fusion_frame->frame->timestamp)) {
    faulted_ = true;
    return false;
  }
  fusion_frame->scene_ptr = scenes_;
  return true;
}

bool ProbabilisticFusion::Fuse(const FusionOptions& options,
                               const base::FrameConstPtr& sensor_frame,
                               std::vector<base::ObjectPtr>* fused_objects) {
  if (fused_objects == nullptr || !sensor_frame) {
    AERROR << "fusion error: fused_objects is nullptr";
    return false;
  }
  fused_objects->clear();

  auto* sensor_data_manager = SensorDataManager::Instance();
  if (!sensor_data_manager->IsKnownSensor(sensor_frame)) {
    AERROR << "Unregistered legacy fusion source.";
    return false;
  }
  std::lock_guard<std::mutex> data_lock(data_mutex_);
  if (faulted_ || !scenes_) {
    AERROR << "Legacy fusion is uninitialized or faulted.";
    return false;
  }
  if (!params_.use_lidar && sensor_data_manager->IsLidar(sensor_frame)) {
    return true;
  }
  if (!params_.use_radar && sensor_data_manager->IsRadar(sensor_frame)) {
    return true;
  }
  if (!params_.use_camera && sensor_data_manager->IsCamera(sensor_frame)) {
    return true;
  }

  bool is_publish_sensor = IsPublishSensor(sensor_frame);

  AINFO << "add sensor measurement: " << sensor_frame->sensor_info.name
        << ", obj_cnt : " << sensor_frame->objects.size() << ", "
        << FORMAT_TIMESTAMP(sensor_frame->timestamp);
  if (!legacy_scheduler_.Add(sensor_frame)) return false;

  if (!is_publish_sensor) {
    return true;
  }
  // 2. query related sensor_frames for fusion
  std::lock_guard<std::mutex> fuse_lock(fuse_mutex_);
  double fusion_time = sensor_frame->timestamp;
  std::vector<SensorFramePtr> frames;
  std::vector<base::FrameConstPtr> observations;
  if (!legacy_scheduler_.Drain(fusion_time, &observations)) return false;
  for (const auto& observation : observations) {
    frames.emplace_back(new SensorFrame(observation));
  }
  AINFO << "Get " << frames.size() << " related frames for fusion";

  // 3. perform fusion on related frames
  for (const auto& frame : frames) {
    if (!FuseFrame(frame)) {
      faulted_ = true;
      return false;
    }
  }
  if (!PredictTracks(fusion_time)) {
    faulted_ = true;
    return false;
  }

  // 4. collect fused objects
  if (!CollectFusedObjects(fusion_time, fused_objects)) {
    faulted_ = true;
    fused_objects->clear();
    return false;
  }
  return true;
}

bool ProbabilisticFusion::IsPublishSensor(
    const base::FrameConstPtr& sensor_frame) const {
  std::string sensor_id = sensor_frame->sensor_info.name;
  return main_sensor_ == sensor_id;
}

bool ProbabilisticFusion::FuseFrame(const SensorFramePtr& frame) {
  if (!frame) {
    AERROR << "Null fusion observation frame.";
    return false;
  }
  // Register only the frame being processed, so queued batches cannot evict
  // poses before camera projection or evidence updates have consumed them.
  if (!SensorDataManager::Instance()->AddSensorFrame(frame)) return false;
  AINFO << "Fusing frame: " << frame->GetSensorId()
        << ", foreground_object_number: "
        << frame->GetForegroundObjects().size()
        << ", background_object_number: "
        << frame->GetBackgroundObjects().size()
        << ", timestamp: " << FORMAT_TIMESTAMP(frame->GetTimestamp());
  if (!PredictTracks(frame->GetTimestamp()) || !FuseForegroundTrack(frame))
    return false;
  FusebackgroundTrack(frame);
  RemoveLostTrack();
  return true;
}

bool ProbabilisticFusion::PredictTracks(double timestamp) {
  const auto& foreground = scenes_->GetForegroundTracks();
  if (foreground.size() != trackers_.size()) {
    AERROR << "Fusion tracker and track ownership are inconsistent.";
    return false;
  }
  for (size_t index = 0; index < trackers_.size(); ++index) {
    if (timestamp - foreground[index]->GetLastMotionObservationTimestamp() >
        probabilistic_fusion_config_.max_prediction_age()) {
      foreground[index]->Expire();
    } else if (!trackers_[index]->PredictTo(timestamp)) {
      return false;
    }
  }
  for (const auto& track : scenes_->GetBackgroundTracks()) {
    if (timestamp - track->GetLastMotionObservationTimestamp() >
        probabilistic_fusion_config_.max_prediction_age()) {
      track->Expire();
      continue;
    }
    track->PredictBackgroundTo(timestamp);
    track->ExpireSensorObjects(timestamp);
  }
  RemoveLostTrack();
  return true;
}

bool ProbabilisticFusion::FuseForegroundTrack(const SensorFramePtr& frame) {
  PERF_BLOCK_START();
  std::string indicator = "fusion_" + frame->GetSensorId();

  AssociationOptions options;
  AssociationResult association_result;
  if (!matcher_->Associate(options, frame, scenes_, &association_result)) {
    AERROR << "Fusion association failed.";
    return false;
  }
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "association");

  if (!UpdateAssignedTracks(frame, association_result)) return false;
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "update_assigned_track");

  if (!UpdateUnassignedTracks(frame, association_result)) return false;
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "update_unassigned_track");

  const std::vector<size_t>& unassigned_obj_inds =
      association_result.unassigned_measurements;
  if (!CreateNewTracks(frame, unassigned_obj_inds)) return false;
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "create_track");
  return true;
}

bool ProbabilisticFusion::UpdateAssignedTracks(
    const SensorFramePtr& frame, const AssociationResult& association) {
  const auto& assignments = association.assignments;
  TrackerOptions options;
  std::vector<SensorObjectPtr>& f_ground_objs = frame->GetForegroundObjects();
  for (size_t i = 0; i < assignments.size(); ++i) {
    size_t track_ind = assignments[i].first;
    size_t obj_ind = assignments[i].second;
    options.match_distance = association.track_association_loss[track_ind];
    if (!trackers_[track_ind]->UpdateWithMeasurement(
            options, f_ground_objs[obj_ind], frame->GetTimestamp())) {
      return false;
    }
  }
  return true;
}

bool ProbabilisticFusion::UpdateUnassignedTracks(
    const SensorFramePtr& frame, const AssociationResult& association) {
  const auto& unassigned_track_inds = association.unassigned_tracks;
  TrackerOptions options;
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < unassigned_track_inds.size(); ++i) {
    size_t track_ind = unassigned_track_inds[i];
    options.match_distance = association.track_miss_similarity[track_ind];
    if (!trackers_[track_ind]->UpdateWithoutMeasurement(
            options, sensor_id, frame->GetTimestamp(), frame->GetTimestamp())) {
      return false;
    }
  }
  return true;
}

bool ProbabilisticFusion::CreateNewTracks(
    const SensorFramePtr& frame,
    const std::vector<size_t>& unassigned_obj_inds) {
  // check valid
  bool prohibition_sensor_flag = false;
  std::for_each(params_.prohibition_sensors.begin(),
                params_.prohibition_sensors.end(),
                [&](const std::string& sensor_name) {
                  if (sensor_name == frame->GetSensorId())
                    prohibition_sensor_flag = true;
                });
  if (prohibition_sensor_flag) {
    return true;
  }

  // add new track
  std::vector<SensorObjectPtr>& f_ground_objs = frame->GetForegroundObjects();
  for (const size_t& obj_ind : unassigned_obj_inds) {
    if (IsCamera(f_ground_objs[obj_ind]) &&
        f_ground_objs[obj_ind]->GetBaseObject()
                ->camera_supplement.local_center.z() <= 0) {
      ADEBUG << "Camera 2D observation cannot initialize a planar motion track.";
      continue;
    }
    TrackPtr track = TrackPool::Instance().Get();
    track->Initialize(f_ground_objs[obj_ind]);

    ADEBUG << "object id: " << f_ground_objs[obj_ind]->GetBaseObject()->track_id
           << ", create new track: " << track->GetTrackId();

    if (params_.tracker_method == "PbfTracker") {
      std::shared_ptr<BaseTracker> tracker;
      tracker.reset(new PbfTracker());
      if (!tracker->Init(track, f_ground_objs[obj_ind])) {
        AERROR << "Failed to initialize fusion track.";
        return false;
      }
      scenes_->AddForegroundTrack(track);
      trackers_.emplace_back(tracker);
    } else {
      AERROR << "Unknown tracker method: " << params_.tracker_method;
      return false;
    }
  }
  return true;
}

void ProbabilisticFusion::FusebackgroundTrack(const SensorFramePtr& frame) {
  // 1. association
  size_t track_size = scenes_->GetBackgroundTracks().size();
  size_t obj_size = frame->GetBackgroundObjects().size();
  std::map<int, size_t> local_id_2_track_ind_map;
  std::vector<bool> track_tag(track_size, false);
  std::vector<bool> object_tag(obj_size, false);

  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < track_size; ++i) {
    const FusedObjectPtr& obj = background_tracks[i]->GetFusedObject();
    int local_id = obj->GetBaseObject()->track_id;
    local_id_2_track_ind_map[local_id] = i;
  }

  std::vector<TrackMeasurmentPair> assignments;
  std::vector<SensorObjectPtr>& frame_objs = frame->GetBackgroundObjects();
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < obj_size; ++i) {
    int local_id = frame_objs[i]->GetBaseObject()->track_id;
    int search_id = Track::ComputeBackgroundGlobalId(sensor_id, local_id);
    const auto& it = local_id_2_track_ind_map.find(search_id);
    if (it != local_id_2_track_ind_map.end()) {
      size_t track_ind = it->second;
      assignments.push_back(std::make_pair(track_ind, i));
      track_tag[track_ind] = true;
      object_tag[i] = true;
    }
  }

  // 2. update assigned track
  for (size_t i = 0; i < assignments.size(); ++i) {
    size_t track_ind = assignments[i].first;
    size_t obj_ind = assignments[i].second;
    background_tracks[track_ind]->UpdateWithSensorObject(frame_objs[obj_ind]);
  }

  // 3. update unassigned track
  for (size_t i = 0; i < track_tag.size(); ++i) {
    if (!track_tag[i]) {
      background_tracks[i]->UpdateWithoutSensorObject(sensor_id,
                                                      frame->GetTimestamp());
    }
  }

  // 4. create new track
  for (size_t i = 0; i < object_tag.size(); ++i) {
    if (!object_tag[i]) {
      TrackPtr track = TrackPool::Instance().Get();
      track->Initialize(frame_objs[i], true);
      scenes_->AddBackgroundTrack(track);
    }
  }
}

void ProbabilisticFusion::RemoveLostTrack() {
  // need to remove tracker at the same time
  size_t f_alive_index = 0;
  std::vector<TrackPtr>& foreground_tracks = scenes_->GetForegroundTracks();
  for (size_t i = 0; i < foreground_tracks.size(); ++i) {
    if (foreground_tracks[i]->IsAlive()) {
      foreground_tracks[f_alive_index] = foreground_tracks[i];
      trackers_[f_alive_index] = trackers_[i];
      ++f_alive_index;
    }
  }
  AINFO << "Remove " << foreground_tracks.size() - f_alive_index
        << " foreground tracks. " << f_alive_index << " tracks left.";
  foreground_tracks.resize(f_alive_index);
  trackers_.resize(f_alive_index);

  // only need to remove frame track
  size_t b_alive_index = 0;
  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < background_tracks.size(); ++i) {
    if (background_tracks[i]->IsAlive()) {
      background_tracks[b_alive_index] = background_tracks[i];
      ++b_alive_index;
    }
  }
  AINFO << "Remove " << background_tracks.size() - b_alive_index
        << " background tracks. " << b_alive_index << " tracks left.";
  background_tracks.resize(b_alive_index);
}

bool ProbabilisticFusion::CollectFusedObjects(
    double timestamp, std::vector<base::ObjectPtr>* fused_objects) {
  fused_objects->clear();

  size_t fg_obj_num = 0;
  const std::vector<TrackPtr>& foreground_tracks =
      scenes_->GetForegroundTracks();
  for (const auto& track_ptr : foreground_tracks) {
    if (timestamp - track_ptr->GetLastMotionObservationTimestamp() <=
        probabilistic_fusion_config_.max_prediction_age() &&
        gate_keeper_->AbleToPublish(track_ptr)) {
      if (!CollectObjectsByTrack(timestamp, track_ptr, fused_objects)) return false;
      ++fg_obj_num;
    }
  }

  size_t bg_obj_num = 0;
  const std::vector<TrackPtr>& background_tracks =
      scenes_->GetBackgroundTracks();
  for (const auto& track_ptr : background_tracks) {
    if (timestamp - track_ptr->GetLastMotionObservationTimestamp() <=
        probabilistic_fusion_config_.max_prediction_age() &&
        gate_keeper_->AbleToPublish(track_ptr)) {
      if (!CollectObjectsByTrack(timestamp, track_ptr, fused_objects)) return false;
      ++bg_obj_num;
    }
  }

  AINFO << "collect objects : fg_obj_cnt = " << fg_obj_num
        << ", bg_obj_cnt = " << bg_obj_num
        << ", timestamp = " << FORMAT_TIMESTAMP(timestamp);
  return true;
}

bool ProbabilisticFusion::CollectObjectsByTrack(
    double timestamp, const TrackPtr& track,
    std::vector<base::ObjectPtr>* fused_objects) {
  return AppendTrackSnapshot(timestamp, track, fused_objects);
}

void ProbabilisticFusion::CollectSensorMeasurementFromObject(
    const SensorObjectConstPtr& object,
    base::SensorObjectMeasurement* measurement) {
  CopySensorMeasurement(object, measurement);
}

FUSION_REGISTER_FUSIONSYSTEM(ProbabilisticFusion);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
