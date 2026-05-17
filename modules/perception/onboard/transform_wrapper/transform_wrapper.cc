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
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

#include "cyber/common/log.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace onboard {

namespace {

}  // namespace

DEFINE_string(obs_sensor2vehicle_tf2_frame_id, "base_link",
              "parent frame id for sensor extrinsics");
DEFINE_string(obs_vehicle2world_tf2_frame_id, "map", "global pose frame id");
DEFINE_string(obs_vehicle2world_tf2_child_frame_id, "base_link",
              "vehicle pose child frame id");
DEFINE_double(obs_tf2_buff_size, 0.01, "query Cyber TF buffer size in second");
DEFINE_double(obs_transform_cache_size, 1.0, "transform cache size in second");
DEFINE_double(obs_max_local_pose_extrapolation_latency, 0.15,
              "max local pose extrapolation period in second");
DEFINE_bool(obs_enable_local_pose_extrapolation, true,
            "use local pose extrapolation");
DEFINE_bool(hardware_trigger, true, "camera trigger method");

namespace {

apollo::transform::TimedTransformResolverOptions
BuildTimedTransformResolverOptions() {
  apollo::transform::TimedTransformResolverOptions options;
  options.tf2_buffer_size_sec = static_cast<float>(FLAGS_obs_tf2_buff_size);
  options.cache_duration_sec = FLAGS_obs_transform_cache_size;
  options.max_extrapolation_latency_sec =
      FLAGS_obs_max_local_pose_extrapolation_latency;
  options.enable_extrapolation = FLAGS_obs_enable_local_pose_extrapolation;
  options.hardware_trigger = FLAGS_hardware_trigger;
  return options;
}

}  // namespace

void TransformWrapper::Init(
    const std::string& sensor2vehicle_tf2_child_frame_id) {
  sensor2vehicle_tf2_frame_id_ = FLAGS_obs_sensor2vehicle_tf2_frame_id;
  sensor2vehicle_tf2_child_frame_id_ = sensor2vehicle_tf2_child_frame_id;
  vehicle2world_tf2_frame_id_ = FLAGS_obs_vehicle2world_tf2_frame_id;
  vehicle2world_tf2_child_frame_id_ =
      FLAGS_obs_vehicle2world_tf2_child_frame_id;
  timed_transform_resolver_.SetOptions(BuildTimedTransformResolverOptions());
  inited_ = true;
}

void TransformWrapper::Init(
    const std::string& sensor2vehicle_tf2_frame_id,
    const std::string& sensor2vehicle_tf2_child_frame_id,
    const std::string& vehicle2world_tf2_frame_id,
    const std::string& vehicle2world_tf2_child_frame_id) {
  sensor2vehicle_tf2_frame_id_ = sensor2vehicle_tf2_frame_id;
  sensor2vehicle_tf2_child_frame_id_ = sensor2vehicle_tf2_child_frame_id;
  vehicle2world_tf2_frame_id_ = vehicle2world_tf2_frame_id;
  vehicle2world_tf2_child_frame_id_ = vehicle2world_tf2_child_frame_id;
  timed_transform_resolver_.SetOptions(BuildTimedTransformResolverOptions());
  inited_ = true;
}

bool TransformWrapper::GetSensor2worldTrans(
    double timestamp, Eigen::Affine3d* sensor2world_trans,
    Eigen::Affine3d* vehicle2world_trans) {
  if (!inited_) {
    AERROR << "TransformWrapper not Initialized,"
           << " unable to call GetSensor2worldTrans.";
    return false;
  }

  if (!EnsureSensor2VehicleExtrinsics()) {
    return false;
  }

  apollo::transform::StampedTransform trans_vehicle2world;
  if (!QueryTrans(timestamp, &trans_vehicle2world,
                  vehicle2world_tf2_frame_id_,
                  vehicle2world_tf2_child_frame_id_)) {
    return false;
  }

  const Eigen::Affine3d vehicle2world =
      trans_vehicle2world.translation * trans_vehicle2world.rotation;
  *sensor2world_trans = vehicle2world * (*sensor2vehicle_extrinsics_);
  if (vehicle2world_trans != nullptr) {
    *vehicle2world_trans = vehicle2world;
  }
  AINFO << "Get pose timestamp: " << timestamp << ", pose: \n"
        << (*sensor2world_trans).matrix();
  return true;
}

bool TransformWrapper::GetExtrinsics(Eigen::Affine3d* trans) {
  if (!inited_ || trans == nullptr) {
    AERROR << "TransformWrapper get extrinsics failed";
    return false;
  }

  if (!EnsureSensor2VehicleExtrinsics()) {
    return false;
  }

  *trans = *sensor2vehicle_extrinsics_;
  return true;
}

bool TransformWrapper::GetTrans(double timestamp, Eigen::Affine3d* trans,
                                const std::string& frame_id,
                                const std::string& child_frame_id) {
  apollo::transform::StampedTransform transform;
  if (!QueryTrans(timestamp, &transform, frame_id, child_frame_id)) {
    return false;
  }

  *trans = transform.translation * transform.rotation;
  return true;
}

bool TransformWrapper::QueryTrans(
  double timestamp, apollo::transform::StampedTransform* trans,
  const std::string& frame_id, const std::string& child_frame_id) {
  return timed_transform_resolver_.Resolve(timestamp, frame_id,
                                           child_frame_id, trans);
}

bool TransformWrapper::QueryStaticTrans(const std::string& frame_id,
                                        const std::string& child_frame_id,
                                        Eigen::Affine3d* trans) {
  if (trans == nullptr) {
    AERROR << "TransformWrapper query static transform failed";
    return false;
  }

  if (!transform_query_.GetLatestStaticTransformToAffine(frame_id,
                                                         child_frame_id,
                                                         trans)) {
    AERROR << "Failed to query static transform from " << child_frame_id
           << " to " << frame_id;
    return false;
  }

  return true;
}

bool TransformWrapper::EnsureSensor2VehicleExtrinsics() {
  if (sensor2vehicle_extrinsics_ != nullptr) {
    return true;
  }

  auto sensor2vehicle_extrinsics = std::make_unique<Eigen::Affine3d>(
      Eigen::Affine3d::Identity());
  if (!QueryStaticTrans(sensor2vehicle_tf2_frame_id_,
                        sensor2vehicle_tf2_child_frame_id_,
                        sensor2vehicle_extrinsics.get())) {
    return false;
  }

  sensor2vehicle_extrinsics_ = std::move(sensor2vehicle_extrinsics);
  AINFO << "Get sensor-to-body extrinsics successfully.";
  return true;
}

bool TransformWrapper::GetExtrinsicsBySensorId(
    const std::string& from_sensor_id, const std::string& to_sensor_id,
    Eigen::Affine3d* trans) {
  if (trans == nullptr) {
    AERROR << "TransformWrapper get extrinsics failed";
    return false;
  }

  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  std::string frame_id = sensor_manager->GetFrameId(to_sensor_id);
  std::string child_frame_id = sensor_manager->GetFrameId(from_sensor_id);

  if (frame_id.empty() || child_frame_id.empty()) {
    AERROR << "TransformWrapper get extrinsics failed, unknown sensor ids: "
           << from_sensor_id << " -> " << to_sensor_id;
    return false;
  }

  return QueryStaticTrans(frame_id, child_frame_id, trans);
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
