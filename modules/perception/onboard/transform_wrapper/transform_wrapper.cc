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
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace onboard {

DEFINE_string(obs_sensor2novatel_tf2_frame_id, "imu",
              "sensor to novatel frame id");
DEFINE_string(obs_novatel2world_tf2_frame_id, "world",
              "novatel to world frame id");
DEFINE_string(obs_novatel2world_tf2_child_frame_id, "imu",
              "novatel to world child frame id");
DEFINE_double(obs_tf2_buff_size, 0.01, "query Cyber TF buffer size in second");
DEFINE_double(obs_transform_cache_size, 1.0, "transform cache size in second");
DEFINE_double(obs_max_local_pose_extrapolation_latency, 0.15,
              "max local pose extrapolation period in second");
DEFINE_bool(obs_enable_local_pose_extrapolation, true,
            "use local pose extrapolation");
DEFINE_bool(hardware_trigger, true, "camera trigger method");

namespace {

apollo::transform::PoseCacheOptions BuildPoseCacheOptions() {
  apollo::transform::PoseCacheOptions options;
  options.cache_duration_sec = FLAGS_obs_transform_cache_size;
  options.max_extrapolation_sec =
      FLAGS_obs_max_local_pose_extrapolation_latency;
  options.query_timeout_sec = static_cast<float>(FLAGS_obs_tf2_buff_size);
  return options;
}

}  // namespace

void TransformWrapper::Init(
    const std::string& sensor2novatel_tf2_child_frame_id) {
  sensor2novatel_tf2_frame_id_ = FLAGS_obs_sensor2novatel_tf2_frame_id;
  sensor2novatel_tf2_child_frame_id_ = sensor2novatel_tf2_child_frame_id;
  novatel2world_tf2_frame_id_ = FLAGS_obs_novatel2world_tf2_frame_id;
  novatel2world_tf2_child_frame_id_ =
      FLAGS_obs_novatel2world_tf2_child_frame_id;
  inited_ = true;
}

void TransformWrapper::Init(
    const std::string& sensor2novatel_tf2_frame_id,
    const std::string& sensor2novatel_tf2_child_frame_id,
    const std::string& novatel2world_tf2_frame_id,
    const std::string& novatel2world_tf2_child_frame_id) {
  sensor2novatel_tf2_frame_id_ = sensor2novatel_tf2_frame_id;
  sensor2novatel_tf2_child_frame_id_ = sensor2novatel_tf2_child_frame_id;
  novatel2world_tf2_frame_id_ = novatel2world_tf2_frame_id;
  novatel2world_tf2_child_frame_id_ = novatel2world_tf2_child_frame_id;
  inited_ = true;
}

bool TransformWrapper::GetSensor2worldTrans(
    double timestamp, Eigen::Affine3d* sensor2world_trans,
    Eigen::Affine3d* novatel2world_trans) {
  if (!inited_) {
    AERROR << "TransformWrapper not Initialized,"
           << " unable to call GetSensor2worldTrans.";
    return false;
  }

  if (sensor2novatel_extrinsics_ == nullptr) {
    StampedTransform trans_sensor2novatel;
    if (!QueryTrans(timestamp, &trans_sensor2novatel,
                    sensor2novatel_tf2_frame_id_,
                    sensor2novatel_tf2_child_frame_id_)) {
      return false;
    }
    sensor2novatel_extrinsics_.reset(new Eigen::Affine3d);
    *sensor2novatel_extrinsics_ =
        trans_sensor2novatel.translation * trans_sensor2novatel.rotation;
    AINFO << "Get sensor2novatel extrinsics successfully.";
  }

  StampedTransform trans_novatel2world;
  trans_novatel2world.timestamp = timestamp;
  Eigen::Affine3d novatel2world;
  if (!GetTrans(timestamp, &novatel2world, novatel2world_tf2_frame_id_,
                novatel2world_tf2_child_frame_id_)) {
    return false;
  }

  *sensor2world_trans = novatel2world * (*sensor2novatel_extrinsics_);
  if (novatel2world_trans != nullptr) {
    *novatel2world_trans = novatel2world;
  }
  AINFO << "Get pose timestamp: " << timestamp << ", pose: \n"
        << (*sensor2world_trans).matrix();
  return true;
}

bool TransformWrapper::GetExtrinsics(Eigen::Affine3d* trans) {
  if (!inited_ || trans == nullptr || sensor2novatel_extrinsics_ == nullptr) {
    AERROR << "TransformWrapper get extrinsics failed";
    return false;
  }
  *trans = *sensor2novatel_extrinsics_;
  return true;
}

bool TransformWrapper::GetTrans(double timestamp, Eigen::Affine3d* trans,
                                const std::string& frame_id,
                                const std::string& child_frame_id) {
  if (trans == nullptr) {
    return false;
  }

  auto* pose_cache = GetOrCreateTransformCache(frame_id, child_frame_id);
  if (pose_cache == nullptr) {
    return false;
  }

  apollo::transform::PoseCacheStatus cache_status =
      apollo::transform::PoseCacheStatus::kEmpty;
  if (pose_cache->QueryCachedStrict(timestamp, trans, &cache_status)) {
    return true;
  }

  StampedTransform transform;
  if (!QueryTrans(timestamp, &transform, frame_id, child_frame_id)) {
    if (!FLAGS_obs_enable_local_pose_extrapolation ||
        !pose_cache->QueryCached(timestamp, trans, &cache_status)) {
      return false;
    }
    return true;
  }

  *trans = transform.translation * transform.rotation;
  const double cache_timestamp =
      transform.timestamp > 0.0 ? transform.timestamp : timestamp;
  pose_cache->StorePose(cache_timestamp, *trans);
  return true;
}

bool TransformWrapper::QueryTrans(double timestamp, StampedTransform* trans,
                                  const std::string& frame_id,
                                  const std::string& child_frame_id) {
  cyber::Time query_time(timestamp);
  std::string err_string;
  double latest_buffer_time = 0.0;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id, query_time,
                                 static_cast<float>(FLAGS_obs_tf2_buff_size),
                                 &err_string)) {
    apollo::transform::TransformStamped latest_transform;
    if (!FLAGS_hardware_trigger) {
      try {
        latest_transform = tf2_buffer_->lookupTransform(
            frame_id, child_frame_id, apollo::cyber::Time(0));
        latest_buffer_time = latest_transform.header().timestamp_sec();
      } catch (tf2::TransformException& ex) {
        AERROR << ex.what();
        return false;
      }
    }
    // In simulation mode, use the latest transform information if query failed.
    if (!cyber::common::GlobalData::Instance()->IsRealityMode()) {
      query_time = cyber::Time(0);
    } else if (!FLAGS_hardware_trigger &&
               (timestamp - latest_buffer_time < 0.015) &&
               (timestamp - latest_buffer_time > 0)) {
      // soft trigger and the latency is within the tolerance range
      query_time = apollo::cyber::Time(0);
    } else {
      AERROR << "Can not find transform. " << FORMAT_TIMESTAMP(timestamp)
             << " frame_id: " << frame_id
             << " child_frame_id: " << child_frame_id
             << " Error info: " << err_string;
      return false;
    }
  }

  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform =
        tf2_buffer_->lookupTransform(frame_id, child_frame_id, query_time);

    trans->translation =
        Eigen::Translation3d(stamped_transform.transform().translation().x(),
                             stamped_transform.transform().translation().y(),
                             stamped_transform.transform().translation().z());
    trans->rotation =
        Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
                           stamped_transform.transform().rotation().qx(),
                           stamped_transform.transform().rotation().qy(),
                           stamped_transform.transform().rotation().qz());
    trans->timestamp = stamped_transform.header().timestamp_sec();
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  return true;
}

apollo::transform::TransformFrameCache*
TransformWrapper::GetOrCreateTransformCache(const std::string& frame_id,
                                            const std::string& child_frame_id) {
  if (frame_id.empty() || child_frame_id.empty()) {
    return nullptr;
  }

  const std::string cache_key = MakeTransformCacheKey(frame_id, child_frame_id);
  std::lock_guard<std::mutex> lock(transform_cache_mutex_);
  auto it = transform_caches_.find(cache_key);
  if (it != transform_caches_.end()) {
    return it->second.get();
  }

  auto cache = std::make_unique<apollo::transform::TransformFrameCache>(
      tf2_buffer_, frame_id, child_frame_id, BuildPoseCacheOptions());
  auto* cache_ptr = cache.get();
  transform_caches_.emplace(cache_key, std::move(cache));
  return cache_ptr;
}

std::string TransformWrapper::MakeTransformCacheKey(
    const std::string& frame_id, const std::string& child_frame_id) {
  return frame_id + "->" + child_frame_id;
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

  StampedTransform transform;
  bool status = QueryTrans(0.0, &transform, frame_id, child_frame_id);
  if (status) {
    *trans = transform.translation * transform.rotation;
  }
  return status;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
