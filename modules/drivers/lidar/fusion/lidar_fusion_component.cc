/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/drivers/lidar/fusion/lidar_fusion_component.h"

#include <chrono>
#include <memory>
#include <vector>

#include "Eigen/Eigen"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "modules/transform/buffer.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool LidarFusionComponent::Init() {
  // Initialize the component, e.g., load configuration, set up readers/writers
  if (!GetProtoConfig(&config_)) {
    AERROR << "Load config file " << ConfigFilePath() << " failed.";
    return false;
  }
  tf2_buffer_ptr_ = apollo::transform::Buffer::Instance();

  writer_ = node_->CreateWriter<apollo::drivers::PointCloud>(
      config_.output_channel());

  for (const auto& channel : config_.input_channel()) {
    auto reader = node_->CreateReader<apollo::drivers::PointCloud>(channel);
    readers_.emplace_back(reader);
  }

  point_cloud_pool_.resize(pool_size_);
  for (size_t i = 0; i < pool_size_; ++i) {
    point_cloud_pool_[i] = std::make_shared<apollo::drivers::PointCloud>();
    point_cloud_pool_[i]->mutable_point()->Reserve(reserved_point_size_);
  }

  return true;
}

bool LidarFusionComponent::Proc(
    const std::shared_ptr<apollo::drivers::PointCloud>& main_pc) {
  auto target_pc = point_cloud_pool_[pool_index_++ % pool_size_];
  target_pc->Clear();
  target_pc->CopyFrom(*main_pc);

  // set measure timestamp
  lidar_system_offset_ns_ = 0;
  if (config_.has_use_system_clock() && config_.use_system_clock()) {
    lidar_system_offset_ns_ =
        target_pc->header().lidar_timestamp() -
        static_cast<uint64_t>(target_pc->header().timestamp_sec() * 1e9);
    target_pc->set_measurement_time(target_pc->header().timestamp_sec());
  } else {
    target_pc->set_measurement_time(main_pc->measurement_time());
  }
  target_pc->mutable_header()->set_lidar_timestamp(
      main_pc->header().lidar_timestamp());
  if (config_.has_target_frame_id() &&
      config_.target_frame_id() != target_pc->header().frame_id()) {
    target_pc->mutable_header()->set_frame_id(config_.target_frame_id());
    target_pc->clear_point();
    Fusion(target_pc, main_pc);
  } else if (lidar_system_offset_ns_ != 0) {
    for (auto& point : *target_pc->mutable_point()) {
      point.set_timestamp(GetPointTimestamp(point.timestamp()));
    }
  }
  auto fusion_readers = readers_;
  auto start_time = cyber::Time::Now().ToSecond();
  while ((cyber::Time::Now().ToSecond() - start_time) <
             config_.wait_time_seconds() &&
         fusion_readers.size() > 0) {
    for (auto itr = fusion_readers.begin(); itr != fusion_readers.end();) {
      (*itr)->Observe();
      if (!(*itr)->Empty()) {
        auto source = (*itr)->GetLatestObserved();
        if (config_.drop_expired_data() && IsExpired(target_pc, source)) {
          ++itr;
        } else {
          Fusion(target_pc, source);
          itr = fusion_readers.erase(itr);
        }
      } else {
        ++itr;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  auto diff =
      cyber::Time::Now().ToNanosecond() - target_pc->header().lidar_timestamp();
  AINFO << "Pointcloud fusion diff: " << diff / 1e6 << "ms";
  target_pc->mutable_header()->set_sequence_num(
      target_pc->header().sequence_num() + 1);
  target_pc->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  target_pc->set_height(main_pc->height());
  target_pc->set_width(target_pc->point_size() / target_pc->height());
  target_pc->set_is_dense(main_pc->is_dense());
  writer_->Write(target_pc);
  return true;
}

bool LidarFusionComponent::IsExpired(
    const std::shared_ptr<apollo::drivers::PointCloud>& target,
    const std::shared_ptr<apollo::drivers::PointCloud>& source) {
  auto diff = target->measurement_time() - source->measurement_time();
  return diff * 1e3 > config_.max_interval_ms();
}

uint64_t LidarFusionComponent::GetPointTimestamp(const uint64_t& timestamp) {
  if (lidar_system_offset_ns_ == 0) {
    return timestamp;
  }
  return static_cast<uint64_t>(timestamp - lidar_system_offset_ns_);
}

void LidarFusionComponent::AppendPointCloud(
    std::shared_ptr<apollo::drivers::PointCloud> target,
    const std::shared_ptr<apollo::drivers::PointCloud> source,
    const Eigen::Affine3f& pose) {
  if (std::isnan(pose(0, 0))) {
    for (auto& point : source->point()) {
      apollo::drivers::PointXYZIT* point_new = target->add_point();
      point_new->set_intensity(point.intensity());
      point_new->set_timestamp(GetPointTimestamp(point.timestamp()));
      point_new->set_x(point.x());
      point_new->set_y(point.y());
      point_new->set_z(point.z());
    }
  } else {
    for (auto& point : source->point()) {
      if (std::isnan(point.x())) {
        apollo::drivers::PointXYZIT* point_new = target->add_point();
        point_new->set_intensity(point.intensity());
        point_new->set_timestamp(GetPointTimestamp(point.timestamp()));
        point_new->set_x(point.x());
        point_new->set_y(point.y());
        point_new->set_z(point.z());
      } else {
        apollo::drivers::PointXYZIT* point_new = target->add_point();
        point_new->set_intensity(point.intensity());
        point_new->set_timestamp(GetPointTimestamp(point.timestamp()));
        Eigen::Matrix<float, 3, 1> pt(point.x(), point.y(), point.z());
        point_new->set_x(static_cast<float>(
            pose(0, 0) * pt.coeffRef(0) + pose(0, 1) * pt.coeffRef(1) +
            pose(0, 2) * pt.coeffRef(2) + pose(0, 3)));
        point_new->set_y(static_cast<float>(
            pose(1, 0) * pt.coeffRef(0) + pose(1, 1) * pt.coeffRef(1) +
            pose(1, 2) * pt.coeffRef(2) + pose(1, 3)));
        point_new->set_z(static_cast<float>(
            pose(2, 0) * pt.coeffRef(0) + pose(2, 1) * pt.coeffRef(1) +
            pose(2, 2) * pt.coeffRef(2) + pose(2, 3)));
      }
    }
  }
}

bool LidarFusionComponent::QueryPoseAffine(const uint64_t& timestamp,
                                           const std::string& target_frame_id,
                                           const std::string& source_frame_id,
                                           Eigen::Affine3d* pose) {
  cyber::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_ptr_->canTransform(target_frame_id, source_frame_id,
                                     query_time, 2e-2, &err_string)) {
    AERROR << "Can not find transform, "
           << "target_frame_id: " << target_frame_id
           << ", source_frame_id: " << source_frame_id
           << ", Error info: " << err_string;
    return false;
  }
  apollo::transform::TransformStamped stamped_transform;
  try {
    stamped_transform = tf2_buffer_ptr_->lookupTransform(
        target_frame_id, source_frame_id, query_time);
  } catch (tf2::TransformException& ex) {
    AERROR << ex.what();
    return false;
  }
  *pose =
      Eigen::Translation3d(stamped_transform.transform().translation().x(),
                           stamped_transform.transform().translation().y(),
                           stamped_transform.transform().translation().z()) *
      Eigen::Quaterniond(stamped_transform.transform().rotation().qw(),
                         stamped_transform.transform().rotation().qx(),
                         stamped_transform.transform().rotation().qy(),
                         stamped_transform.transform().rotation().qz());
  return true;
}

bool LidarFusionComponent::Fusion(
    std::shared_ptr<apollo::drivers::PointCloud>& target_pc,
    const std::shared_ptr<apollo::drivers::PointCloud>& source_pc) {
  auto& target_frame_id = target_pc->header().frame_id();
  auto& source_frame_id = source_pc->header().frame_id();
  // query tf
  auto transform = static_tf_map_.find(source_frame_id);
  if (transform == static_tf_map_.end()) {
    Eigen::Affine3d pose;
    if (!QueryPoseAffine(0, target_frame_id, source_frame_id, &pose)) {
      AERROR << "Failed to query pose from TF2 for source frame: "
             << source_frame_id << " to target frame: " << target_frame_id;
      return false;
    }
    static_tf_map_[source_frame_id] = pose.cast<float>();
  }
  Eigen::Affine3f pose = static_tf_map_[source_frame_id];
  AppendPointCloud(target_pc, source_pc, pose);
  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
