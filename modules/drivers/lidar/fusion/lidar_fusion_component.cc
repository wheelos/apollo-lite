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
#include <future>
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
  target_pointcloud_ = std::make_shared<apollo::drivers::PointCloud>();
  target_pointcloud_->mutable_point()->Reserve(reserved_point_size_);

  return true;
}

bool LidarFusionComponent::Proc(
    const std::shared_ptr<apollo::drivers::PointCloud>& main_pc) {
  auto& target_pc = target_pointcloud_;
  int reserved_size = target_pc->point_size();
  int target_size = main_pc->point_size();
  if (reserved_size < target_size) {
    // new fresh struct, increase point size to fit main pointcloud
    FitPointSize(target_pc, target_size);
    reserved_size = target_size;
  }
  // copy main pointcloud to target
  target_pc->mutable_header()->CopyFrom(main_pc->header());
  target_pc->set_frame_id(main_pc->frame_id());
  target_pc->set_is_dense(main_pc->is_dense());
  target_pc->set_measurement_time(main_pc->measurement_time());
  target_pc->set_height(main_pc->height());
  for (int i = 0; i < target_size; ++i) {
    const auto& src = main_pc->point(i);
    auto* dst = target_pc->mutable_point(i);
    dst->set_x(src.x());
    dst->set_y(src.y());
    dst->set_z(src.z());
    dst->set_intensity(src.intensity());
    dst->set_timestamp(src.timestamp());
  }
  target_pc->set_width(target_size);

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
    target_size = 0;
    Fusion(target_pc, main_pc, &target_size);
  } else if (lidar_system_offset_ns_ != 0) {
    for (auto& point : *target_pc->mutable_point()) {
      point.set_timestamp(GetPointTimestamp(point.timestamp()));
    }
  }
  auto fusion_readers = readers_;
  auto start_time = cyber::Time::Now().ToSecond();
  std::vector<std::shared_ptr<apollo::drivers::PointCloud>> source_pcs;
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
          // save pointcloud first, then fusion, to reduce lock time
          source_pcs.emplace_back(source);
          itr = fusion_readers.erase(itr);
        }
      } else {
        ++itr;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  int fusion_size = 0;
  for (const auto& pc : source_pcs) {
    fusion_size += pc->point_size();
  }
  FitPointSize(target_pc, target_size + fusion_size);

  // fusion
  if (config_.enable_parallel_fusion() && !config_.enable_nan_filter()) {
    // nan filter breaks the order and size pre-allocation, not support parallel

    std::vector<int> begin_sizes;
    begin_sizes.resize(source_pcs.size());
    std::vector<std::future<void>> futures;
    for (size_t i = 0; i < source_pcs.size(); ++i) {
      const auto& source = source_pcs[i];
      Eigen::Affine3f pose;
      if (GetPoseAffine(target_pc, source, &pose)) {
        // if failed to get pose, skip this source pointcloud
        // fusion from target_size to make sure order correct in parallel
        begin_sizes[i] = target_size;
        int begin_size = begin_sizes[i];
        std::future<void> future = apollo::cyber::Async([&, target_pc, source,
                                                         pose, begin_size]() {
          // each task has its own begin size, make a copy to avoid
          // accessing after destruction
          int begin_size_inner = begin_size;
          this->AppendPointCloud(target_pc, source, pose, &begin_size_inner);
        });
        target_size += source->point_size();
        futures.push_back(std::move(future));
      }
    }
    for (auto& future : futures) {
      future.get();
    }
  } else {
    for (const auto& source : source_pcs) {
      Fusion(target_pc, source, &target_size);
    }
  }
  // trim pointcloud, after fusion, the size may smaller than reserved
  FitPointSize(target_pc, target_size);

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

void LidarFusionComponent::FitPointSize(
    std::shared_ptr<apollo::drivers::PointCloud>& target_pc, int size) {
  int current_size = target_pc->point_size();
  int diff_size = size - current_size;
  if (diff_size > 0) {
    for (int i = 0; i < diff_size; ++i) {
      target_pc->add_point();
    }
  } else if (diff_size < 0) {
    target_pc->mutable_point()->DeleteSubrange(size, -diff_size);
  }
}

void LidarFusionComponent::AppendPointCloud(
    std::shared_ptr<apollo::drivers::PointCloud> target,
    const std::shared_ptr<apollo::drivers::PointCloud> source,
    const Eigen::Affine3f& pose, int* target_size) {
  int source_points_size = source->point_size();
  int fused_points_size = 0;
  int filtered_out_points_size = 0;
  if (std::isnan(pose(0, 0))) {
    // invalid transform, copy only
    for (int i = 0; i < source_points_size; ++i) {
      const auto& point = source->point(i);
      if ((std::isnan(point.x()) || std::isnan(point.y()) ||
           std::isnan(point.z())) &&
          config_.enable_nan_filter()) {
        ++filtered_out_points_size;
        continue;
      }
      apollo::drivers::PointXYZIT* np =
          target->mutable_point(*target_size + fused_points_size++);
      np->set_intensity(point.intensity());
      np->set_timestamp(GetPointTimestamp(point.timestamp()));
      np->set_x(point.x());
      np->set_y(point.y());
      np->set_z(point.z());
    }
  } else {
    const Eigen::Matrix3f& rot = pose.linear();
    const Eigen::Vector3f& trans = pose.translation();
    float r00 = rot.coeff(0, 0);
    float r01 = rot.coeff(0, 1);
    float r02 = rot.coeff(0, 2);
    float tx = trans.x();
    float r10 = rot.coeff(1, 0);
    float r11 = rot.coeff(1, 1);
    float r12 = rot.coeff(1, 2);
    float ty = trans.y();
    float r20 = rot.coeff(2, 0);
    float r21 = rot.coeff(2, 1);
    float r22 = rot.coeff(2, 2);
    float tz = trans.z();
    for (int i = 0; i < source_points_size; ++i) {
      const auto& point = source->point(i);
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        if (config_.enable_nan_filter()) {
          ++filtered_out_points_size;
          continue;
        } else {
          apollo::drivers::PointXYZIT* np =
              target->mutable_point(*target_size + fused_points_size++);
          np->set_intensity(point.intensity());
          np->set_timestamp(GetPointTimestamp(point.timestamp()));
          np->set_x(point.x());
          np->set_y(point.y());
          np->set_z(point.z());
        }
      } else {
        apollo::drivers::PointXYZIT* np =
            target->mutable_point(*target_size + fused_points_size++);
        np->set_intensity(point.intensity());
        np->set_timestamp(GetPointTimestamp(point.timestamp()));
        np->set_x(r00 * point.x() + r01 * point.y() + r02 * point.z() + tx);
        np->set_y(r10 * point.x() + r11 * point.y() + r12 * point.z() + ty);
        np->set_z(r20 * point.x() + r21 * point.y() + r22 * point.z() + tz);
      }
    }
  }
  if (filtered_out_points_size > 0) {
    ADEBUG << "Fusion: filter out " << filtered_out_points_size << " points.";
    // if filter enabled, the size may smaller than reserved size, need trim
    *target_size += (fused_points_size);
  } else {
    // no filter
    *target_size += (source_points_size);
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

bool LidarFusionComponent::GetPoseAffine(
    std::shared_ptr<apollo::drivers::PointCloud>& target_pc,
    const std::shared_ptr<apollo::drivers::PointCloud>& source_pc,
    Eigen::Affine3f* pose) {
  auto& target_frame_id = target_pc->header().frame_id();
  auto& source_frame_id = source_pc->header().frame_id();
  // query tf
  auto transform = static_tf_map_.find(source_frame_id);
  if (transform == static_tf_map_.end()) {
    Eigen::Affine3d pose_d;
    if (!QueryPoseAffine(0, target_frame_id, source_frame_id, &pose_d)) {
      AERROR << "Failed to query pose from TF2 for source frame: "
             << source_frame_id << " to target frame: " << target_frame_id;
      return false;
    }
    static_tf_map_[source_frame_id] = pose_d.cast<float>();
  }
  *pose = static_tf_map_[source_frame_id];
  return true;
}

bool LidarFusionComponent::Fusion(
    std::shared_ptr<apollo::drivers::PointCloud>& target_pc,
    const std::shared_ptr<apollo::drivers::PointCloud>& source_pc,
    int* target_size) {
  Eigen::Affine3f pose;
  if (!GetPoseAffine(target_pc, source_pc, &pose)) {
    return false;
  }
  AppendPointCloud(target_pc, source_pc, pose, target_size);
  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
