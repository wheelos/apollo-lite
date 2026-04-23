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
#include "modules/drivers/lidar/compensator/lidar_compensator_component.h"

#include <memory>

#include "modules/drivers/lidar/compensator/proto/lidar_compensator_config.pb.h"

#include "cyber/time/time.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"

namespace apollo {
namespace drivers {
namespace lidar {

bool LidarCompensatorComponent::Init() {
  if (!GetProtoConfig(&config_)) {
    AWARN << "Load config failed, config file" << ConfigFilePath();
    return false;
  }

  writer_ = node_->CreateWriter<PointCloud>(config_.output_channel());
  compensator_.reset(new Compensator(config_));
  compensator_pool_.reset(
      new apollo::cyber::base::CCObjectPool<PointCloud>(pool_size_));
  compensator_pool_->ConstructAll();
  for (int i = 0; i < pool_size_; ++i) {
    auto point_cloud = compensator_pool_->GetObject();
    if (point_cloud == nullptr) {
      AERROR << "fail to getobject:" << i;
      return false;
    }
    point_cloud->mutable_point()->Reserve(config_.reserve_point_cloud_size());
  }
  return true;
}

bool LidarCompensatorComponent::Proc(
    const std::shared_ptr<PointCloud>& point_cloud) {
  const auto start_time = apollo::cyber::Time::Now();
  std::shared_ptr<PointCloud> point_cloud_compensated =
      compensator_pool_->GetObject();
  if (point_cloud_compensated == nullptr) {
    AWARN << "compensator fail to getobject, will be new";
    point_cloud_compensated = std::make_shared<PointCloud>();
    point_cloud_compensated->mutable_point()->Reserve(
        config_.reserve_point_cloud_size());
  }
  if (point_cloud_compensated == nullptr) {
    AWARN << "compensator point_cloud is nullptr";
    return false;
  }
  point_cloud_compensated->Clear();
  if (compensator_->MotionCompensation(point_cloud, point_cloud_compensated)) {
    const auto end_time = apollo::cyber::Time::Now();
    const auto diff = end_time - start_time;
    const auto meta_diff =
        end_time - apollo::cyber::Time(
                       point_cloud_compensated->header().lidar_timestamp());
    AINFO << "compenstator diff (ms):" << (diff.ToNanosecond() / 1e6)
          << ";meta (ns):"
          << point_cloud_compensated->header().lidar_timestamp()
          << ";meta diff (ms): " << (meta_diff.ToNanosecond() / 1e6);
    static common::LatencyRecorder latency_recorder(FLAGS_pointcloud_topic);
    latency_recorder.AppendLatencyRecord(
        point_cloud_compensated->header().lidar_timestamp(), start_time,
        end_time);

    point_cloud_compensated->mutable_header()->set_sequence_num(seq_);
    writer_->Write(point_cloud_compensated);
    seq_++;
  }

  return true;
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
