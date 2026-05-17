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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/perception/pipeline/proto/stage/spatio_temporal_ground_detector_config.pb.h"

#include "cyber/cyber.h"
#include "modules/perception/common/i_lib/pc/i_ground.h"
#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_ground_detector.h"
#include "modules/perception/lidar/lib/scene_manager/ground_service/ground_service.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"
#include "modules/perception/pipeline/stage.h"

namespace apollo {
namespace perception {
namespace lidar {

class SpatioTemporalGroundDetector : public BaseGroundDetector {
 public:
  SpatioTemporalGroundDetector() = default;
  virtual ~SpatioTemporalGroundDetector() =
      default;  // unique_ptr handles cleanup

  bool Init(const GroundDetectorInitOptions& options =
                GroundDetectorInitOptions()) override;

  bool Detect(const GroundDetectorOptions& options, LidarFrame* frame) override;

  bool Init(const StageConfig& stage_config) override;

  bool Process(DataFrame* data_frame) override;

  bool IsEnabled() const override { return enable_; }

  std::string Name() const override { return name_; }

 private:
  // Updates the global GroundService for other modules (e.g. MapManager,
  // Tracker)
  void UpdateGroundService(GroundServiceContent& content);

  void PublishDebugCloud(const LidarFrame& frame,
                         const std::vector<int>& ground_indices);

  // Internal init logic to share between two Init overrides
  bool InitInternal(const SpatioTemporalGroundDetectorConfig& config);

  // Members
  std::unique_ptr<common::PlaneFitGroundDetectorParam> param_;
  std::unique_ptr<common::PlaneFitGroundDetector> pfdetector_;

  // Reusable buffers to avoid allocation
  std::vector<float> data_;
  std::vector<float> ground_height_signed_;
  std::vector<int> point_indices_temp_;

  // Params
  bool use_roi_ = true;
  bool use_ground_service_ = false;
  float ground_thres_ = 0.25f;
  float near_range_dist_ = 10.0f;
  float near_range_ground_thres_ = 0.25f;
  float middle_range_dist_ = 20.0f;
  float middle_range_ground_thres_ = 0.25f;
  size_t default_point_size_ = 320000;
  Eigen::Vector3d cloud_center_ = Eigen::Vector3d::Zero();
  GroundServiceContent ground_service_content_;

  bool publish_debug_cloud_ = false;
  std::string debug_cloud_channel_;
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>>
      debug_writer_ = nullptr;
  uint32_t debug_seq_num_ = 0;

  SpatioTemporalGroundDetectorConfig config_;
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
