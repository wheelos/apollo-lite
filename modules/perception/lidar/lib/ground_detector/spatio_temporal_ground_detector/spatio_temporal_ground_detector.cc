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

#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "cyber/common/file.h"
#include "modules/perception/common/point_cloud_processing/common.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/lidar_point_label.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;
using apollo::cyber::common::GetProtoFromFile;

// =============================================================================
// Initialization
// =============================================================================

bool SpatioTemporalGroundDetector::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }
  config_ = stage_config.spatio_temporal_ground_detector_config();
  return InitInternal(config_);
}

bool SpatioTemporalGroundDetector::Init(
    const GroundDetectorInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig("SpatioTemporalGroundDetector",
                                        &model_config));

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));

  std::string config_file = GetAbsolutePath(work_root, root_path);
  config_file =
      GetAbsolutePath(config_file, "spatio_temporal_ground_detector.conf");

  SpatioTemporalGroundDetectorConfig config_params;
  ACHECK(GetProtoFromFile(config_file, &config_params));

  return InitInternal(config_params);
}

bool SpatioTemporalGroundDetector::InitInternal(
    const SpatioTemporalGroundDetectorConfig& config) {
  // 1. Load Parameters
  ground_thres_ = config.ground_thres();
  use_roi_ = config.use_roi();
  use_ground_service_ = config.use_ground_service();
  near_range_dist_ = config.near_range_dist();
  near_range_ground_thres_ = config.near_range_ground_thres();
  middle_range_dist_ = config.middle_range_dist();
  middle_range_ground_thres_ = config.middle_range_ground_thres();

  publish_debug_cloud_ = config.enable_debug_non_ground_cloud();
  debug_cloud_channel_ = config.debug_cloud_channel();
  if (publish_debug_cloud_) {
    node_ = cyber::CreateNode("spatio_temporal_ground_detector");
    debug_writer_ =
        node_->CreateWriter<apollo::drivers::PointCloud>(debug_cloud_channel_);
    AINFO << "SpatioTemporalGroundDetector: publishing non-ground cloud on "
          << debug_cloud_channel_;
  }

  // 2. Setup Algorithm Parameters
  // Using make_unique for exception safety (though Apollo doesn't use
  // exceptions much)
  param_ = std::make_unique<common::PlaneFitGroundDetectorParam>();
  param_->roi_region_rad_x = config.roi_rad_x();
  param_->roi_region_rad_y = config.roi_rad_y();
  param_->roi_region_rad_z = config.roi_rad_z();
  // Prefer small_grid_size/big_grid_size if provided (10.0 parity).
  const uint32_t coarse_grid = config.has_small_grid_size()
                                   ? config.small_grid_size()
                                   : config.grid_size();
  param_->nr_grids_coarse = coarse_grid;
  if (config.has_big_grid_size()) {
    param_->nr_grids_fine = config.big_grid_size();
  }

  param_->nr_smooth_iter = config.nr_smooth_iter();
  param_->sample_region_z_lower = config.sample_region_z_lower();
  param_->sample_region_z_upper = config.sample_region_z_upper();
  param_->roi_near_rad = config.roi_near_rad();
  param_->planefit_orien_threshold = config.planefit_orien_threshold();
  if (config.has_planefit_dist_thres_near()) {
    param_->planefit_dist_threshold_near = config.planefit_dist_thres_near();
  }
  if (config.has_planefit_dist_thres_far()) {
    param_->planefit_dist_threshold_far = config.planefit_dist_thres_far();
  }
  if (config.has_z_compare_thres()) {
    param_->planefit_filter_threshold = config.z_compare_thres();
  }
  if (config.has_smooth_z_thres()) {
    param_->candidate_filter_threshold = config.smooth_z_thres();
  }
  if (config.has_inliers_min_threshold()) {
    param_->nr_inliers_min_threshold = config.inliers_min_threshold();
  }

  // 3. Init Core Detector
  pfdetector_ = std::make_unique<common::PlaneFitGroundDetector>(*param_);
  pfdetector_->Init();

  // 4. Pre-allocate Memory
  // 65536 is too small for VLP-128 or Hesai-128, use 256k or adaptive
  default_point_size_ = 262144;
  point_indices_temp_.resize(default_point_size_);
  data_.resize(default_point_size_ * 3);
  ground_height_signed_.resize(default_point_size_);

  ground_service_content_.Init(config.roi_rad_x(), config.roi_rad_y(),
                               coarse_grid, coarse_grid);
  return true;
}

void SpatioTemporalGroundDetector::PublishDebugCloud(
    const LidarFrame& frame, const std::vector<int>& non_ground_indices) {
  if (!publish_debug_cloud_ || debug_writer_ == nullptr) {
    return;
  }
  if (frame.cloud == nullptr) {
    return;
  }

  auto msg = std::make_shared<apollo::drivers::PointCloud>();
  auto* header = msg->mutable_header();
  header->set_timestamp_sec(frame.timestamp);
  header->set_module_name(name_);
  header->set_sequence_num(++debug_seq_num_);
  header->set_frame_id(frame.sensor_info.frame_id);

  msg->set_frame_id(frame.sensor_info.frame_id);
  msg->set_measurement_time(frame.timestamp);
  msg->set_is_dense(false);
  msg->set_height(1);
  msg->set_width(static_cast<uint32_t>(non_ground_indices.size()));

  const uint64_t ts =
      static_cast<uint64_t>(frame.timestamp * 1e9);  // seconds -> ns
  msg->mutable_point()->Reserve(non_ground_indices.size());

  const auto& points = frame.cloud->points();
  for (int idx : non_ground_indices) {
    if (idx < 0 || static_cast<size_t>(idx) >= points.size()) {
      continue;
    }
    const auto& pt = points[static_cast<size_t>(idx)];
    auto* proto_pt = msg->add_point();
    proto_pt->set_x(pt.x);
    proto_pt->set_y(pt.y);
    proto_pt->set_z(pt.z);
    if (pt.intensity > 0.0f) {
      proto_pt->set_intensity(static_cast<uint32_t>(pt.intensity));
    }
    proto_pt->set_timestamp(ts);
  }

  debug_writer_->Write(msg);
}

// =============================================================================
// Processing
// =============================================================================

bool SpatioTemporalGroundDetector::Process(DataFrame* data_frame) {
  if (!data_frame || !data_frame->lidar_frame) return false;
  GroundDetectorOptions options;
  return Detect(options, data_frame->lidar_frame);
}

bool SpatioTemporalGroundDetector::Detect(const GroundDetectorOptions& options,
                                          LidarFrame* frame) {
  if (!frame || !frame->cloud || !frame->world_cloud ||
      frame->world_cloud->empty()) {
    AWARN << "Input frame invalid for ground detection.";
    return false;
  }

  // 1. Determine Input Size & ROI
  // Check if ROI indices are actually populated if flag is set
  bool actual_use_roi = use_roi_ && !frame->roi_indices.indices.empty();
  const size_t num_points = actual_use_roi ? frame->roi_indices.indices.size()
                                           : frame->world_cloud->size();

  if (num_points == 0) return true;

  // 2. Auto-Grow Memory
  if (num_points > default_point_size_) {
    default_point_size_ = static_cast<size_t>(num_points * 1.2);
    point_indices_temp_.resize(default_point_size_);
    data_.resize(default_point_size_ * 3);
    ground_height_signed_.resize(default_point_size_);
    AINFO << "SpatioTemporalGroundDetector resized buffer to "
          << default_point_size_;
  }

  // 3. Prepare Center (Translation)
  // Use translation directly to act as the origin of the "stabilized local
  // frame"
  cloud_center_ = frame->lidar2world_pose.translation();
  const float cx = static_cast<float>(cloud_center_.x());
  const float cy = static_cast<float>(cloud_center_.y());
  const float cz = static_cast<float>(cloud_center_.z());

  // 4. Data Filling (Optimized)
  // Use pointer arithmetic for speed
  float* xyz_ptr = data_.data();
  int* idx_ptr = point_indices_temp_.data();
  unsigned int valid_count = 0;

  if (actual_use_roi) {
    const auto& indices = frame->roi_indices.indices;
    // Pipelining optimization: avoid bounds check inside loop
    const auto* world_pts = &frame->world_cloud->points();

    for (size_t i = 0; i < num_points; ++i) {
      int idx = indices[i];
      // Use direct access if possible, or operator[]
      const auto& pt = (*world_pts)[idx];

      idx_ptr[valid_count] = idx;

      // Unrolling: writing sequentially is cache-friendly
      *xyz_ptr++ = static_cast<float>(pt.x) - cx;
      *xyz_ptr++ = static_cast<float>(pt.y) - cy;
      *xyz_ptr++ = static_cast<float>(pt.z) - cz;

      valid_count++;
    }
  } else {
    // Process all points
    for (size_t i = 0; i < num_points; ++i) {
      const auto& pt = frame->world_cloud->at(i);

      idx_ptr[valid_count] = static_cast<int>(i);

      *xyz_ptr++ = static_cast<float>(pt.x) - cx;
      *xyz_ptr++ = static_cast<float>(pt.y) - cy;
      *xyz_ptr++ = static_cast<float>(pt.z) - cz;

      valid_count++;
    }
  }

  // 5. Core Algorithm Execution
  bool success = pfdetector_->Detect(data_.data(), ground_height_signed_.data(),
                                     valid_count, 3);

  if (!success) {
    AWARN << "Ground detector failed. Fallback: treat all as non-ground.";
    frame->non_ground_indices.indices.assign(
        point_indices_temp_.begin(), point_indices_temp_.begin() + valid_count);
    return false;  // Or true, depending on system requirements for degradation
  }

  // 6. Write Results (Batch Update)
  frame->non_ground_indices.indices.clear();
  frame->non_ground_indices.indices.reserve(valid_count);

  std::vector<int> non_ground_indices;
  if (publish_debug_cloud_) {
    non_ground_indices.reserve(valid_count);
  }

  // Get raw pointers/accessors to avoid repeated lookups
  auto& height_local = *frame->cloud->mutable_points_height();
  auto& height_world = *frame->world_cloud->mutable_points_height();
  auto& label_local = *frame->cloud->mutable_points_label();
  auto& label_world = *frame->world_cloud->mutable_points_label();

  const uint8_t ground_label = static_cast<uint8_t>(LidarPointLabel::GROUND);

  for (size_t i = 0; i < valid_count; ++i) {
    int idx = idx_ptr[i];
    float h = ground_height_signed_[i];

    // Using operator[] is safe here as idx comes from valid indices
    height_local[idx] = h;
    height_world[idx] = h;

    // Treat points above ground as non-ground; do not use abs().
    // This avoids misclassifying points slightly below the fitted plane.
    float threshold = ground_thres_;
    const auto& pt_local = frame->cloud->at(idx);
    // Use novatel(vehicle) frame longitudinal distance for adaptive
    // thresholding (closest to 10.0 logic). If extrinsics are identity, this
    // degenerates to lidar frame.
    const Eigen::Vector3d pt_novatel =
        frame->lidar2novatel_extrinsics *
        Eigen::Vector3d(pt_local.x, pt_local.y, pt_local.z);
    const float forward_dist = static_cast<float>(pt_novatel.y());
    if (forward_dist > 0.0f && forward_dist < near_range_dist_) {
      threshold = near_range_ground_thres_;
    } else if (forward_dist >= near_range_dist_ &&
               forward_dist < middle_range_dist_) {
      threshold = middle_range_ground_thres_;
    }

    if (h > threshold) {
      // Important: downstream modules (e.g. CloudMask::AddIndicesOfIndices)
      // interpret non_ground_indices as "indices-of-indices" into roi_indices
      // when roi_indices is used. Keep 10.0 semantics:
      // - when actual_use_roi: store position within roi_indices
      // - otherwise: store absolute point index in the full cloud
      frame->non_ground_indices.indices.push_back(
          actual_use_roi ? static_cast<int>(i) : idx);
      if (publish_debug_cloud_) {
        non_ground_indices.push_back(idx);
      }
    } else {
      label_local[idx] = ground_label;
      label_world[idx] = ground_label;
    }
  }

  if (publish_debug_cloud_) {
    PublishDebugCloud(*frame, non_ground_indices);
  }

  // 7. Update Global Service (Optional, for Map/Tracker)
  if (use_ground_service_) {
    UpdateGroundService(ground_service_content_);
  }
  return true;
}

void SpatioTemporalGroundDetector::UpdateGroundService(
    GroundServiceContent& content) {
  auto ground_service = SceneManager::Instance().Service("GroundService");
  if (!ground_service) return;

  content.grid_center_ = cloud_center_;
  content.grid_.Reset();

  GroundNode* node_base_ptr = content.grid_.DataPtr();
  unsigned int rows = pfdetector_->GetGridDimY();
  unsigned int cols = pfdetector_->GetGridDimX();

  // Assuming row-major layout
  for (unsigned int r = 0; r < rows; ++r) {
    for (unsigned int c = 0; c < cols; ++c) {
      const common::GroundPlaneLiDAR* plane = pfdetector_->GetGroundPlane(r, c);
      if (plane && plane->IsValid()) {
        GroundNode* node = node_base_ptr + (r * cols + c);
        // Plane params: [a, b, c, d] -> ax+by+cz+d=0
        node->params(0) = plane->params[0];
        node->params(1) = plane->params[1];
        node->params(2) = plane->params[2];
        node->params(3) = plane->params[3];
        node->confidence = 1.0;
      }
    }
  }
  ground_service->UpdateServiceContent(content);
}

PERCEPTION_REGISTER_GROUNDDETECTOR(SpatioTemporalGroundDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
