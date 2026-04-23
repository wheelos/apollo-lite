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
#include "modules/perception/lidar/lib/map_manager/map_manager.h"

#include "modules/perception/pipeline/proto/stage/map_manager_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;
using cyber::common::PathExists;

bool HdmapContextProvider::Init(const HdmapContextProviderInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  const std::string config_dir = GetAbsolutePath(work_root, root_path);
  std::string config_file =
      GetAbsolutePath(config_dir, "hdmap_context_provider.conf");
  if (!PathExists(config_file)) {
    config_file = GetAbsolutePath(config_dir, "map_manager.conf");
  }
  HdmapContextProviderConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  update_pose_ = config.update_pose();
  roi_search_distance_ = config.roi_search_distance();
  hdmap_input_ = map::HDMapInput::Instance();
  if (!hdmap_input_->Init()) {
    AINFO << "Failed to init hdmap input.";
    return false;
  }
  return true;
}

bool HdmapContextProvider::Init(const StageConfig& stage_config) {
  if (!Initialize(stage_config)) {
    return false;
  }

  hdmap_context_provider_config_ = stage_config.hdmap_context_provider_config();

  update_pose_ = hdmap_context_provider_config_.update_pose();
  roi_search_distance_ = hdmap_context_provider_config_.roi_search_distance();

  hdmap_input_ = map::HDMapInput::Instance();
  if (!hdmap_input_->Init()) {
    AERROR << "HdmapContextProvider::Init: Failed to initialize HDMapInput.";
    return false;
  }
  return true;
}

bool HdmapContextProvider::Process(DataFrame* data_frame) {
  if (data_frame == nullptr || data_frame->lidar_frame == nullptr) {
    AINFO << "DataFrame or LidarFrame is nullptr.";
    return false;
  }

  HdmapContextProviderOptions options;
  return Update(options, data_frame->lidar_frame);
}

bool HdmapContextProvider::Update(const HdmapContextProviderOptions& options,
                                  LidarFrame* frame) {
  if (!frame) {
    AINFO << "Frame is nullptr.";
    return false;
  }
  if (hdmap_input_ == nullptr) {
    AERROR << "HDMapInput is nullptr.";
    return false;
  }
  if (!frame->hdmap_struct) {
    frame->hdmap_struct.reset(new base::HdmapStruct);
  }
  if (update_pose_) {
    if (!QueryPose(&(frame->lidar2world_pose))) {
      AINFO << "Failed to query updated pose.";
    }
  }
  base::PointD point;
  point.x = frame->lidar2world_pose.translation()(0);
  point.y = frame->lidar2world_pose.translation()(1);
  point.z = frame->lidar2world_pose.translation()(2);
  if (!hdmap_input_->GetRoiHDMapStruct(point, roi_search_distance_,
                                       frame->hdmap_struct)) {
    frame->hdmap_struct->road_polygons.clear();
    frame->hdmap_struct->road_boundary.clear();
    frame->hdmap_struct->hole_polygons.clear();
    frame->hdmap_struct->junction_polygons.clear();
    AINFO << "Failed to get roi from hdmap.";
  }
  return true;
}
bool HdmapContextProvider::QueryPose(Eigen::Affine3d* sensor2world_pose) const {
  // TODO(daohu527): implement map-based pose refinement in a dedicated stage
  // or service. This stage currently acts as an HDMap context provider.
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
