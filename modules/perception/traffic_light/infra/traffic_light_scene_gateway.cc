/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/perception/traffic_light/infra/traffic_light_scene_gateway.h"

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace infra {

using apollo::cyber::Clock;
using apollo::perception::common::SensorManager;

bool TrafficLightSceneGateway::Init(
    const domain::TrafficLightComponentConfig& config,
    const camera::TrafficLightPreprocessorInitOptions& init_options) {
  config_ = config;
  if (!preprocessor_.Init(config.tl_preprocessor_name, init_options)) {
    return false;
  }
  preprocessor_option_.image_borders_size = &image_border_sizes_;

  const auto& cameras = preprocessor_.GetCameraNamesByDescendingFocalLen();
  if (cameras.empty()) {
    return false;
  }
  SensorManager* sensor_manager = SensorManager::Instance();
  for (size_t i = 0; i < config.camera_names.size(); ++i) {
    const auto& camera_name = config.camera_names[i];
    if (!sensor_manager->IsSensorExist(camera_name)) {
      return false;
    }
    std::shared_ptr<apollo::perception::TransformWrapper> wrapper(
        new apollo::perception::TransformWrapper);
    wrapper->Init(sensor_manager->GetFrameId(camera_name));
    camera2world_trans_wrapper_map_[camera_name] = wrapper;
    image_border_sizes_[camera_name] =
        camera_name == cameras.back() ? 0 : config.default_image_border_size;
    last_sub_camera_image_ts_[camera_name] = 0.0;
  }

  hd_map_ = map::HDMapInput::Instance();
  return hd_map_ != nullptr && hd_map_->Init();
}

void TrafficLightSceneGateway::RecordCameraHeartbeat(
    const std::string& camera_name, double timestamp) {
  last_sub_camera_image_ts_[camera_name] = timestamp;
}

bool TrafficLightSceneGateway::CheckCameraImageStatus(
    double timestamp, double interval, const std::string& camera_name) {
  for (const auto& entry : last_sub_camera_image_ts_) {
    if (entry.second < 1.0 || timestamp - entry.second > interval) {
      preprocessor_.SetCameraWorkingFlag(entry.first, false);
    }
  }

  bool is_camera_working = false;
  if (!preprocessor_.GetCameraWorkingFlag(camera_name, &is_camera_working)) {
    return false;
  }
  if (!is_camera_working) {
    return preprocessor_.SetCameraWorkingFlag(camera_name, true);
  }
  return true;
}

bool TrafficLightSceneGateway::UpdateCameraSelection(double timestamp,
                                                     camera::CameraFrame* frame) {
  const double current_ts = Clock::NowInSeconds();
  if (last_query_tf_ts_ > 0.0 &&
      current_ts - last_query_tf_ts_ < config_.query_tf_interval_seconds) {
    return true;
  }
  camera::CarPose pose;
  std::vector<apollo::hdmap::Signal> signals;
  if (!QueryPoseAndSignals(timestamp, &pose, &signals)) {
    return false;
  }
  last_query_tf_ts_ = current_ts;
  GenerateTrafficLights(signals, &frame->traffic_lights);
  return preprocessor_.UpdateCameraSelection(pose, preprocessor_option_,
                                             &frame->traffic_lights);
}

bool TrafficLightSceneGateway::VerifyLightsProjection(
    double timestamp, const std::string& camera_name,
    camera::CameraFrame* frame) {
  camera::CarPose pose;
  std::vector<apollo::hdmap::Signal> signals;
  if (!QueryPoseAndSignals(timestamp, &pose, &signals)) {
    return false;
  }
  GenerateTrafficLights(signals, &frame->traffic_lights);
  return preprocessor_.UpdateLightsProjection(pose, preprocessor_option_,
                                              camera_name,
                                              &frame->traffic_lights);
}

bool TrafficLightSceneGateway::SyncInformation(double timestamp,
                                               const std::string& camera_name) {
  return preprocessor_.SyncInformation(timestamp, camera_name);
}

bool TrafficLightSceneGateway::QueryPoseAndSignals(
    double timestamp, camera::CarPose* pose,
    std::vector<apollo::hdmap::Signal>* signals) {
  if (!GetCarPose(timestamp, pose)) {
    return false;
  }
  Eigen::Vector3d car_position = pose->getCarPosition();
  if (!hd_map_->GetSignals(car_position, 150.0, signals)) {
    if (timestamp - last_signals_ts_ < config_.valid_hdmap_interval) {
      *signals = last_signals_;
    } else {
      return false;
    }
  } else {
    last_signals_ts_ = timestamp;
    last_signals_ = *signals;
  }
  return true;
}

void TrafficLightSceneGateway::GenerateTrafficLights(
    const std::vector<apollo::hdmap::Signal>& signals,
    std::vector<base::TrafficLightPtr>* lights) {
  lights->clear();
  for (const auto& signal : signals) {
    base::TrafficLightPtr light(new base::TrafficLight);
    light->id = signal.id().id();
    for (int i = 0; i < signal.boundary().point_size(); ++i) {
      base::PointXYZID point;
      point.x = signal.boundary().point(i).x();
      point.y = signal.boundary().point(i).y();
      point.z = signal.boundary().point(i).z();
      light->region.points.push_back(point);
    }
    light->semantic = 0;
    lights->push_back(light);
    stoplines_ = signal.stop_line();
  }
}

bool TrafficLightSceneGateway::GetCarPose(double timestamp,
                                          camera::CarPose* pose) {
  Eigen::Matrix4d pose_matrix;
  if (!GetPoseFromTF(timestamp, config_.tf2_frame_id, config_.tf2_child_frame_id,
                     &pose_matrix)) {
    return false;
  }
  pose->timestamp_ = timestamp;
  pose->pose_ = pose_matrix;
  int state = 0;
  Eigen::Affine3d affine3d_trans;
  for (const auto& camera_name : config_.camera_names) {
    auto wrapper = camera2world_trans_wrapper_map_[camera_name];
    if (wrapper->GetSensor2worldTrans(timestamp, &affine3d_trans)) {
      pose->c2w_poses_[camera_name] = affine3d_trans.matrix();
      ++state;
    }
  }
  return state > 0;
}

bool TrafficLightSceneGateway::GetPoseFromTF(
    double timestamp, const std::string& frame_id,
    const std::string& child_frame_id, Eigen::Matrix4d* pose_matrix) {
  apollo::cyber::Time query_time(timestamp);
  std::string err_string;
  if (!tf2_buffer_->canTransform(frame_id, child_frame_id, query_time,
                                 static_cast<float>(config_.tf2_timeout_second),
                                 &err_string)) {
    return false;
  }
  apollo::transform::TransformStamped stamped_transform =
      tf2_buffer_->lookupTransform(frame_id, child_frame_id, query_time);
  Eigen::Translation3d translation(
      stamped_transform.transform().translation().x(),
      stamped_transform.transform().translation().y(),
      stamped_transform.transform().translation().z());
  Eigen::Quaterniond rotation(stamped_transform.transform().rotation().qw(),
                              stamped_transform.transform().rotation().qx(),
                              stamped_transform.transform().rotation().qy(),
                              stamped_transform.transform().rotation().qz());
  *pose_matrix = (translation * rotation).matrix();
  return true;
}

double TrafficLightSceneGateway::ComputeStoplineDistance(
    double timestamp, const std::string& camera_name) {
  if (stoplines_.empty()) {
    return -1;
  }
  camera::CarPose pose;
  if (!GetCarPose(timestamp, &pose) ||
      pose.c2w_poses_.find(camera_name) == pose.c2w_poses_.end()) {
    return -1;
  }
  const auto& stopline = stoplines_.Get(0);
  if (stopline.segment().empty() || !stopline.segment(0).has_line_segment() ||
      stopline.segment(0).line_segment().point().empty()) {
    return -1;
  }
  Eigen::Vector3d stopline_pt(stopline.segment(0).line_segment().point(0).x(),
                              stopline.segment(0).line_segment().point(0).y(),
                              stopline.segment(0).line_segment().point(0).z());
  Eigen::Vector3d stopline_pt_cam =
      (pose.c2w_poses_.at(camera_name).inverse() *
       Eigen::Vector4d(stopline_pt(0), stopline_pt(1), stopline_pt(2), 1.0))
          .head(3);
  return stopline_pt_cam(2);
}

}  // namespace infra
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
