/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/common_msgs/map_msgs/map_geometry.pb.h"
#include "modules/common_msgs/map_msgs/map_signal.pb.h"
#include "modules/perception/map/hdmap/hdmap_input.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/traffic_light/domain/traffic_light_component_config.h"
#include "modules/perception/traffic_light/infra/preprocessor_gateway.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace infra {

class TrafficLightSceneGateway {
 public:
  bool Init(const domain::TrafficLightComponentConfig& config,
            const camera::TrafficLightPreprocessorInitOptions& init_options);

  void RecordCameraHeartbeat(const std::string& camera_name, double timestamp);
  bool CheckCameraImageStatus(double timestamp, double interval,
                              const std::string& camera_name);
  bool UpdateCameraSelection(double timestamp, camera::CameraFrame* frame);
  bool VerifyLightsProjection(double timestamp, const std::string& camera_name,
                              camera::CameraFrame* frame);
  bool SyncInformation(double timestamp, const std::string& camera_name);
  bool GetCarPose(double timestamp, camera::CarPose* pose);
  double ComputeStoplineDistance(double timestamp,
                                 const std::string& camera_name);

  const std::map<std::string, int>& image_border_sizes() const {
    return image_border_sizes_;
  }

 private:
  bool QueryPoseAndSignals(double timestamp, camera::CarPose* pose,
                           std::vector<apollo::hdmap::Signal>* signals);
  bool GetPoseFromTF(double timestamp, const std::string& frame_id,
                     const std::string& child_frame_id,
                     Eigen::Matrix4d* pose_matrix);
  void GenerateTrafficLights(const std::vector<apollo::hdmap::Signal>& signals,
                             std::vector<base::TrafficLightPtr>* lights);

 private:
  domain::TrafficLightComponentConfig config_;
  PreprocessorGateway preprocessor_;
  apollo::perception::map::HDMapInput* hd_map_ = nullptr;
  Buffer* tf2_buffer_ = Buffer::Instance();
  camera::TLPreprocessorOption preprocessor_option_;
  std::map<std::string, std::shared_ptr<apollo::perception::TransformWrapper>>
      camera2world_trans_wrapper_map_;
  std::map<std::string, int> image_border_sizes_;
  std::map<std::string, double> last_sub_camera_image_ts_;
  double last_query_tf_ts_ = 0.0;
  double last_signals_ts_ = -1.0;
  std::vector<apollo::hdmap::Signal> last_signals_;
  ::google::protobuf::RepeatedPtrField<apollo::hdmap::Curve> stoplines_;
};

}  // namespace infra
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
