// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/perception/onboard/transform_wrapper/sensor_calibration.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "modules/common/vehicle_calibration/lidar_height.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/onboard/transform_wrapper/transform_wrapper.h"

namespace apollo {
namespace perception {
namespace onboard {

bool QueryStaticTransform(TransformWrapper* wrapper, const std::string& target,
                          const std::string& source,
                          Eigen::Matrix4d* transform) {
  if (wrapper == nullptr || transform == nullptr) {
    AERROR << "Static sensor transform requires wrapper and output.";
    return false;
  }
  const auto* sensors = common::SensorManager::Instance();
  const auto target_frame = sensors->GetFrameId(target);
  const auto source_frame = sensors->GetFrameId(source);
  Eigen::Affine3d affine;
  if (!wrapper->QueryStaticTrans(target_frame.empty() ? target : target_frame,
                                 source_frame.empty() ? source : source_frame,
                                 &affine)) {
    return false;
  }
  *transform = affine.matrix();
  return true;
}

bool SetCameraHeight(TransformWrapper* wrapper, const std::string& lidar,
                     const std::string& camera, float* height) {
  const auto* sensors = common::SensorManager::Instance();
  apollo::common::vehicle_calibration::LidarHeight reference;
  const auto frame = sensors->GetFrameId(camera);
  const auto* selected = sensors->calibration().FindSensor(camera);
  if (wrapper == nullptr || height == nullptr || selected == nullptr ||
      selected->calibration.sensor_type() !=
          apollo::common::vehicle_calibration::CAMERA ||
      !apollo::common::vehicle_calibration::LoadLidarHeight(
          sensors->calibration(), lidar, &reference)) {
    AERROR << "Camera height requires a selected LiDAR, camera and output.";
    return false;
  }
  Eigen::Affine3d ground_from_camera;
  if (!wrapper->QueryGroundTrans(reference.frame_id, reference.height, frame,
                                 &ground_from_camera)) {
    return false;
  }
  const double value = ground_from_camera.translation().z();
  if (!std::isfinite(value) || value <= 0.0 ||
      value > std::numeric_limits<float>::max()) {
    AERROR << "Invalid selected camera ground height: " << camera;
    return false;
  }
  *height = static_cast<float>(value);
  return true;
}

bool GetCommonCameraImageSize(const common::SensorManager& sensors,
                              const std::vector<std::string>& cameras,
                              int* width, int* height) {
  if (cameras.empty() || width == nullptr || height == nullptr) {
    AERROR << "Camera image size requires selected cameras and outputs.";
    return false;
  }
  int selected_width = 0;
  int selected_height = 0;
  for (const auto& camera : cameras) {
    const auto model = sensors.GetUndistortCameraModel(camera);
    if (model == nullptr || model->get_width() <= 0 ||
        model->get_height() <= 0 ||
        model->get_width() > std::numeric_limits<int>::max() ||
        model->get_height() > std::numeric_limits<int>::max()) {
      AERROR << "Missing or invalid calibrated image dimensions: " << camera;
      return false;
    }
    const int camera_width = static_cast<int>(model->get_width());
    const int camera_height = static_cast<int>(model->get_height());
    if (selected_width != 0 &&
        (camera_width != selected_width || camera_height != selected_height)) {
      AERROR << "This component requires equal camera calibration resolutions: "
             << camera;
      return false;
    }
    selected_width = camera_width;
    selected_height = camera_height;
  }
  *width = selected_width;
  *height = selected_height;
  return true;
}

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
