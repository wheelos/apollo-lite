/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/camera/tools/offline/transform_server.h"

#include <cmath>
#include <fstream>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/camera/common/util.h"
#include "modules/transform/static_transform_loader.h"
#include "tf2/exceptions.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace perception {
namespace camera {

bool TransformServer::Init(
    const std::map<std::string, std::string> &sensor_frame_ids,
    const std::string &lidar_frame_id) {
  if (sensor_frame_ids.empty() || lidar_frame_id.empty()) {
    AERROR << "Offline TF requires explicit sensor-to-frame mappings.";
    return false;
  }
  if (!calibration_registry_.LoadFromEnvironment()) {
    AERROR << "Failed to load selected vehicle calibration profile.";
    return false;
  }

  apollo::transform::TransformStampeds transforms;
  if (!apollo::transform::LoadStaticTransforms(calibration_registry_,
                                               &transforms)) {
    AERROR << "Failed to load selected calibration static transforms.";
    return false;
  }

  static_buffer_.reset(new tf2::BufferCore());
  for (const auto &stamped : transforms.transforms()) {
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = stamped.header().frame_id();
    transform.child_frame_id = stamped.child_frame_id();
    const auto &translation = stamped.transform().translation();
    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    const auto &rotation = stamped.transform().rotation();
    transform.transform.rotation.x = rotation.qx();
    transform.transform.rotation.y = rotation.qy();
    transform.transform.rotation.z = rotation.qz();
    transform.transform.rotation.w = rotation.qw();
    if (!static_buffer_->setTransform(transform, "vehicle calibration", true)) {
      AERROR << "Failed to insert static TF edge "
             << transform.header.frame_id << " -> "
             << transform.child_frame_id;
      return false;
    }
    vertices_.insert(transform.header.frame_id);
    vertices_.insert(transform.child_frame_id);
  }

  auto has_calibration_frame =
      [this](const std::string &frame_id,
             apollo::common::vehicle_calibration::SensorType sensor_type) {
    for (const auto &sensor : calibration_registry_.sensors()) {
      if (sensor.calibration.frame_id() == frame_id &&
          sensor.calibration.sensor_type() == sensor_type) {
        return true;
      }
    }
    return false;
  };
  sensor_frame_ids_ = sensor_frame_ids;
  for (const auto &entry : sensor_frame_ids_) {
    if (entry.first.empty() || entry.second.empty() ||
        !has_calibration_frame(entry.second,
                               apollo::common::vehicle_calibration::CAMERA)) {
      AERROR << "Explicit camera frame is absent from selected calibration: "
             << entry.first << " -> " << entry.second;
      return false;
    }
  }
  lidar_frame_id_ = lidar_frame_id;
  if (!has_calibration_frame(
          lidar_frame_id_, apollo::common::vehicle_calibration::LIDAR)) {
    AERROR << "Explicit lidar frame is absent from selected calibration: "
           << lidar_frame_id_;
    return false;
  }
  for (const auto &entry : sensor_frame_ids_) {
    if (vertices_.find(entry.second) == vertices_.end()) {
      AERROR << "Sensor metadata frame is absent from selected calibration TF: "
             << entry.first << " -> " << entry.second;
      return false;
    }
  }
  if (vertices_.find(lidar_frame_id_) == vertices_.end()) {
    AERROR << "Selected lidar frame is absent from calibration TF: "
           << lidar_frame_id_;
    return false;
  }

  const std::string &height_file =
      calibration_registry_.resolved_lidar_height_file();
  if (height_file.empty() || !cyber::common::PathExists(height_file)) {
    AERROR << "Selected calibration has no readable lidar height asset: "
           << height_file;
    return false;
  }
  try {
    const YAML::Node height = YAML::LoadFile(height_file);
    const std::string height_frame_id =
        height["frame_id"].as<std::string>();
    lidar_height_m_ = height["height_m"].as<double>();
    if (height_frame_id != lidar_frame_id_ ||
        !std::isfinite(lidar_height_m_)) {
      AERROR << "Invalid lidar height calibration in " << height_file
             << ": expected frame " << lidar_frame_id_ << ", found "
             << height_frame_id;
      return false;
    }
  } catch (const YAML::Exception &error) {
    AERROR << "Failed to parse lidar height calibration " << height_file
           << ": " << error.what();
    return false;
  }
  return true;
}

bool TransformServer::LoadFromFile(const std::string &tf_input,
                                   float frequency) {
  if (frequency <= 0) {
    AERROR << "Error frequency value:" << frequency;
    return false;
  }
  std::ifstream fin(tf_input);
  Transform tf;
  int64_t ts;
  while (fin >> ts) {
    tf.timestamp = static_cast<double>(ts) * 1e-9;
    fin >> tf.tx;
    fin >> tf.ty;
    fin >> tf.tz;
    fin >> tf.qx;
    fin >> tf.qy;
    fin >> tf.qz;
    fin >> tf.qw;
    tf_.push_back(tf);
  }
  fin.close();
  error_limit_ = 1 / frequency / 2.0f;
  AINFO << "Load tf successfully. count: " << tf_.size()
        << " error limit:" << error_limit_;
  return true;
}

bool TransformServer::QueryPos(double timestamp, Eigen::Affine3d *pose) {
  for (auto &&tf : tf_) {
    if (Equal(timestamp, tf.timestamp, error_limit_)) {
      Eigen::Quaterniond rotation(tf.qw, tf.qx, tf.qy, tf.qz);
      pose->linear() = rotation.matrix();
      pose->translation() << tf.tx, tf.ty, tf.tz;
      AINFO << "Get Pose:\n" << pose->matrix();
      return true;
    }
  }
  return false;
}

bool TransformServer::QueryTransform(const std::string &child_frame_id,
                                     const std::string &frame_id,
                                     Eigen::Affine3d *transform) {
  if (transform == nullptr || static_buffer_ == nullptr) {
    AERROR << "Offline TF query has no output or initialized buffer.";
    return false;
  }
  const std::string source_frame_id = FrameId(child_frame_id);
  if (frame_id == "ground") {
    const std::string base_frame_id = "base_link";
    geometry_msgs::TransformStamped base_to_camera;
    geometry_msgs::TransformStamped base_to_lidar;
    try {
      base_to_camera = static_buffer_->lookupTransform(
          base_frame_id, source_frame_id, tf2::Time(0));
      base_to_lidar = static_buffer_->lookupTransform(
          base_frame_id, lidar_frame_id_, tf2::Time(0));
    } catch (const tf2::TransformException &error) {
      AERROR << "Failed to query camera/lidar transform for ground height: "
             << error.what();
      return false;
    }
    Eigen::Quaterniond rotation(
        base_to_camera.transform.rotation.w,
        base_to_camera.transform.rotation.x,
        base_to_camera.transform.rotation.y,
        base_to_camera.transform.rotation.z);
    transform->linear() = rotation.toRotationMatrix();
    transform->translation()
        << base_to_camera.transform.translation.x,
        base_to_camera.transform.translation.y,
        base_to_camera.transform.translation.z -
            base_to_lidar.transform.translation.z + lidar_height_m_;
    return true;
  }

  const std::string target_frame_id = FrameId(frame_id);
  try {
    const auto result = static_buffer_->lookupTransform(
        target_frame_id, source_frame_id, tf2::Time(0));
    Eigen::Quaterniond rotation(result.transform.rotation.w,
                                result.transform.rotation.x,
                                result.transform.rotation.y,
                                result.transform.rotation.z);
    *transform = Eigen::Translation3d(result.transform.translation.x,
                                       result.transform.translation.y,
                                       result.transform.translation.z) *
                 rotation;
  } catch (const tf2::TransformException &error) {
    AERROR << "Failed to query static TF from " << source_frame_id << " to "
           << target_frame_id << ": " << error.what();
    return false;
  }
  return true;
}

std::string TransformServer::FrameId(const std::string &sensor_name) const {
  const auto entry = sensor_frame_ids_.find(sensor_name);
  return entry == sensor_frame_ids_.end() ? sensor_name : entry->second;
}

void TransformServer::print() {
  for (const auto &vertex : vertices_) {
    AINFO << "TF frame: " << vertex;
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
