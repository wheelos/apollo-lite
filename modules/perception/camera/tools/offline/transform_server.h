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
#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "tf2/buffer_core.h"

#include "modules/common/vehicle_calibration/registry.h"

namespace apollo {
namespace perception {
namespace camera {

struct Transform {
  double timestamp;
  double qw;
  double qx;
  double qy;
  double qz;
  double tx;
  double ty;
  double tz;
};

class TransformServer {
 public:
  TransformServer() {}
  ~TransformServer() {}

  inline const std::set<std::string> &vertices() const { return vertices_; }

  bool Init(const std::map<std::string, std::string> &sensor_frame_ids,
            const std::string &lidar_frame_id);

  bool QueryTransform(const std::string &child_frame_id,
                      const std::string &frame_id, Eigen::Affine3d *transform);

  std::string FrameId(const std::string &sensor_name) const;
  const std::string &LidarFrameId() const { return lidar_frame_id_; }
  const std::string &CalibrationBundlePath() const {
    return calibration_registry_.bundle_path();
  }

  void print();

  bool LoadFromFile(const std::string &tf_input, float frequency = 200.0f);

  bool QueryPos(double timestamp, Eigen::Affine3d *pose);

 private:
  std::vector<Transform> tf_;

  double error_limit_ = 1.0;
  std::set<std::string> vertices_;
  std::map<std::string, std::string> sensor_frame_ids_;
  std::string lidar_frame_id_;
  double lidar_height_m_ = 0.0;
  apollo::common::vehicle_calibration::Registry calibration_registry_;
  std::unique_ptr<tf2::BufferCore> static_buffer_;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
