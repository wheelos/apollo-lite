/******************************************************************************
 * Copyright 2026 WheelOS All Rights Reserved.
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
#include "modules/transform/static_transform_loader.h"

#include <cmath>
#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "yaml-cpp/yaml.h"

namespace apollo {
namespace transform {
namespace {

bool ParseTransformYaml(const std::string& file_path,
                        const std::string& parent_frame_id,
                        const std::string& child_frame_id,
                        TransformStamped* transform) {
  if (transform == nullptr || !cyber::common::PathExists(file_path)) {
    AERROR << "Static transform file is missing or output is null: "
           << file_path;
    return false;
  }

  try {
    const YAML::Node yaml = YAML::LoadFile(file_path);
    transform->mutable_header()->set_frame_id(
        yaml["header"]["frame_id"].as<std::string>());
    transform->set_child_frame_id(
        yaml["child_frame_id"].as<std::string>());
    auto* translation = transform->mutable_transform()->mutable_translation();
    translation->set_x(yaml["transform"]["translation"]["x"].as<double>());
    translation->set_y(yaml["transform"]["translation"]["y"].as<double>());
    translation->set_z(yaml["transform"]["translation"]["z"].as<double>());
    auto* rotation = transform->mutable_transform()->mutable_rotation();
    rotation->set_qx(yaml["transform"]["rotation"]["x"].as<double>());
    rotation->set_qy(yaml["transform"]["rotation"]["y"].as<double>());
    rotation->set_qz(yaml["transform"]["rotation"]["z"].as<double>());
    rotation->set_qw(yaml["transform"]["rotation"]["w"].as<double>());

    const double quaternion_norm =
        rotation->qx() * rotation->qx() + rotation->qy() * rotation->qy() +
        rotation->qz() * rotation->qz() + rotation->qw() * rotation->qw();
    if (!std::isfinite(translation->x()) ||
        !std::isfinite(translation->y()) ||
        !std::isfinite(translation->z()) ||
        !std::isfinite(rotation->qx()) || !std::isfinite(rotation->qy()) ||
        !std::isfinite(rotation->qz()) || !std::isfinite(rotation->qw()) ||
        std::abs(quaternion_norm - 1.0) > 1e-3) {
      AERROR << "Static transform has invalid numeric values: " << file_path;
      return false;
    }
  } catch (const YAML::Exception& error) {
    AERROR << "Failed to parse static transform " << file_path << ": "
           << error.what();
    return false;
  }

  if (transform->header().frame_id() != parent_frame_id ||
      transform->child_frame_id() != child_frame_id) {
    AERROR << "Static transform frame pair does not match manifest: "
           << file_path << " expected " << parent_frame_id << " -> "
           << child_frame_id << " but found "
           << transform->header().frame_id() << " -> "
           << transform->child_frame_id();
    return false;
  }
  return true;
}

}  // namespace

bool LoadStaticTransforms(
    const common::vehicle_calibration::Registry& registry,
    TransformStampeds* transforms) {
  if (transforms == nullptr || registry.sensors().empty()) {
    AERROR << "Static transform loading requires a loaded registry and output.";
    return false;
  }
  transforms->Clear();
  for (const auto& sensor : registry.sensors()) {
    const auto& calibration = sensor.calibration;
    TransformStamped* transform = transforms->add_transforms();
    if (!ParseTransformYaml(sensor.resolved_extrinsic_file,
                            calibration.parent_frame_id(),
                            calibration.frame_id(), transform)) {
      return false;
    }
    if (calibration.sensor_type() ==
        common::vehicle_calibration::CAMERA) {
      transform = transforms->add_transforms();
      if (!ParseTransformYaml(sensor.resolved_optical_extrinsic_file,
                              calibration.frame_id(),
                              calibration.optical_frame_id(), transform)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace transform
}  // namespace apollo
