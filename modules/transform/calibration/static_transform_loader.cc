// Copyright 2026 WheelOS. All Rights Reserved.
#include "modules/transform/calibration/static_transform_loader.h"

#include <cmath>
#include <string>
#include <utility>

#include "yaml-cpp/yaml.h"

#include "cyber/common/log.h"

namespace apollo {
namespace transform {

bool LoadStaticTransforms(const common::vehicle_calibration::Registry& registry,
                          TransformStampeds* transforms) {
  if (transforms == nullptr || registry.static_transforms().empty()) {
    AERROR << "Static transform loading requires a loaded registry and output.";
    return false;
  }
  transforms->Clear();
  TransformStampeds candidate;
  for (const auto& edge : registry.static_transforms()) {
    const auto& path = edge.resolved_extrinsic_file;
    auto* transform = candidate.add_transforms();
    try {
      const auto yaml = YAML::LoadFile(path);
      const auto parent = yaml["header"]["frame_id"].as<std::string>();
      const auto child = yaml["child_frame_id"].as<std::string>();
      if (parent != edge.calibration.parent_frame_id() ||
          child != edge.calibration.child_frame_id()) {
        AERROR << "Calibration frame mismatch in " << path << ": expected "
               << edge.calibration.parent_frame_id() << " -> "
               << edge.calibration.child_frame_id() << ", got " << parent
               << " -> " << child;
        return false;
      }
      transform->mutable_header()->set_frame_id(parent);
      transform->set_child_frame_id(child);
      auto* translation = transform->mutable_transform()->mutable_translation();
      translation->set_x(yaml["transform"]["translation"]["x"].as<double>());
      translation->set_y(yaml["transform"]["translation"]["y"].as<double>());
      translation->set_z(yaml["transform"]["translation"]["z"].as<double>());
      auto* rotation = transform->mutable_transform()->mutable_rotation();
      rotation->set_qx(yaml["transform"]["rotation"]["x"].as<double>());
      rotation->set_qy(yaml["transform"]["rotation"]["y"].as<double>());
      rotation->set_qz(yaml["transform"]["rotation"]["z"].as<double>());
      rotation->set_qw(yaml["transform"]["rotation"]["w"].as<double>());
      const double norm =
          rotation->qx() * rotation->qx() + rotation->qy() * rotation->qy() +
          rotation->qz() * rotation->qz() + rotation->qw() * rotation->qw();
      if (!std::isfinite(translation->x()) ||
          !std::isfinite(translation->y()) ||
          !std::isfinite(translation->z()) || !std::isfinite(norm) ||
          std::abs(norm - 1.0) > 1e-3) {
        AERROR << "Non-finite extrinsics or non-unit quaternion in " << path;
        return false;
      }
      const double scale = 1.0 / std::sqrt(norm);
      rotation->set_qx(rotation->qx() * scale);
      rotation->set_qy(rotation->qy() * scale);
      rotation->set_qz(rotation->qz() * scale);
      rotation->set_qw(rotation->qw() * scale);
    } catch (const YAML::Exception& error) {
      AERROR << "Invalid static calibration " << path << ": " << error.what();
      return false;
    }
  }
  *transforms = std::move(candidate);
  return true;
}

}  // namespace transform
}  // namespace apollo
