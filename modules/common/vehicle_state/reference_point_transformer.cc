// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/common/vehicle_state/reference_point_transformer.h"

#include <cmath>

#include "modules/common/configs/vehicle_config_helper.h"

namespace apollo {
namespace common {
namespace {

bool IsSupportedReferencePoint(const ReferencePoint reference_point) {
  return reference_point == REAR_AXLE_CENTER ||
         reference_point == FRONT_AXLE_CENTER ||
         reference_point == CENTER_OF_MASS;
}

}  // namespace

ReferencePointTransformer::ReferencePointTransformer()
    : description_(VehicleConfigHelper::GetConfig()) {}

ReferencePointTransformer::ReferencePointTransformer(
    const VehicleConfig& vehicle_config)
    : description_(vehicle_config) {}

ReferencePointTransformer::ReferencePointTransformer(
    const VehicleDescription& description)
    : description_(description) {}

Status ReferencePointTransformer::TransformState(
    const VehicleState& source_state, const ReferencePoint target_point,
    VehicleState* target_state) const {
  if (target_state == nullptr) {
    return Status(ErrorCode::LOCALIZATION_ERROR, "target_state is null");
  }
  if (!IsSupportedReferencePoint(source_state.reference_point()) ||
      !IsSupportedReferencePoint(target_point)) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "unsupported vehicle state reference point");
  }

  *target_state = source_state;
  target_state->set_reference_point(target_point);

  if (source_state.reference_point() == target_point) {
    return Status::OK();
  }

  const double target_offset = description_.LongitudinalOffset(target_point);
  const double source_offset =
      description_.LongitudinalOffset(source_state.reference_point());
  const double delta_x = target_offset - source_offset;

  if (std::abs(delta_x) < 1e-9) {
    return Status::OK();
  }

  double translation_x = 0.0;
  double translation_y = 0.0;

  if (source_state.has_pose() && source_state.pose().has_orientation() &&
      source_state.pose().orientation().has_qw() &&
      source_state.pose().orientation().has_qx() &&
      source_state.pose().orientation().has_qy() &&
      source_state.pose().orientation().has_qz()) {
    const auto& orientation = source_state.pose().orientation();
    const double qw = orientation.qw();
    const double qx = orientation.qx();
    const double qy = orientation.qy();
    const double qz = orientation.qz();
    const double quaternion_norm =
        std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (std::isfinite(quaternion_norm) && quaternion_norm > 1.0e-12) {
      const double normalized_qw = qw / quaternion_norm;
      const double normalized_qx = qx / quaternion_norm;
      const double normalized_qy = qy / quaternion_norm;
      const double normalized_qz = qz / quaternion_norm;
      const double r00 = 1.0 - 2.0 * (normalized_qy * normalized_qy +
                                      normalized_qz * normalized_qz);
      const double r10 =
          2.0 * (normalized_qx * normalized_qy + normalized_qw * normalized_qz);
      translation_x = r00 * delta_x;
      translation_y = r10 * delta_x;
    } else {
      const double heading = source_state.heading();
      translation_x = std::cos(heading) * delta_x;
      translation_y = std::sin(heading) * delta_x;
    }
  } else {
    const double heading = source_state.heading();
    translation_x = std::cos(heading) * delta_x;
    translation_y = std::sin(heading) * delta_x;
  }

  const double new_x = source_state.x() + translation_x;
  const double new_y = source_state.y() + translation_y;
  target_state->set_x(new_x);
  target_state->set_y(new_y);
  if (target_state->has_pose()) {
    target_state->mutable_pose()->mutable_position()->set_x(new_x);
    target_state->mutable_pose()->mutable_position()->set_y(new_y);
  }

  double omega = source_state.angular_velocity();
  if (std::abs(omega) < 1e-6 && source_state.has_kappa() &&
      source_state.has_linear_velocity()) {
    omega = source_state.linear_velocity() * source_state.kappa();
  }
  target_state->set_angular_velocity(omega);

  const double v_lin = source_state.linear_velocity();
  target_state->set_linear_velocity(v_lin);
  target_state->set_lateral_velocity(source_state.lateral_velocity() +
                                     omega * delta_x);

  const double a_lin = source_state.linear_acceleration();
  target_state->set_linear_acceleration(a_lin - omega * omega * delta_x);

  const double target_v = target_state->linear_velocity();
  if (std::abs(target_v) > 1e-3) {
    target_state->set_kappa(omega / target_v);
  } else {
    target_state->set_kappa(source_state.kappa());
  }

  return Status::OK();
}

}  // namespace common
}  // namespace apollo
