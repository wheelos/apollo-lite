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
namespace {}  // namespace

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

  const VehicleState source_snapshot = source_state;
  if (!std::isfinite(source_snapshot.x()) ||
      !std::isfinite(source_snapshot.y()) ||
      !std::isfinite(source_snapshot.heading())) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "vehicle state contains non-finite 2D pose data");
  }
  if (source_snapshot.has_pose()) {
    if (!source_snapshot.pose().has_position() ||
        !std::isfinite(source_snapshot.pose().position().x()) ||
        !std::isfinite(source_snapshot.pose().position().y())) {
      return Status(ErrorCode::LOCALIZATION_ERROR,
                    "vehicle pose must contain finite 2D position");
    }
  }
  if (!std::isfinite(source_snapshot.linear_velocity()) ||
      !std::isfinite(source_snapshot.lateral_velocity()) ||
      !std::isfinite(source_snapshot.linear_acceleration()) ||
      (source_snapshot.has_angular_velocity() &&
       !std::isfinite(source_snapshot.angular_velocity())) ||
      (source_snapshot.has_kappa() &&
       !std::isfinite(source_snapshot.kappa()))) {
    return Status(ErrorCode::LOCALIZATION_ERROR,
                  "vehicle state contains non-finite kinematic data");
  }
  *target_state = source_snapshot;
  target_state->set_reference_point(target_point);

  if (source_snapshot.reference_point() == target_point) {
    return Status::OK();
  }

  double target_offset = 0.0;
  double source_offset = 0.0;
  auto status = description_.LongitudinalOffset(target_point, &target_offset);
  if (!status.ok()) {
    return status;
  }
  status = description_.LongitudinalOffset(source_snapshot.reference_point(),
                                           &source_offset);
  if (!status.ok()) {
    return status;
  }
  const double delta_x = target_offset - source_offset;

  if (std::abs(delta_x) < 1e-9) {
    return Status::OK();
  }

  double translation_x = 0.0;
  double translation_y = 0.0;

  const double heading = source_snapshot.heading();
  translation_x = std::cos(heading) * delta_x;
  translation_y = std::sin(heading) * delta_x;

  const double new_x = source_snapshot.x() + translation_x;
  const double new_y = source_snapshot.y() + translation_y;
  target_state->set_x(new_x);
  target_state->set_y(new_y);
  if (target_state->has_pose()) {
    target_state->mutable_pose()->mutable_position()->set_x(new_x);
    target_state->mutable_pose()->mutable_position()->set_y(new_y);
  }

  double omega = 0.0;
  if (source_snapshot.has_angular_velocity()) {
    omega = source_snapshot.angular_velocity();
  } else if (source_snapshot.has_kappa() &&
             source_snapshot.has_linear_velocity()) {
    omega = source_snapshot.linear_velocity() * source_snapshot.kappa();
  }
  target_state->set_angular_velocity(omega);

  const double v_lin = source_snapshot.linear_velocity();
  target_state->set_linear_velocity(v_lin);
  target_state->set_lateral_velocity(source_snapshot.lateral_velocity() +
                                     omega * delta_x);

  const double a_lin = source_snapshot.linear_acceleration();
  target_state->set_linear_acceleration(a_lin - omega * omega * delta_x);

  const double target_v = target_state->linear_velocity();
  if (std::abs(target_v) > 1e-3) {
    target_state->set_kappa(omega / target_v);
  } else {
    target_state->set_kappa(source_snapshot.kappa());
  }

  return Status::OK();
}

}  // namespace common
}  // namespace apollo
