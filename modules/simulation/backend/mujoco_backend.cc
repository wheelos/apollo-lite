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

//  Created Date: 2026-09-10
//  Author: daohu527

#include "modules/simulation/backend/mujoco_backend.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#if defined(USE_MUJOCO)
#include <mujoco/mujoco.h>
#endif

#include "cyber/common/log.h"
#include "modules/simulation/common/simulation_gflags.h"

namespace apollo {
namespace simulation {
namespace {

constexpr double kGeometryTolerance = 1e-6;

bool IsCompatible(double configured, double model_value) {
  return std::isfinite(configured) && configured > 0.0 &&
         std::isfinite(model_value) && model_value > 0.0 &&
         std::abs(configured - model_value) <= kGeometryTolerance;
}

#if defined(USE_MUJOCO)
std::array<double, 3> RotateBodyToWorld(const std::array<double, 4>& q,
                                        const std::array<double, 3>& v) {
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return {
      (1.0 - 2.0 * (y * y + z * z)) * v[0] +
          2.0 * (x * y - w * z) * v[1] +
          2.0 * (x * z + w * y) * v[2],
      2.0 * (x * y + w * z) * v[0] +
          (1.0 - 2.0 * (x * x + z * z)) * v[1] +
          2.0 * (y * z - w * x) * v[2],
      2.0 * (x * z - w * y) * v[0] +
          2.0 * (y * z + w * x) * v[1] +
          (1.0 - 2.0 * (x * x + y * y)) * v[2]};
}

std::array<double, 3> RotateWorldToBody(const std::array<double, 4>& q,
                                        const std::array<double, 3>& v) {
  const std::array<double, 4> conjugate{q[0], -q[1], -q[2], -q[3]};
  return RotateBodyToWorld(conjugate, v);
}
#endif

}  // namespace

MujocoBackend::MujocoBackend() { Reset(0.0, 0.0, 0.0); }

MujocoBackend::~MujocoBackend() {
#if defined(USE_MUJOCO)
  if (mj_data_) {
    mj_deleteData(static_cast<mjData*>(mj_data_));
    mj_data_ = nullptr;
  }
  if (mj_model_) {
    mj_deleteModel(static_cast<mjModel*>(mj_model_));
    mj_model_ = nullptr;
  }
#endif
}

bool MujocoBackend::Init(const std::string& model_path) {
  model_path_ = model_path;
#if defined(USE_MUJOCO)
  if (mj_data_) {
    mj_deleteData(static_cast<mjData*>(mj_data_));
    mj_data_ = nullptr;
  }
  if (mj_model_) {
    mj_deleteModel(static_cast<mjModel*>(mj_model_));
    mj_model_ = nullptr;
  }

  char error[1024] = {0};
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    AERROR << "Failed to load MuJoCo model from " << model_path << ": "
           << error;
    return false;
  }
  mj_model_ = m;
  mj_data_ = mj_makeData(m);
  if (!mj_data_) {
    AERROR << "Failed to allocate MuJoCo data for " << model_path;
    mj_deleteModel(m);
    mj_model_ = nullptr;
    return false;
  }

  // Look up actuator / joint IDs
  steer_fl_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "steer_fl");
  steer_fr_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "steer_fr");
  steer_rl_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "steer_rl");
  steer_rr_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "steer_rr");
  steer_fl_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "steer_fl_joint");
  steer_fr_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "steer_fr_joint");
  steer_rl_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "steer_rl_joint");
  steer_rr_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "steer_rr_joint");
  wheel_fl_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "wheel_fl_joint");
  wheel_fr_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "wheel_fr_joint");
  wheel_rl_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "wheel_rl_joint");
  wheel_rr_joint_id_ = mj_name2id(m, mjOBJ_JOINT, "wheel_rr_joint");
  drive_fl_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "drive_fl");
  drive_fr_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "drive_fr");
  drive_rl_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "drive_rl");
  drive_rr_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "drive_rr");
  vehicle_body_id_ = mj_name2id(m, mjOBJ_BODY, "ego_vehicle");
  freejoint_id_ = mj_name2id(m, mjOBJ_JOINT, "vehicle_freejoint");
  if (freejoint_id_ >= 0 && m->jnt_type[freejoint_id_] != mjJNT_FREE) {
    freejoint_id_ = -1;
  }

  if (freejoint_id_ < 0) {
    // Search for first freejoint
    for (int j = 0; j < m->njnt; ++j) {
      if (m->jnt_type[j] == mjJNT_FREE) {
        freejoint_id_ = j;
        break;
      }
    }
  }

  auto actuator_binds_joint = [m](int actuator_id, int joint_id) {
    return actuator_id >= 0 && joint_id >= 0 &&
           m->actuator_trntype[actuator_id] == mjTRN_JOINT &&
           m->actuator_trnid[2 * actuator_id] == joint_id;
  };
  if (vehicle_body_id_ < 0 || freejoint_id_ < 0 || steer_fl_id_ < 0 ||
      steer_fr_id_ < 0 || drive_fl_id_ < 0 || drive_fr_id_ < 0 ||
      drive_rl_id_ < 0 || drive_rr_id_ < 0 || steer_fl_joint_id_ < 0 ||
      steer_fr_joint_id_ < 0 || wheel_fl_joint_id_ < 0 ||
      wheel_fr_joint_id_ < 0 || wheel_rl_joint_id_ < 0 ||
      wheel_rr_joint_id_ < 0 ||
      !actuator_binds_joint(steer_fl_id_, steer_fl_joint_id_) ||
      !actuator_binds_joint(steer_fr_id_, steer_fr_joint_id_) ||
      !actuator_binds_joint(drive_fl_id_, wheel_fl_joint_id_) ||
      !actuator_binds_joint(drive_fr_id_, wheel_fr_joint_id_) ||
      !actuator_binds_joint(drive_rl_id_, wheel_rl_joint_id_) ||
      !actuator_binds_joint(drive_rr_id_, wheel_rr_joint_id_)) {
    AERROR << "MuJoCo model is missing ego_vehicle, vehicle_freejoint, or "
              "required steering/drive actuators.";
    mj_deleteData(static_cast<mjData*>(mj_data_));
    mj_deleteModel(static_cast<mjModel*>(mj_model_));
    mj_data_ = nullptr;
    mj_model_ = nullptr;
    return false;
  }

  mj_forward(m, static_cast<mjData*>(mj_data_));
  const mjData* d = static_cast<const mjData*>(mj_data_);
  const auto anchor = [d](int joint_id, int axis) {
    return d->xanchor[3 * joint_id + axis];
  };
  const double front_x =
      0.5 * (anchor(steer_fl_joint_id_, 0) + anchor(steer_fr_joint_id_, 0));
  const double front_y =
      0.5 * (anchor(steer_fl_joint_id_, 1) + anchor(steer_fr_joint_id_, 1));
  const double rear_x =
      0.5 * (anchor(wheel_rl_joint_id_, 0) + anchor(wheel_rr_joint_id_, 0));
  const double rear_y =
      0.5 * (anchor(wheel_rl_joint_id_, 1) + anchor(wheel_rr_joint_id_, 1));
  model_wheelbase_m_ = std::hypot(front_x - rear_x, front_y - rear_y);
  model_track_width_m_ =
      std::hypot(anchor(steer_fl_joint_id_, 0) - anchor(steer_fr_joint_id_, 0),
                 anchor(steer_fl_joint_id_, 1) -
                     anchor(steer_fr_joint_id_, 1));
  const int wheel_fl_geom_id =
      mj_name2id(m, mjOBJ_GEOM, "wheel_fl_geom");
  model_wheel_radius_m_ =
      wheel_fl_geom_id >= 0 ? m->geom_size[3 * wheel_fl_geom_id] : 0.0;
  model_max_steer_angle_rad_ =
      std::min(std::abs(m->actuator_ctrlrange[2 * steer_fl_id_]),
               std::abs(m->actuator_ctrlrange[2 * steer_fr_id_]));

  init_z_ = m->body_pos[3 * vehicle_body_id_ + 2];
  if (init_z_ <= 0.0) init_z_ = 0.35;

  AINFO << "MuJoCo model loaded successfully from " << model_path;
  Reset(0.0, 0.0, 0.0);
  return true;
#else
  AWARN << "MuJoCo backend compiled without USE_MUJOCO flag. "
        << "Using placeholder mode for " << model_path;
  Reset(0.0, 0.0, 0.0);
  return true;
#endif
}

void MujocoBackend::Reset(double x, double y, double yaw) {
  sim_time_sec_ = 0.0;
  cached_state_ = VehicleState{};
  cached_state_.x = x;
  cached_state_.y = y;
  cached_state_.yaw = yaw;
  cached_state_.qz = std::sin(yaw * 0.5);
  cached_state_.qw = std::cos(yaw * 0.5);
  previous_velocity_mps_ = 0.0;
  previous_lateral_velocity_mps_ = 0.0;
  previous_state_time_sec_ = 0.0;
  previous_yaw_ = 0.0;
  previous_world_velocity_mps_ = {0.0, 0.0, 0.0};
  has_previous_state_ = false;
  debug_step_count_ = 0;
  wheel_velocity_sign_ = {0.0, 0.0, 0.0, 0.0};

#if defined(USE_MUJOCO)
  if (mj_model_ && mj_data_) {
    mjModel* m = static_cast<mjModel*>(mj_model_);
    mjData* d = static_cast<mjData*>(mj_data_);
    mj_resetData(m, d);

    int qpos_adr = (freejoint_id_ >= 0) ? m->jnt_qposadr[freejoint_id_] : 0;
    const double half_wheelbase = 0.5 * model_wheelbase_m_;
    d->qpos[qpos_adr + 0] = x + half_wheelbase * std::cos(yaw);
    d->qpos[qpos_adr + 1] = y + half_wheelbase * std::sin(yaw);
    d->qpos[qpos_adr + 2] = init_z_;
    // Quaternion: qw, qx, qy, qz in MuJoCo
    d->qpos[qpos_adr + 3] = std::cos(yaw * 0.5);
    d->qpos[qpos_adr + 4] = 0.0;
    d->qpos[qpos_adr + 5] = 0.0;
    d->qpos[qpos_adr + 6] = std::sin(yaw * 0.5);
    mj_forward(m, d);
  }

#endif
}

bool MujocoBackend::SetVehicleGeometry(double wheelbase_m,
                                       double track_width_m,
                                       double wheel_radius_m) {
#if defined(USE_MUJOCO)
  if (!IsCompatible(wheelbase_m, model_wheelbase_m_) ||
      !IsCompatible(track_width_m, model_track_width_m_) ||
      !IsCompatible(wheel_radius_m, model_wheel_radius_m_)) {
    AERROR << "MuJoCo model geometry does not match vehicle configuration: "
           << "configured=(" << wheelbase_m << ", " << track_width_m << ", "
           << wheel_radius_m << "), model=(" << model_wheelbase_m_ << ", "
           << model_track_width_m_ << ", " << model_wheel_radius_m_ << ")";
    return false;
  }
#endif
  return true;
}

bool MujocoBackend::SetMaxSteerAngle(double max_steer_angle_rad) {
#if defined(USE_MUJOCO)
  if (!IsCompatible(max_steer_angle_rad, model_max_steer_angle_rad_)) {
    AERROR << "MuJoCo steering limit does not match vehicle configuration: "
           << "configured=" << max_steer_angle_rad
           << ", model=" << model_max_steer_angle_rad_;
    return false;
  }
#endif
  max_steer_angle_rad_ = max_steer_angle_rad;
  return true;
}

bool MujocoBackend::SetMaxRearSteerAngle(double max_rear_steer_angle_rad) {
#if defined(USE_MUJOCO)
  if (!std::isfinite(max_rear_steer_angle_rad) ||
      max_rear_steer_angle_rad < 0.0) {
    return false;
  }
  if (max_rear_steer_angle_rad > 0.0 &&
      (steer_rl_id_ < 0 || steer_rr_id_ < 0 ||
       steer_rl_joint_id_ < 0 || steer_rr_joint_id_ < 0)) {
    AERROR << "MuJoCo model lacks rear steering actuators or joints.";
    return false;
  }
#else
  if (!std::isfinite(max_rear_steer_angle_rad) ||
      max_rear_steer_angle_rad < 0.0) {
    return false;
  }
#endif
  return true;
}

bool MujocoBackend::ApplyActuation(const VehicleActuation& actuation) {
  current_actuation_ = actuation;
#if defined(USE_MUJOCO)
  if (!mj_model_ || !mj_data_) return false;
  mjModel* m = static_cast<mjModel*>(mj_model_);
  mjData* d = static_cast<mjData*>(mj_data_);

  // Apply steering positions
  if (steer_fl_id_ >= 0) d->ctrl[steer_fl_id_] = actuation.wheel_steer_rad[0];
  if (steer_fr_id_ >= 0) d->ctrl[steer_fr_id_] = actuation.wheel_steer_rad[1];
  if (steer_rl_id_ >= 0) d->ctrl[steer_rl_id_] = actuation.wheel_steer_rad[2];
  if (steer_rr_id_ >= 0) d->ctrl[steer_rr_id_] = actuation.wheel_steer_rad[3];

  // Apply drive / brake torques
  const auto braking_torque = [m, d, this](int joint_id, int wheel_index,
                                           double brake_torque) {
    if (joint_id < 0 || brake_torque <= 0.0) return 0.0;
    const double wheel_velocity = d->qvel[m->jnt_dofadr[joint_id]];
    constexpr double kBrakeVelocityScaleRadps = 0.05;
    constexpr double kWheelVelocityDeadbandRadps = 1e-3;
    if (std::abs(wheel_velocity) > kWheelVelocityDeadbandRadps) {
      wheel_velocity_sign_[wheel_index] = wheel_velocity > 0.0 ? 1.0 : -1.0;
    }
    const double sign =
        std::abs(wheel_velocity) > kWheelVelocityDeadbandRadps
            ? std::tanh(wheel_velocity / kBrakeVelocityScaleRadps)
            : wheel_velocity_sign_[wheel_index];
    return -brake_torque * sign;
  };
  if (drive_fl_id_ >= 0) {
    d->ctrl[drive_fl_id_] =
        actuation.drive_torque_nm[0] +
        braking_torque(wheel_fl_joint_id_, 0, actuation.brake_torque_nm[0]);
  }
  if (drive_fr_id_ >= 0) {
    d->ctrl[drive_fr_id_] =
        actuation.drive_torque_nm[1] +
        braking_torque(wheel_fr_joint_id_, 1, actuation.brake_torque_nm[1]);
  }
  if (drive_rl_id_ >= 0) {
    d->ctrl[drive_rl_id_] =
        actuation.drive_torque_nm[2] +
        braking_torque(wheel_rl_joint_id_, 2, actuation.brake_torque_nm[2]);
  }
  if (drive_rr_id_ >= 0) {
    d->ctrl[drive_rr_id_] =
        actuation.drive_torque_nm[3] +
        braking_torque(wheel_rr_joint_id_, 3, actuation.brake_torque_nm[3]);
  }
#endif
  return true;
}

bool MujocoBackend::Step(double dt_sec) {
  if (dt_sec <= 0.0) return false;
#if defined(USE_MUJOCO)
  if (!mj_model_ || !mj_data_) return false;
  mjModel* m = static_cast<mjModel*>(mj_model_);
  mjData* d = static_cast<mjData*>(mj_data_);

  // SimulationEngine performs configured physics substepping. Keep MuJoCo's
  // integration timestep aligned with the actual step being requested.
  m->opt.timestep = dt_sec;
  mj_step(m, d);

  sim_time_sec_ += dt_sec;
  ++debug_step_count_;
  if (FLAGS_sim_debug_log && FLAGS_sim_debug_log_interval > 0 &&
      debug_step_count_ % FLAGS_sim_debug_log_interval == 0) {
    const int qpos_adr =
        (freejoint_id_ >= 0) ? m->jnt_qposadr[freejoint_id_] : 0;
    const auto wheel_velocity = [m, d](int joint_id) {
      return d->qvel[m->jnt_dofadr[joint_id]];
    };
    AINFO << "MuJoCo physics: t=" << sim_time_sec_ << " qpos=("
          << d->qpos[qpos_adr] << ", " << d->qpos[qpos_adr + 1] << ", "
          << d->qpos[qpos_adr + 2] << ") qvel=("
          << d->qvel[m->jnt_dofadr[freejoint_id_]] << ", "
          << d->qvel[m->jnt_dofadr[freejoint_id_] + 1] << ", "
          << d->qvel[m->jnt_dofadr[freejoint_id_] + 2] << ") wheel_vel=("
          << wheel_velocity(wheel_fl_joint_id_) << ", "
          << wheel_velocity(wheel_fr_joint_id_) << ", "
          << wheel_velocity(wheel_rl_joint_id_) << ", "
          << wheel_velocity(wheel_rr_joint_id_) << ") contacts=" << d->ncon;
  }
  return true;
#else
  sim_time_sec_ += dt_sec;
  return true;
#endif
}

bool MujocoBackend::GetVehicleState(VehicleState* state) const {
  if (!state) return false;
#if defined(USE_MUJOCO)
  if (!mj_model_ || !mj_data_) return false;
  mjModel* m = static_cast<mjModel*>(mj_model_);
  mjData* d = static_cast<mjData*>(mj_data_);

  state->timestamp_sec = sim_time_sec_;
  int body_id = vehicle_body_id_;

  const double rear_axle_x =
      0.5 * (d->xanchor[3 * wheel_rl_joint_id_ + 0] +
             d->xanchor[3 * wheel_rr_joint_id_ + 0]);
  const double rear_axle_y =
      0.5 * (d->xanchor[3 * wheel_rl_joint_id_ + 1] +
             d->xanchor[3 * wheel_rr_joint_id_ + 1]);
  const double rear_axle_z =
      0.5 * (d->xanchor[3 * wheel_rl_joint_id_ + 2] +
             d->xanchor[3 * wheel_rr_joint_id_ + 2]);
  state->x = rear_axle_x;
  state->y = rear_axle_y;
  state->z = rear_axle_z;

  // MuJoCo quaternion: w, x, y, z -> Apollo: x, y, z, w
  state->qw = d->xquat[4 * body_id + 0];
  state->qx = d->xquat[4 * body_id + 1];
  state->qy = d->xquat[4 * body_id + 2];
  state->qz = d->xquat[4 * body_id + 3];

  // Heading calculation from quaternion
  state->roll =
      std::atan2(2.0 * (state->qw * state->qy - state->qx * state->qz),
                 2.0 * (state->qw * state->qw + state->qz * state->qz) -
                     1.0);
  state->pitch = std::asin(std::clamp(
      2.0 * (state->qw * state->qx + state->qy * state->qz), -1.0, 1.0));
  state->yaw =
      std::atan2(2.0 * (state->qw * state->qz - state->qx * state->qy),
                 2.0 * (state->qw * state->qw + state->qy * state->qy) -
                     1.0);

  const std::array<double, 4> quaternion{
      state->qw, state->qx, state->qy, state->qz};
  // Velocities from freejoint DOF or spatial vector
  if (freejoint_id_ >= 0) {
    int dof_adr = m->jnt_dofadr[freejoint_id_];
    const std::array<double, 3> body_velocity{
        d->qvel[dof_adr + 0], d->qvel[dof_adr + 1], d->qvel[dof_adr + 2]};
    const std::array<double, 3> body_angular_velocity{
        d->qvel[dof_adr + 3], d->qvel[dof_adr + 4], d->qvel[dof_adr + 5]};
    const double body_x = d->xpos[3 * body_id + 0];
    const double body_y = d->xpos[3 * body_id + 1];
    const double body_z = d->xpos[3 * body_id + 2];
    const double rear_offset_x = rear_axle_x - body_x;
    const double rear_offset_y = rear_axle_y - body_y;
    const double rear_offset_z = rear_axle_z - body_z;
    const auto world_angular_velocity =
        RotateBodyToWorld(quaternion, body_angular_velocity);
    const std::array<double, 3> rear_offset{
        rear_offset_x, rear_offset_y, rear_offset_z};
    const std::array<double, 3> rear_velocity_world{
        body_velocity[0] + world_angular_velocity[1] * rear_offset[2] -
            world_angular_velocity[2] * rear_offset[1],
        body_velocity[1] + world_angular_velocity[2] * rear_offset[0] -
            world_angular_velocity[0] * rear_offset[2],
        body_velocity[2] + world_angular_velocity[0] * rear_offset[1] -
            world_angular_velocity[1] * rear_offset[0]};
    const auto rear_velocity_body =
        RotateWorldToBody(quaternion, rear_velocity_world);
    state->linear_velocity_mps = rear_velocity_body[0];
    state->lateral_velocity_mps = rear_velocity_body[1];
    state->angular_velocity_body_radps = body_angular_velocity;
    state->angular_velocity_world_radps = world_angular_velocity;
    state->linear_velocity_world_mps = rear_velocity_world;
    state->angular_velocity_yaw_radps = body_angular_velocity[2];
  } else {
    state->linear_velocity_mps =
        std::sqrt(d->cvel[6 * body_id + 3] * d->cvel[6 * body_id + 3] +
                  d->cvel[6 * body_id + 4] * d->cvel[6 * body_id + 4]);
    state->angular_velocity_yaw_radps = d->cvel[6 * body_id + 2];
  }
  const auto wheel_speed = [m, d, this](int joint_id) {
    if (joint_id < 0) return 0.0;
    // The wheel hinge axis is +Y, whose positive rotation rolls toward -X.
    // Apollo wheel speed is positive in the vehicle-forward (+X) direction.
    return -d->qvel[m->jnt_dofadr[joint_id]] * model_wheel_radius_m_;
  };
  state->wheel_speed_mps = {
      wheel_speed(wheel_fl_joint_id_), wheel_speed(wheel_fr_joint_id_),
      wheel_speed(wheel_rl_joint_id_), wheel_speed(wheel_rr_joint_id_)};
  if (steer_fl_joint_id_ >= 0 && steer_fr_joint_id_ >= 0) {
    int fl_adr = m->jnt_qposadr[steer_fl_joint_id_];
    int fr_adr = m->jnt_qposadr[steer_fr_joint_id_];
    state->front_steering_rad = 0.5 * (d->qpos[fl_adr] + d->qpos[fr_adr]);
    state->steering_percentage = std::max(
        -100.0, std::min(100.0, state->front_steering_rad /
                                      max_steer_angle_rad_ * 100.0));
  }
  if (steer_rl_joint_id_ >= 0 && steer_rr_joint_id_ >= 0) {
    const int rl_adr = m->jnt_qposadr[steer_rl_joint_id_];
    const int rr_adr = m->jnt_qposadr[steer_rr_joint_id_];
    state->rear_steering_rad = 0.5 * (d->qpos[rl_adr] + d->qpos[rr_adr]);
  }
  if (has_previous_state_ && sim_time_sec_ > previous_state_time_sec_) {
    const double dt = sim_time_sec_ - previous_state_time_sec_;
    const double yaw_delta =
        std::remainder(state->yaw - previous_yaw_, 2.0 * M_PI);
    state->angular_velocity_yaw_radps = yaw_delta / dt;
    for (int i = 0; i < 3; ++i) {
      state->linear_acceleration_world_mps2[i] =
          (state->linear_velocity_world_mps[i] -
           previous_world_velocity_mps_[i]) /
          dt;
    }
    const auto acceleration_body =
        RotateWorldToBody(quaternion, state->linear_acceleration_world_mps2);
    state->linear_acceleration_body_mps2 = acceleration_body;
    state->linear_acceleration_mps2 = acceleration_body[0];
    state->lateral_acceleration_mps2 = acceleration_body[1];
  }
  auto is_vehicle_body = [m, body_id](int candidate) {
    while (candidate >= 0) {
      if (candidate == body_id) return true;
      const int parent = m->body_parentid[candidate];
      if (parent == candidate) break;
      candidate = parent;
    }
    return false;
  };
  state->is_collision = false;
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& contact = d->contact[i];
    const int body1 = m->geom_bodyid[contact.geom1];
    const int body2 = m->geom_bodyid[contact.geom2];
    if ((is_vehicle_body(body1) && body2 != 0 && !is_vehicle_body(body2)) ||
        (is_vehicle_body(body2) && body1 != 0 && !is_vehicle_body(body1))) {
      state->is_collision = true;
      break;
    }
  }
  previous_velocity_mps_ = state->linear_velocity_mps;
  previous_lateral_velocity_mps_ = state->lateral_velocity_mps;
  previous_world_velocity_mps_ = state->linear_velocity_world_mps;
  previous_yaw_ = state->yaw;
  previous_state_time_sec_ = sim_time_sec_;
  has_previous_state_ = true;
  cached_state_ = *state;
  return true;
#else
  *state = cached_state_;
  state->timestamp_sec = sim_time_sec_;
  return true;
#endif
}

}  // namespace simulation
}  // namespace apollo
