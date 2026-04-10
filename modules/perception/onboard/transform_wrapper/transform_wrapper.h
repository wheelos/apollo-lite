/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#pragma once

#include <memory>
#include <string>

#include "Eigen/Geometry"
#include "gflags/gflags.h"

#include "modules/transform/timed_transform_resolver.h"
#include "modules/transform/transform_query.h"

namespace apollo {
namespace perception {
namespace onboard {

DECLARE_string(obs_sensor2vehicle_tf2_frame_id);
DECLARE_string(obs_vehicle2world_tf2_frame_id);
DECLARE_string(obs_vehicle2world_tf2_child_frame_id);

class TransformWrapper {
 public:
  TransformWrapper() : timed_transform_resolver_(&transform_query_) {}
  ~TransformWrapper() = default;

  void Init(const std::string& sensor2vehicle_tf2_child_frame_id);
  void Init(const std::string& sensor2vehicle_tf2_frame_id,
            const std::string& sensor2vehicle_tf2_child_frame_id,
            const std::string& vehicle2world_tf2_frame_id,
            const std::string& vehicle2world_tf2_child_frame_id);

  // Attention: must initialize TransformWrapper first
  bool GetSensor2worldTrans(double timestamp,
                            Eigen::Affine3d* sensor2world_trans,
                            Eigen::Affine3d* vehicle2world_trans = nullptr);

  bool GetExtrinsics(Eigen::Affine3d* trans);

  // Attention: can be called without initlization
  bool GetTrans(double timestamp, Eigen::Affine3d* trans,
                const std::string& frame_id, const std::string& child_frame_id);

  bool GetExtrinsicsBySensorId(const std::string& from_sensor_id,
                               const std::string& to_sensor_id,
                               Eigen::Affine3d* trans);

 protected:
  bool QueryTrans(double timestamp, apollo::transform::StampedTransform* trans,
                  const std::string& frame_id,
                  const std::string& child_frame_id);

  bool QueryStaticTrans(const std::string& frame_id,
                        const std::string& child_frame_id,
                        Eigen::Affine3d* trans);

  bool EnsureSensor2VehicleExtrinsics();

 private:
  bool inited_ = false;

  std::string sensor2vehicle_tf2_frame_id_;
  std::string sensor2vehicle_tf2_child_frame_id_;
  std::string vehicle2world_tf2_frame_id_;
  std::string vehicle2world_tf2_child_frame_id_;

  std::unique_ptr<Eigen::Affine3d> sensor2vehicle_extrinsics_;

  apollo::transform::TransformQuery transform_query_;
  apollo::transform::TimedTransformResolver timed_transform_resolver_;
};

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
