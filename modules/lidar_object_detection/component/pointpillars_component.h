// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "modules/lidar_object_detection/proto/pointpillars.pb.h"
#include "wheelos_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/cyber.h"
#include "modules/lidar_object_detection/inference/pointpillars_runtime_api.h"

namespace apollo::lidar_object_detection {
class PointPillarsComponent : public cyber::Component<drivers::PointCloud> {
 public:
  ~PointPillarsComponent() override;
  bool Init() override;
  bool Proc(const std::shared_ptr<drivers::PointCloud>& cloud) override;

 private:
  PointPillarsComponentConfig config_;
  void* library_ = nullptr;
  void* runtime_ = nullptr;
  decltype(&pointpillars_runtime_destroy) destroy_ = nullptr;
  decltype(&pointpillars_runtime_infer) infer_ = nullptr;
  std::shared_ptr<cyber::Writer<PointPillarsFrameResult>> writer_;
  std::mutex mutex_;
};
CYBER_REGISTER_COMPONENT(PointPillarsComponent);
}  // namespace apollo::lidar_object_detection
