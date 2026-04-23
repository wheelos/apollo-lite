/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include <vector>

#include "modules/drivers/lidar/compensator/proto/lidar_compensator_config.pb.h"

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/node/writer.h"
#include "modules/drivers/lidar/compensator/compensator.h"

namespace apollo {
namespace drivers {
namespace lidar {

class LidarCompensatorComponent : public apollo::cyber::Component<PointCloud> {
 public:
  bool Init() override;
  bool Proc(
      const std::shared_ptr<apollo::drivers::PointCloud>& point_cloud) override;

 private:
  LidarCompensatorConfig config_;
  std::unique_ptr<Compensator> compensator_ = nullptr;
  int pool_size_ = 8;
  int seq_ = 0;
  std::shared_ptr<apollo::cyber::Writer<PointCloud>> writer_ = nullptr;
  std::shared_ptr<apollo::cyber::base::CCObjectPool<PointCloud>>
      compensator_pool_ = nullptr;
};

CYBER_REGISTER_COMPONENT(LidarCompensatorComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
