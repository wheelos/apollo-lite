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

#include "Eigen/Eigen"

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/drivers/lidar/fusion/proto/lidar_fusion_config.pb.h"

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace drivers {
namespace lidar {

class LidarFusionComponent
    : public apollo::cyber::Component<apollo::drivers::PointCloud> {
 public:
  bool Init() override;

  bool Proc(
      const std::shared_ptr<apollo::drivers::PointCloud>& point_cloud) override;

 private:
  /**
   * @brief Get lidar point real timestamp.
   * if not use system clock, return raw timestamp directly,
   * else return the result of adding raw timestamp and offset time.
   *
   * @param timestamp The point raw timestamp.
   *
   * @return The new timestamp in nanoseconds.
   */
  uint64_t GetPointTimestamp(const uint64_t& timestamp);

  bool IsExpired(const std::shared_ptr<apollo::drivers::PointCloud>& target,
                 const std::shared_ptr<apollo::drivers::PointCloud>& source);

  bool QueryPoseAffine(const uint64_t& timestamp,
                       const std::string& target_frame_id,
                       const std::string& source_frame_id,
                       Eigen::Affine3d* pose);

  void AppendPointCloud(
      std::shared_ptr<apollo::drivers::PointCloud> target,
      const std::shared_ptr<apollo::drivers::PointCloud> source,
      const Eigen::Affine3f& pose);

  bool Fusion(std::shared_ptr<apollo::drivers::PointCloud>& target_pc,
              const std::shared_ptr<apollo::drivers::PointCloud>& source_pc);

  LidarFusionConfig config_;
  apollo::transform::Buffer* tf2_buffer_ptr_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> writer_;
  std::vector<
      std::shared_ptr<apollo::cyber::Reader<apollo::drivers::PointCloud>>>
      readers_;
  // std::vector<std::shared_ptr<apollo::drivers::PointCloud>>
  // point_cloud_pool_;
  std::map<
      std::string, Eigen::Affine3f, std::less<std::string>,
      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>>
      static_tf_map_;

  // lidar to system clock offset nanoseconds
  int64_t lidar_system_offset_ns_ = 0;

  std::vector<std::shared_ptr<PointCloud>> point_cloud_pool_;
  size_t pool_size_ = 10;
  size_t pool_index_ = 0;
  size_t reserved_point_size_ = 500000;
};

CYBER_REGISTER_COMPONENT(LidarFusionComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
