/******************************************************************************
 * Copyright 2026 The Wheel.OS Authors. All Rights Reserved.
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

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/perception/lidar/common/pcl_util.h"

namespace apollo {
namespace perception {
namespace lidar {

class PcdExporter {
 public:
  using PcMsg = apollo::drivers::PointCloud;

  explicit PcdExporter(const std::string& output_dir)
      : output_dir_(output_dir) {}

  void OnPointCloud(const std::shared_ptr<const PcMsg>& cloud_msg) {
    if (!cloud_msg) return;

    const double timestamp =
        cloud_msg->has_measurement_time()
            ? cloud_msg->measurement_time()
            : (cloud_msg->has_header() ? cloud_msg->header().timestamp_sec()
                                       : 0.0);

    pcl::PointCloud<PCLPointXYZIT> cloud;
    if (cloud_msg->point_size() > 0) {
      cloud.resize(cloud_msg->point_size());
      for (int i = 0; i < cloud_msg->point_size(); ++i) {
        const apollo::drivers::PointXYZIT& pt = cloud_msg->point(i);
        if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
          continue;
        }
        if (std::fabs(pt.x()) > 1e3 || std::fabs(pt.y()) > 1e3 ||
            std::fabs(pt.z()) > 1e3) {
          continue;
        }
        cloud[i].x = pt.x();
        cloud[i].y = pt.y();
        cloud[i].z = pt.z();
        cloud[i].intensity = static_cast<std::uint8_t>(
            std::min<std::uint32_t>(pt.intensity(), 255u));
        cloud[i].timestamp = static_cast<double>(pt.timestamp()) * 1e-9;
      }
    }

    SavePointCloud(cloud, timestamp);
  }

 private:
  void SavePointCloud(const pcl::PointCloud<PCLPointXYZIT>& point_cloud,
                      double timestamp) {
    const std::uint64_t seq = seq_.fetch_add(1, std::memory_order_relaxed);
    char path[1024];
    // Keep the stem stable (timestamp) for downstream tools; add seq to avoid
    // collisions when timestamps repeat.
    std::snprintf(path, sizeof(path), "%s/%.6f_%lu.pcd", output_dir_.c_str(),
                  timestamp, static_cast<unsigned long>(seq));
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(path, point_cloud);
    AINFO << "Saved: " << path << " (#points=" << point_cloud.size() << ")";
  }

  std::string output_dir_;
  std::atomic<std::uint64_t> seq_{0};
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: pcd_exporter <pointcloud_channel> <output_dir>\n";
    return 2;
  }

  const std::string channel = argv[1];
  const std::string output_dir = argv[2];

  apollo::cyber::Init(argv[0]);
  auto node = apollo::cyber::CreateNode("pcd_exporter");
  if (!node) {
    std::cerr << "Failed to create cyber node.\n";
    return 1;
  }

  if (!apollo::cyber::common::DirectoryExists(output_dir)) {
    if (!apollo::cyber::common::CreateDirectories(output_dir)) {
      std::cerr << "Failed to create output dir: " << output_dir << "\n";
      return 1;
    }
  }

  apollo::perception::lidar::PcdExporter exporter(output_dir);
  auto reader = node->CreateReader<apollo::drivers::PointCloud>(
      channel, [&exporter](const auto& msg) { exporter.OnPointCloud(msg); });
  if (!reader) {
    std::cerr << "Failed to create reader for channel: " << channel << "\n";
    return 1;
  }

  std::cout << "Listening on channel: " << channel << "\n";
  std::cout << "Writing PCDs to: " << output_dir << "\n";
  apollo::cyber::WaitForShutdown();
  return 0;
}
