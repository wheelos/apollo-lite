// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_object_detection/component/lidar_object_detection_component.h"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "google/protobuf/text_format.h"

namespace apollo::lidar_object_detection {

namespace {

bool LoadPointPillarsSample(const std::string& path,
                            drivers::PointCloud* cloud) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }
  float point[4];
  while (input.read(reinterpret_cast<char*>(point), sizeof(point))) {
    for (const float value : point) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    auto* output = cloud->add_point();
    output->set_x(point[0]);
    output->set_y(point[1]);
    output->set_z(point[2]);
    output->set_intensity(point[3]);
  }
  return input.eof() && input.gcount() == 0 && cloud->point_size() > 0;
}

}  // namespace

TEST(LidarObjectDetectionComponent,
     RunsAndReloadsPointPillarsFromUnifiedConfiguration) {
  const char* runtime = std::getenv("POINTPILLARS_RUNTIME");
  const char* engine = std::getenv("POINTPILLARS_ENGINE");
  if (runtime == nullptr || engine == nullptr) {
    GTEST_SKIP() << "Set POINTPILLARS_RUNTIME and POINTPILLARS_ENGINE";
  }
  const char* configured_root = std::getenv("APOLLO_ROOT_DIR");
  const std::string software_root =
      configured_root != nullptr && *configured_root != '\0' ? configured_root
                                                             : "/apollo";
  for (int iteration = 0; iteration < 2; ++iteration) {
    LidarObjectDetectionComponentConfig config;
    auto* model = config.mutable_pointpillars();
    model->set_runtime_library_path(runtime);
    model->set_engine_path(engine);
    model->set_intensity_scale(1.0F);
    model->set_model_revision("pinned-kitti-test");
    config.set_sensor_frame_id("migration_test_lidar");
    config.set_output_channel("/migration_test/lidar_detection/output");
    std::string text;
    ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(config, &text));
    std::string path = software_root + "/.lidar-object-detection-XXXXXX";
    const int descriptor = mkstemp(path.data());
    ASSERT_GE(descriptor, 0);
    close(descriptor);
    {
      std::ofstream output(path);
      output << text;
      ASSERT_TRUE(output.good());
    }
    cyber::proto::ComponentConfig component_config;
    component_config.set_name("lidar_detection_migration_test_" +
                              std::to_string(iteration));
    component_config.set_config_file_path(path);
    component_config.add_readers()->set_channel(
        "/migration_test/lidar_detection/input");
    auto component = std::make_shared<LidarObjectDetectionComponent>();
    const bool initialized = component->Initialize(component_config);
    EXPECT_EQ(std::remove(path.c_str()), 0);
    ASSERT_TRUE(initialized);
    auto cloud = std::make_shared<drivers::PointCloud>();
    cloud->set_frame_id("migration_test_lidar");
    cloud->set_measurement_time(42);
    const char* sample = std::getenv("POINTPILLARS_SAMPLE");
    if (sample != nullptr && *sample != '\0') {
      ASSERT_TRUE(LoadPointPillarsSample(sample, cloud.get()));
    } else {
      auto* point = cloud->add_point();
      point->set_x(2);
      point->set_y(0);
      point->set_z(-1);
      point->set_intensity(0.5F);
    }
    ASSERT_TRUE(component->Proc(cloud));
    cloud->set_frame_id("wrong_frame");
    EXPECT_FALSE(component->Proc(cloud));
    cloud->set_frame_id("migration_test_lidar");
    EXPECT_TRUE(component->Proc(cloud));
    component->Shutdown();
  }
}
}  // namespace apollo::lidar_object_detection

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  apollo::cyber::Init(argv[0]);
  return RUN_ALL_TESTS();
}
