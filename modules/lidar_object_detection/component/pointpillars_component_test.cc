// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_object_detection/component/pointpillars_component.h"

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "google/protobuf/text_format.h"

namespace apollo::lidar_object_detection {
TEST(PointPillarsComponent, RunsAndReloadsWithExplicitModelConfiguration) {
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
    PointPillarsComponentConfig model;
    model.set_runtime_library_path(runtime);
    model.set_engine_path(engine);
    model.set_sensor_frame_id("migration_test_lidar");
    model.set_output_channel("/migration_test/pointpillars/output");
    model.set_intensity_scale(1.0F / 255.0F);
    model.set_model_revision("pinned-kitti-test");
    std::string text;
    ASSERT_TRUE(google::protobuf::TextFormat::PrintToString(model, &text));
    std::string path = software_root + "/.pointpillars-component-XXXXXX";
    const int descriptor = mkstemp(path.data());
    ASSERT_GE(descriptor, 0);
    close(descriptor);
    {
      std::ofstream output(path);
      output << text;
      ASSERT_TRUE(output.good());
    }
    cyber::proto::ComponentConfig config;
    config.set_name("pointpillars_migration_test_" + std::to_string(iteration));
    config.set_config_file_path(path);
    config.add_readers()->set_channel("/migration_test/pointpillars/input");
    auto component = std::make_shared<PointPillarsComponent>();
    const bool initialized = component->Initialize(config);
    EXPECT_EQ(std::remove(path.c_str()), 0);
    ASSERT_TRUE(initialized);
    auto cloud = std::make_shared<drivers::PointCloud>();
    cloud->set_frame_id("migration_test_lidar");
    cloud->set_measurement_time(42);
    auto* point = cloud->add_point();
    point->set_x(2);
    point->set_y(0);
    point->set_z(-1);
    point->set_intensity(128);
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
