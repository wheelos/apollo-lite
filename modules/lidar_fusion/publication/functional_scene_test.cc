// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/publication/functional_scene.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "modules/lidar_fusion/core/runtime/fusion_test_support.h"

namespace apollo {
namespace lidar_fusion {
using testing_support::Config;
using testing_support::FusionTest;

TEST_F(FusionTest, DiagnosticPublicationRequiresDistinctFunctionalChannel) {
  SingleFrameFusion fusion;
  std::string error;
  config.mutable_publication()->set_diagnostic_channel("/test/full");
  EXPECT_FALSE(fusion.Init(config, &error));
  config.mutable_publication()->set_detail(FUNCTIONAL_SCENE);
  EXPECT_TRUE(fusion.Init(config, &error)) << error;
  for (const auto& channel :
       {config.pointcloud_channel(), config.detection_channel(),
        config.segmentation_channel(), config.output_channel()}) {
    config.mutable_publication()->set_diagnostic_channel(channel);
    EXPECT_FALSE(fusion.Init(config, &error));
  }
}

TEST_F(FusionTest, FunctionalProjectionRejectsWrongSweepAndAliasing) {
  Obstacle();
  Box();
  Run(&det, &seg);
  const auto complete = output.SerializeAsString();
  FusionOutput scene;
  scene.set_odom_frame("unchanged-on-error");
  const auto before = scene.SerializeAsString();
  auto wrong = cloud;
  wrong.mutable_header()->set_lidar_timestamp(cloud.header().lidar_timestamp() +
                                              1);
  std::string error;
  EXPECT_FALSE(ProjectFunctionalOutput(config, wrong, output, &scene, &error));
  EXPECT_EQ(scene.SerializeAsString(), before);
  EXPECT_FALSE(ProjectFunctionalOutput(config, cloud, output, &output, &error));
  EXPECT_EQ(output.SerializeAsString(), complete);
  ASSERT_TRUE(ProjectFunctionalOutput(config, cloud, output, &scene, &error));
  EXPECT_FALSE(ProjectFunctionalOutput(config, cloud, scene, &output, &error));
  EXPECT_EQ(output.SerializeAsString(), complete);
}

TEST_F(FusionTest, FunctionalScenePreservesAllNonDiagnosticFieldsInEveryMode) {
  Obstacle();
  Box();
  config.mutable_spatial()->set_point_views(true);
  config.mutable_candidates()->set_shadow_semantic_partitions(true);
  config.mutable_publication()->set_detail(FUNCTIONAL_SCENE);
  config.mutable_publication()->set_diagnostic_channel("/test/full");
  for (int mode = 0; mode < 4; ++mode) {
    Run(mode == 2 || mode == 3 ? nullptr : &det,
        mode == 1 || mode == 3 ? nullptr : &seg);
    EXPECT_EQ(output.health().mode(), static_cast<Mode>(mode));
    output.set_odom_frame("fixture_odom");
    output.add_track()->set_id(42);
    output.add_history_cell()->set_log_odds(0.7);
    output.add_shadow_partition()->set_parent_unknown_id(31);
    const auto original = output.SerializeAsString();
    FusionOutput scene;
    std::string error;
    ASSERT_TRUE(ProjectFunctionalOutput(config, cloud, output, &scene, &error))
        << error;
    EXPECT_EQ(output.SerializeAsString(), original);
    EXPECT_EQ(scene.point_evidence_size(), 0);
    EXPECT_EQ(scene.point_decision_size(), 0);
    const auto& context = scene.publication();
    EXPECT_EQ(context.detail(), FUNCTIONAL_SCENE);
    EXPECT_EQ(context.source_point_count(), cloud.point_size());
    EXPECT_EQ(context.source_header().SerializeAsString(),
              cloud.header().SerializeAsString());
    EXPECT_EQ(context.source_measurement_time(), cloud.measurement_time());
    EXPECT_EQ(context.source_pointcloud_channel(), config.pointcloud_channel());
    EXPECT_EQ(context.sensor_to_base().SerializeAsString(),
              config.sensor_to_base().SerializeAsString());
    EXPECT_TRUE(context.spatial_options().point_views());
    EXPECT_EQ(context.diagnostic_channel(), "/test/full");
    EXPECT_EQ(context.shadow_partitions_enabled(), mode == 0 || mode == 2);
    auto expected = output;
    expected.clear_point_evidence();
    expected.clear_point_decision();
    scene.clear_publication();
    EXPECT_EQ(scene.SerializeAsString(), expected.SerializeAsString());
  }
}

TEST_F(FusionTest, PodTransportIsOptInAndRejectsUnboundedFrameBudgets) {
  SingleFrameFusion fusion;
  std::string error;
  EXPECT_EQ(config.publication().transport(), PROTOBUF);
  EXPECT_TRUE(fusion.Init(config, &error)) << error;
  config.mutable_publication()->set_transport(CORE_POD);
  for (auto detail : {FULL_EVIDENCE, FUNCTIONAL_SCENE}) {
    config.mutable_publication()->set_detail(detail);
    EXPECT_TRUE(fusion.Init(config, &error)) << error;
  }
  for (uint32_t limit : {0u, 0x80000000u, 0xffffffffu}) {
    config.mutable_publication()->set_max_pod_frame_bytes(limit);
    EXPECT_FALSE(fusion.Init(config, &error));
  }
}

}  // namespace lidar_fusion
}  // namespace apollo
