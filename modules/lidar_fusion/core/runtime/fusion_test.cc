// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "modules/lidar_fusion/core/alignment/sweep_identity.h"
#include "modules/lidar_fusion/core/runtime/fusion_test_support.h"

namespace apollo {
namespace lidar_fusion {
using testing_support::Config;
using testing_support::FusionTest;

TEST_F(FusionTest, CorridorNoiseUsesHigherConfidenceAndAllModesAreExplicit) {
  Add(2.0, 1.4, 0.8, 20);
  seg.mutable_point_label(6)->set_confidence(0.7F);
  Sync();
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), NORMAL);
  EXPECT_EQ(output.point_decision(6).explanation(), UNEXPLAINED);
  Run(nullptr, &seg);
  EXPECT_EQ(output.health().mode(), NO_DET);
  seg.mutable_point_label(6)->set_confidence(0.95F);
  Run(nullptr, &seg);
  EXPECT_EQ(output.point_decision(6).explanation(), EXPL_NOISE);
  Run(nullptr, nullptr);
  EXPECT_EQ(output.health().mode(), GEO_ONLY);
  EXPECT_FALSE(output.health().temporal_enabled());
}

TEST_F(FusionTest, FullRigidCalibrationTransformsPointsBoxesAndVelocity) {
  Obstacle();
  Box();
  auto* t = config.mutable_sensor_to_base();
  t->set_x(1);
  t->set_y(-1);
  t->set_yaw(std::acos(-1.0) / 2);
  t->set_pitch(0.1);
  det.mutable_detection(0)->set_velocity_x(1);
  det.mutable_detection(0)->set_velocity_y(0);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  const auto& center = output.object(0).box().center();
  EXPECT_NEAR(center.x(), 1 - 1.4, 1e-6);
  EXPECT_NEAR(center.y(), -1 + 2 * std::cos(0.1) + 0.8 * std::sin(0.1), 1e-6);
  EXPECT_NEAR(center.z(), -2 * std::sin(0.1) + 0.8 * std::cos(0.1), 1e-6);
  EXPECT_NEAR(output.object(0).velocity().y(), std::cos(0.1), 1e-6);
  EXPECT_NEAR(output.object(0).velocity().z(), -std::sin(0.1), 1e-6);
}

TEST_F(FusionTest, MalformedSegmentationDegradesInsteadOfShiftingIndices) {
  Obstacle();
  Box();
  seg.mutable_point_label(1)->set_point_index(0);
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), NO_SEG);
  EXPECT_EQ(output.health().seg_reason(), SOURCE_INVALID);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).consistency(), INSUFFICIENT);
  EXPECT_FALSE(output.point_evidence(6).has_raw_semantic_label());
}

TEST_F(FusionTest, RejectsInvalidConfigurationAndCloudRatherThanStaleOutput) {
  SingleFrameFusion fusion;
  std::string error;
  config.mutable_sensor_to_base()->clear_version();
  EXPECT_FALSE(fusion.Init(config, &error));
  config = Config();
  ASSERT_TRUE(fusion.Init(config, &error));
  output.add_object()->set_id(99);
  cloud.set_frame_id("other_lidar");
  EXPECT_FALSE(fusion.Process(cloud, nullptr, nullptr, &output, &error));
  EXPECT_EQ(output.object_size(), 0);
}

TEST_F(FusionTest, RejectsMisalignedAndOutOfRangeModelInputsIndependently) {
  Obstacle();
  Box();
  det.mutable_header()->set_lidar_timestamp(99);
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), NO_DET);
  EXPECT_EQ(output.health().det_reason(), SOURCE_MISALIGNED);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).source(), SEG_RECALL);
  seg.mutable_point_label(0)->set_point_index(cloud.point_size());
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), GEO_ONLY);
  EXPECT_EQ(output.health().seg_reason(), SOURCE_INVALID);
}

TEST_F(FusionTest, RemapsShuffledLabelsWithoutChangingInputs) {
  Obstacle();
  Box();
  std::reverse(seg.mutable_point_label()->begin(),
               seg.mutable_point_label()->end());
  const std::string raw_cloud = cloud.SerializeAsString();
  const std::string raw_seg = seg.SerializeAsString();
  const std::string raw_det = det.SerializeAsString();
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), NORMAL);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).consistency(), SUPPORT);
  EXPECT_FLOAT_EQ(output.object(0).raw_score(), 0.8F);
  EXPECT_FALSE(output.object(0).has_velocity());
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_EQ(output.point_evidence(6).raw_semantic_label(), 1);
  EXPECT_EQ(output.point_decision(6).object_id(), output.object(0).id());
  EXPECT_EQ(raw_cloud, cloud.SerializeAsString());
  EXPECT_EQ(raw_seg, seg.SerializeAsString());
  EXPECT_EQ(raw_det, det.SerializeAsString());
  EXPECT_EQ(output.header().frame_id(), "base_link");
  EXPECT_DOUBLE_EQ(output.header().timestamp_sec(), 100);
  const auto& semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_TRUE(semantic.source_available());
  EXPECT_TRUE(semantic.identity_qualified());
  EXPECT_FALSE(semantic.cross_thing_disagreement());
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).unified_class(), CAR);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 6);
}

TEST_F(FusionTest, SemanticConfidenceAndFiniteGeometryAreValidated) {
  Obstacle();
  Box();
  seg.mutable_point_label(0)->set_confidence(
      std::numeric_limits<float>::quiet_NaN());
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), NO_SEG);
  seg.mutable_point_label(0)->set_confidence(0.95F);
  det.mutable_detection(0)->set_center_x(
      std::numeric_limits<float>::infinity());
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), NO_DET);
  cloud.mutable_point(6)->set_x(std::numeric_limits<float>::quiet_NaN());
  Run(nullptr, &seg);
  ASSERT_EQ(output.point_evidence_size(), cloud.point_size());
  EXPECT_FALSE(output.point_evidence(6).finite());
  EXPECT_EQ(output.point_decision(6).explanation(), INVALID_POINT);
  EXPECT_EQ(output.point_evidence(6).raw_semantic_label(), 1);
}

TEST_F(FusionTest, SourceFrameTopicAndTaxonomyAreNotInterchangeable) {
  Obstacle();
  Box();
  det.set_backend(
      lidar_object_detection::LIDAR_OBJECT_DETECTION_BACKEND_POINTPILLARS);
  seg.set_source_topic("/other/cloud");
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), GEO_ONLY);
  EXPECT_EQ(output.health().det_reason(), SOURCE_INVALID);
  EXPECT_EQ(output.health().seg_reason(), SOURCE_MISALIGNED);
  det.set_backend(config.detection_backend());
  det.set_sensor_frame_id("other_lidar");
  seg.set_source_topic(config.pointcloud_channel());
  seg.set_source_frame_id("other_lidar");
  Run(&det, &seg);
  EXPECT_EQ(output.health().mode(), GEO_ONLY);
}

TEST(SweepIdentity, SupportsCopiedHeaderAndPointPillarsWithoutNearestMatching) {
  drivers::PointCloud cloud;
  cloud.set_measurement_time(42);
  cloud.mutable_header()->set_sequence_num(3);
  cloud.mutable_header()->set_timestamp_sec(42.01);
  common::Header result = cloud.header();
  EXPECT_TRUE(SameSweep(cloud, result));
  result.set_timestamp_sec(42);
  EXPECT_TRUE(SameSweep(cloud, result));
  result.set_timestamp_sec(42.001);
  EXPECT_FALSE(SameSweep(cloud, result));
  result.set_timestamp_sec(42);
  result.set_sequence_num(4);
  EXPECT_FALSE(SameSweep(cloud, result));
}

}  // namespace lidar_fusion
}  // namespace apollo
