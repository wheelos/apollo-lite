// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_fusion/core/fusion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "gtest/gtest.h"

namespace apollo {
namespace lidar_fusion {
namespace {

FusionConfig Config() {
  FusionConfig c;
  c.set_pointcloud_channel("/test/cloud");
  c.set_detection_channel("/test/det");
  c.set_segmentation_channel("/test/seg");
  c.set_output_channel("/test/fusion");
  c.set_sensor_frame("lidar");
  c.set_detection_backend(
      lidar_object_detection::LIDAR_OBJECT_DETECTION_BACKEND_CENTERPOINT);
  auto* t = c.mutable_sensor_to_base();
  t->set_version("fixture-calibration");
  t->set_x(0);
  t->set_y(0);
  t->set_z(0);
  t->set_roll(0);
  t->set_pitch(0);
  t->set_yaw(0);
  auto* det = c.add_detection_mapping();
  det->set_raw_id(0);
  det->set_group(THING);
  det->set_unified_class(CAR);
  for (const auto& entry : {std::make_pair(0U, GENERIC_OBS),
                            {1U, THING},
                            {9U, SURFACE},
                            {13U, STATIC_STRUCT},
                            {15U, VEGETATION},
                            {20U, NOISE}}) {
    auto* mapping = c.add_semantic_mapping();
    mapping->set_raw_id(entry.first);
    mapping->set_group(entry.second);
    if (entry.second == THING) {
      mapping->set_unified_class(CAR);
    }
  }
  c.set_connectivity_size(0.2);
  c.set_min_x(0);
  c.set_max_x(8);
  c.set_min_y(-4);
  c.set_max_y(4);
  return c;
}

class FusionTest : public testing::Test {
 protected:
  void SetUp() override {
    config = Config();
    cloud.set_frame_id("lidar");
    cloud.set_measurement_time(100);
    cloud.mutable_header()->set_frame_id("lidar");
    cloud.mutable_header()->set_sequence_num(7);
    cloud.mutable_header()->set_lidar_timestamp(100000000000ULL);
    cloud.mutable_header()->set_timestamp_sec(100.01);
    // Independent, flat, noncollinear ground samples in one fitting tile.
    for (double y : {0.2, 0.8}) {
      for (double x : {1.0, 1.6, 2.2}) {
        Add(x, y, 0, 9);
      }
    }
    Sync();
  }

  void Add(double x, double y, double z, uint32_t label) {
    auto* p = cloud.add_point();
    p->set_x(x);
    p->set_y(y);
    p->set_z(z);
    auto* s = seg.add_point_label();
    s->set_point_index(cloud.point_size() - 1);
    s->set_semantic_label(label);
    s->set_confidence(0.95F);
  }

  void Obstacle(uint32_t label = 1) {
    for (double y : {1.3, 1.4}) {
      for (double x : {1.9, 2.0, 2.1}) {
        Add(x, y, 0.8, label);
      }
    }
    Sync();
  }

  void Box(float score = 0.8F) {
    auto* d = det.add_detection();
    d->set_center_x(2);
    d->set_center_y(1.4);
    d->set_center_z(0.8);
    d->set_size_x(0.8);
    d->set_size_y(0.6);
    d->set_size_z(0.8);
    d->set_yaw(0);
    d->set_class_id(0);
    d->set_score(score);
  }

  void Sync() {
    *det.mutable_header() = cloud.header();
    det.set_sensor_frame_id("lidar");
    det.set_model_revision("fixture-model");
    det.set_backend(config.detection_backend());
    // Accumulated sweep count is intentionally not the current cloud count.
    det.set_input_point_count(1000);
    *seg.mutable_header() = cloud.header();
    seg.set_source_frame_id("lidar");
    seg.set_source_topic(config.pointcloud_channel());
    seg.set_point_count(cloud.point_size());
  }

  void Run(const DetectionFrame* d, const SemanticFrame* s,
           const GroundPrior* ground_prior = nullptr,
           const ObjectPrior* object_prior = nullptr) {
    SingleFrameFusion fusion;
    std::string error;
    ASSERT_TRUE(fusion.Init(config, &error)) << error;
    ASSERT_TRUE(fusion.Process(cloud, d, s, &output, &error, ground_prior,
                               object_prior))
        << error;
  }

  FusionConfig config;
  drivers::PointCloud cloud;
  DetectionFrame det;
  SemanticFrame seg;
  FusionOutput output;
};

TEST_F(FusionTest, GroundHistoryCannotEraseCurrentMeasuredObstacle) {
  Obstacle(9);
  Run(nullptr, &seg);
  const auto baseline = output.space_map().SerializeAsString();
  GroundPrior prior;
  for (const auto& p : output.point_evidence()) {
    auto ground = p.position();
    ground.set_z(0.8);
    prior.push_back(ground);
  }
  Run(nullptr, &seg, &prior);
  EXPECT_EQ(output.space_map().SerializeAsString(), baseline);
  ASSERT_TRUE(output.point_evidence(6).has_ground_prior_z());
  EXPECT_FALSE(output.point_evidence(6).ground_prior_consistent());
  EXPECT_FALSE(output.point_evidence(6).is_ground_geo());
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

TEST_F(FusionTest, IndependentGroundPriorDoesNotChangeObjectPriorRecovery) {
  Obstacle(0);
  Box();
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  const ObjectPrior objects{output.object(0)};
  const auto prediction = objects.front().SerializeAsString();
  GroundPrior ground;
  for (const auto& point : output.point_evidence()) {
    auto sample = point.position();
    sample.set_z(0);
    ground.push_back(sample);
  }
  Run(nullptr, &seg, nullptr, &objects);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).source(), TRACK_PRIOR);
  EXPECT_FALSE(output.point_evidence(6).has_ground_prior_z());
  const auto recovered = output.object(0).SerializeAsString();
  const auto space = output.space_map().SerializeAsString();

  Run(nullptr, &seg, &ground, &objects);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).SerializeAsString(), recovered);
  EXPECT_EQ(output.space_map().SerializeAsString(), space);
  EXPECT_TRUE(output.point_evidence(6).has_ground_prior_z());
  EXPECT_EQ(objects.front().SerializeAsString(), prediction);
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

TEST_F(FusionTest, CrossThingEvidencePreservesLegacyClassAndConsumerPolicy) {
  auto* mapping = config.add_semantic_mapping();
  mapping->set_raw_id(2);
  mapping->set_group(THING);
  mapping->set_unified_class(PEDESTRIAN);
  Obstacle(2);
  Box();
  Run(&det, &seg);
  const auto& hypothesis = output.hypothesis(0);
  const auto& semantic = hypothesis.association().semantic_evidence();
  ASSERT_TRUE(semantic.cross_thing_disagreement());
  EXPECT_EQ(hypothesis.consistency(), INSUFFICIENT);
  EXPECT_TRUE(hypothesis.existence_supported());
  EXPECT_FALSE(hypothesis.identity_supported());
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).unified_class(), PEDESTRIAN);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 6);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).unified_class(), CAR);
  EXPECT_FALSE(output.object(0).class_conflict());
  EXPECT_EQ(output.object(0).classification_use(), CLASSIFICATION_CANDIDATE);
  EXPECT_EQ(hypothesis.raw_detection().SerializeAsString(),
            det.detection(0).SerializeAsString());
}

TEST_F(FusionTest, DisagreementIgnoresMarginAndCannotAcceptWeakDetection) {
  auto* mapping = config.add_semantic_mapping();
  mapping->set_raw_id(2);
  mapping->set_group(THING);
  mapping->set_unified_class(PEDESTRIAN);
  Obstacle(2);
  Add(2.35, 1.4, 0.8, 2);
  Add(2.5, 1.4, 0.8, 1);
  Sync();
  Box(0.2F);
  Run(&det, &seg);
  const auto& hypothesis = output.hypothesis(0);
  const auto& association = hypothesis.association();
  const auto& semantic = association.semantic_evidence();
  ASSERT_EQ(association.n_core(), 7U);
  ASSERT_EQ(association.n_margin(), 1U);
  EXPECT_TRUE(semantic.cross_thing_disagreement());
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 7);
  const auto& ids = semantic.thing_support(0).point_id();
  EXPECT_EQ(std::find(ids.begin(), ids.end(), 13U), ids.end());
  EXPECT_EQ(hypothesis.consistency(), INSUFFICIENT);
  EXPECT_EQ(hypothesis.decision(), BELOW_INSTANCE_THRESHOLD);
  EXPECT_FALSE(hypothesis.has_object_id());
  EXPECT_TRUE(std::none_of(
      output.object().begin(), output.object().end(),
      [](const ObjectCandidate& object) { return object.source() == DET; }));
}

TEST_F(FusionTest, SemanticEvidenceSeparatesMissingWeakAndUnqualifiedInput) {
  Obstacle();
  Box();
  Run(&det, nullptr);
  auto semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_FALSE(semantic.source_available());
  EXPECT_FALSE(semantic.identity_qualified());
  EXPECT_EQ(semantic.n_unavailable(), 6U);
  EXPECT_EQ(semantic.thing_support_size(), 0);

  config.mutable_quality()->set_semantic_identity_unverified(true);
  Run(&det, &seg);
  semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_TRUE(semantic.source_available());
  EXPECT_FALSE(semantic.identity_qualified());
  EXPECT_EQ(semantic.n_unavailable(), 0U);
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 6);
  EXPECT_FALSE(semantic.cross_thing_disagreement());

  config.mutable_quality()->set_semantic_identity_unverified(false);
  for (int i = 6; i < seg.point_label_size(); ++i) {
    seg.mutable_point_label(i)->set_confidence(0.49F);
  }
  Run(&det, &seg);
  semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_TRUE(semantic.source_available());
  EXPECT_TRUE(semantic.identity_qualified());
  EXPECT_EQ(semantic.n_low_confidence(), 6U);
  EXPECT_EQ(semantic.thing_support_size(), 0);
  EXPECT_EQ(semantic.group_support_size(), 0);
}

TEST_F(FusionTest, SparseOrMixedAlternativesRemainExplicitButNotStrong) {
  auto* mapping = config.add_semantic_mapping();
  mapping->set_raw_id(2);
  mapping->set_group(THING);
  mapping->set_unified_class(PEDESTRIAN);
  Obstacle(2);
  Box();
  for (int i = 6; i < 9; ++i) {
    seg.mutable_point_label(i)->set_semantic_label(1);
  }
  Run(&det, &seg);
  auto semantic = output.hypothesis(0).association().semantic_evidence();
  ASSERT_EQ(semantic.thing_support_size(), 2);
  EXPECT_FALSE(semantic.cross_thing_disagreement());
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 3);
  EXPECT_EQ(semantic.thing_support(1).point_id_size(), 3);

  for (int i = 0; i < 3; ++i) {
    cloud.mutable_point()->RemoveLast();
    seg.mutable_point_label()->RemoveLast();
  }
  for (int i = 6; i < seg.point_label_size(); ++i) {
    seg.mutable_point_label(i)->set_semantic_label(2);
  }
  Sync();
  Run(&det, &seg);
  semantic = output.hypothesis(0).association().semantic_evidence();
  EXPECT_FALSE(output.hypothesis(0).existence_supported());
  EXPECT_FALSE(semantic.cross_thing_disagreement());
  ASSERT_EQ(semantic.thing_support_size(), 1);
  EXPECT_EQ(semantic.thing_support(0).point_id_size(), 3);
}

TEST_F(FusionTest, ShadowPartitionsAreReadOnlyAndRequireQualifiedSemantics) {
  Obstacle();
  for (double x : {2.2, 2.3, 2.4, 2.5}) {
    for (double y : {1.3, 1.4, 1.5}) {
      Add(x, y, 0.8, 15);
    }
  }
  Sync();
  for (bool qualified : {false, true}) {
    config.mutable_quality()->set_semantic_identity_unverified(!qualified);
    for (int mode = NORMAL; mode <= GEO_ONLY; ++mode) {
      const auto* detection =
          mode == NO_DET || mode == GEO_ONLY ? nullptr : &det;
      const auto* semantic =
          mode == NO_SEG || mode == GEO_ONLY ? nullptr : &seg;
      config.mutable_candidates()->set_shadow_semantic_partitions(false);
      Run(detection, semantic);
      ASSERT_GT(output.unknown_size(), 0);
      ASSERT_EQ(output.shadow_partition_size(), 0);
      auto control = output;
      control.mutable_health()->clear_latency_ms();
      config.mutable_candidates()->set_shadow_semantic_partitions(true);
      Run(detection, semantic);
      EXPECT_EQ(output.shadow_partition_size() > 0,
                qualified && semantic != nullptr);
      output.clear_shadow_partition();
      output.mutable_health()->clear_latency_ms();
      EXPECT_EQ(output.SerializeAsString(), control.SerializeAsString());
    }
  }
}

TEST_F(FusionTest, ReadOnlyViewsPreserveEvidenceAndBackgroundOccupancy) {
  Obstacle(13);
  Run(nullptr, &seg);
  const auto original = output;
  config.mutable_spatial()->set_point_views(true);
  Run(nullptr, &seg);
  ASSERT_EQ(output.point_view_size(), 6);
  EXPECT_EQ(output.point_view(STRUCTURE_POINTS).point_id_size(), 6);
  EXPECT_EQ(output.point_view(NON_GROUND_POINTS).point_id_size(), 6);
  EXPECT_EQ(output.space_map().SerializeAsString(),
            original.space_map().SerializeAsString());
  EXPECT_EQ(output.object_size(), original.object_size());
  for (int i = 0; i < output.point_evidence_size(); ++i) {
    EXPECT_EQ(output.point_evidence(i).SerializeAsString(),
              original.point_evidence(i).SerializeAsString());
  }
  EXPECT_EQ(output.point_evidence(0).ground_seed_count(), 6U);
  EXPECT_NEAR(output.point_evidence(0).ground_slope(), 0, 1e-10);
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

TEST_F(FusionTest, ConflictRetainsClassFlagRawScoreAndObservedOccupancy) {
  Obstacle(15);
  Box();
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_TRUE(output.object(0).class_conflict());
  EXPECT_EQ(output.hypothesis(0).consistency(), CONFLICT);
  EXPECT_TRUE(output.hypothesis(0).existence_supported());
  EXPECT_FALSE(output.hypothesis(0).identity_supported());
  EXPECT_FLOAT_EQ(output.hypothesis(0).raw_detection().score(), 0.8F);
  ASSERT_EQ(output.unknown_size(), 1);
  ASSERT_EQ(output.unknown(0).related_object_id_size(), 1);
  EXPECT_EQ(output.unknown(0).related_object_id(0), output.object(0).id());
  EXPECT_TRUE(std::any_of(output.space_map().cell().begin(),
                          output.space_map().cell().end(),
                          [](const auto& c) { return c.occupancy_obs(); }));
}

TEST_F(FusionTest, DisconnectedTireIsNotSwallowedByExpandedCarBox) {
  Obstacle();
  Box();
  config.set_association_margin(1);
  Add(2.9, 1.4, 0.4, 0);
  Sync();
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_EQ(output.hypothesis(0).association().n_unconnected_margin(), 1);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).point_id_size(), 1);
  EXPECT_TRUE(output.unknown(0).in_corridor());
  EXPECT_EQ(output.unknown(0).related_object_id_size(), 0);
}

TEST_F(FusionTest, WeakClassPolicyRetainsRawHypothesisAndBothOccupancyLayers) {
  Obstacle(15);
  Box(0.15F);
  Run(&det, &seg);
  const auto original = output;
  config.mutable_quality()->set_separate_weak_detections(true);
  Run(&det, &seg);
  ASSERT_EQ(original.object_size(), 1);
  EXPECT_EQ(output.object_size(), 0);
  ASSERT_EQ(output.hypothesis_size(), 1);
  EXPECT_EQ(output.hypothesis(0).decision(), RETAINED_WEAK);
  EXPECT_EQ(output.hypothesis(0).consistency(), CONFLICT);
  EXPECT_EQ(output.hypothesis(0).raw_detection().SerializeAsString(),
            det.detection(0).SerializeAsString());
  EXPECT_FALSE(output.hypothesis(0).has_object_id());
  ASSERT_GT(output.unknown_size(), 0);
  for (const auto& cell : original.space_map().cell()) {
    const auto found =
        std::find_if(output.space_map().cell().begin(),
                     output.space_map().cell().end(), [&](const auto& current) {
                       return current.x_index() == cell.x_index() &&
                              current.y_index() == cell.y_index();
                     });
    ASSERT_NE(found, output.space_map().cell().end());
    EXPECT_EQ(found->occupancy_obs(), cell.occupancy_obs());
    EXPECT_EQ(found->occupancy_det(), cell.occupancy_det());
  }
  det.mutable_detection(0)->set_score(0.8F);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).classification_use(), IDENTITY_UNCERTAIN);
}

TEST_F(FusionTest, CompatibleMarginAndTrustedViewDoNotRequireCoreAgreement) {
  Obstacle();
  Box();
  // Connected vegetation lies just outside the original detector box.
  Add(2.5, 1.4, 0.8, 15);
  Add(2.3, 1.4, 0.8, 15);
  Sync();
  config.set_connectivity_size(0.3);
  config.mutable_quality()->set_compatible_margin(true);
  config.mutable_quality()->set_trusted_point_view(true);
  config.mutable_spatial()->set_point_views(true);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_FALSE(output.point_decision(12).has_object_id());
  EXPECT_TRUE(output.point_decision(13).has_object_id());
  EXPECT_EQ(output.object(0).point_id_size(), 7);
  EXPECT_EQ(output.point_view(TRUSTED_OBJECT_POINTS).point_id_size(), 6);
  Run(&det, nullptr);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).classification_use(), CLASSIFICATION_CANDIDATE);
  EXPECT_EQ(output.point_view(TRUSTED_OBJECT_POINTS).point_id_size(), 0);
}

TEST_F(FusionTest, SemanticRecallDoesNotTreatBackgroundConnectivityAsIdentity) {
  Obstacle();
  Box();
  for (double x : {3.0, 3.1, 3.2}) {
    for (double y : {1.3, 1.4}) {
      Add(x, y, 0.8, 1);
    }
  }
  Add(2.4, 1.4, 0.8, 15);
  Add(2.6, 1.4, 0.8, 15);
  Add(2.8, 1.4, 0.8, 15);
  Sync();
  config.set_connectivity_size(0.3);
  config.mutable_quality()->set_compatible_margin(true);
  config.mutable_candidates()->set_separate_semantic_instances(true);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 2);
  EXPECT_EQ(output.object(1).source(), SEG_RECALL);
  EXPECT_EQ(output.object(1).point_id_size(), 6);
}

TEST_F(FusionTest, ConfidentForegroundRecallRetainsMixedUnknownRisk) {
  for (double x : {5.0, 5.1, 5.2}) {
    for (double y : {1.3, 1.4}) {
      Add(x, y, 0.8, 1);
    }
  }
  for (double x : {5.25, 5.3, 5.35}) {
    Add(x, 1.35, 0.8, 9);
  }
  Sync();
  Run(nullptr, &seg);
  ASSERT_EQ(output.object_size(), 0);
  ASSERT_EQ(output.unknown_size(), 1);
  const auto baseline = output;
  config.mutable_candidates()->set_recall_confident_foreground(true);
  Run(nullptr, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).source(), SEG_RECALL);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_NE(
      std::find(output.object(0).reason().begin(),
                output.object(0).reason().end(), FOREGROUND_SUBSET_RECALL),
      output.object(0).reason().end());
  ASSERT_EQ(output.unknown_size(), 1);
  ASSERT_EQ(output.unknown(0).point_id_size(),
            baseline.unknown(0).point_id_size());
  for (int i = 0; i < output.unknown(0).point_id_size(); ++i) {
    EXPECT_EQ(output.unknown(0).point_id(i), baseline.unknown(0).point_id(i));
  }
  for (int i = 0; i < output.point_evidence_size(); ++i) {
    EXPECT_EQ(output.point_evidence(i).SerializeAsString(),
              baseline.point_evidence(i).SerializeAsString());
    EXPECT_EQ(output.point_decision(i).has_object_id(), i >= 6 && i < 12);
  }
  for (const auto& cell : baseline.space_map().cell()) {
    if (!cell.occupancy_obs() && !cell.occupancy_unknown()) {
      continue;
    }
    const auto found =
        std::find_if(output.space_map().cell().begin(),
                     output.space_map().cell().end(), [&](const auto& current) {
                       return current.x_index() == cell.x_index() &&
                              current.y_index() == cell.y_index();
                     });
    ASSERT_NE(found, output.space_map().cell().end());
    EXPECT_EQ(found->occupancy_obs(), cell.occupancy_obs());
    EXPECT_EQ(found->occupancy_unknown(), cell.occupancy_unknown());
  }
  config.mutable_quality()->set_semantic_identity_unverified(true);
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_EQ(output.unknown_size(), 1);
  config.mutable_quality()->set_semantic_identity_unverified(false);
  config.set_unknown_min_points(10);
  config.set_corridor_half_width(0.1);
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_EQ(output.unknown_size(), 0);
}

TEST_F(FusionTest, ForegroundRecallCannotConnectThroughRemovedBackground) {
  for (double x : {5.05, 5.15, 5.25, 5.85, 5.95, 6.05}) {
    Add(x, 1.3, 0.8, 1);
  }
  Add(5.45, 1.3, 0.8, 9);
  Add(5.65, 1.3, 0.8, 9);
  Sync();
  config.mutable_candidates()->set_recall_confident_foreground(true);
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).point_id_size(), 8);
}

TEST_F(FusionTest, DuplicateInstancesMergeWithoutChangingRawHypotheses) {
  Obstacle();
  Box(0.8F);
  Box(0.9F);
  det.mutable_detection(1)->set_center_y(1.45);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  ASSERT_EQ(output.hypothesis_size(), 2);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_FLOAT_EQ(output.object(0).raw_score(), 0.9F);
  EXPECT_FLOAT_EQ(output.hypothesis(0).raw_detection().score(), 0.8F);
  EXPECT_FLOAT_EQ(output.hypothesis(1).raw_detection().score(), 0.9F);
  EXPECT_TRUE(output.point_decision(6).competition());
  EXPECT_EQ(output.point_decision(6).object_id(), output.object(0).id());
}

TEST_F(FusionTest, UnverifiedSemanticIdentityCannotVetoOrPromoteDetections) {
  Obstacle(15);
  Box();
  Box(0.15F);
  config.mutable_quality()->set_separate_weak_detections(true);
  config.mutable_quality()->set_semantic_identity_unverified(true);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).consistency(), INSUFFICIENT);
  EXPECT_EQ(output.object(0).classification_use(), CLASSIFICATION_CANDIDATE);
  EXPECT_EQ(output.hypothesis(1).decision(), BELOW_INSTANCE_THRESHOLD);
  for (auto& label : *seg.mutable_point_label()) {
    label.set_semantic_label(1);
  }
  Run(nullptr, &seg);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_GT(output.unknown_size(), 0);
}

TEST_F(FusionTest, SparseHighScoreBoxesSurviveWithoutSemanticSupport) {
  Box(0.8F);
  Run(&det, nullptr);
  EXPECT_EQ(output.health().mode(), NO_SEG);
  ASSERT_EQ(output.object_size(), 1);
  EXPECT_EQ(output.object(0).consistency(), INSUFFICIENT);
  EXPECT_EQ(output.object(0).point_id_size(), 0);
  bool inferred_only = false;
  for (const auto& cell : output.space_map().cell()) {
    if (cell.occupancy_det() && !cell.occupancy_obs()) {
      inferred_only = true;
      EXPECT_EQ(cell.traversability(), OCCUPIED);
    }
  }
  EXPECT_TRUE(inferred_only);
}

TEST_F(FusionTest, NearbyInstancesAreNotMergedJustBecauseTheirBoxesOverlap) {
  Obstacle();
  for (double y : {1.3, 1.4}) {
    for (double x : {2.6, 2.7, 2.8}) {
      Add(x, y, 0.8, 1);
    }
  }
  Sync();
  Box();
  Box();
  det.mutable_detection(0)->set_size_x(1.0);
  det.mutable_detection(1)->set_center_x(2.7);
  det.mutable_detection(1)->set_size_x(1.0);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 2);
  EXPECT_EQ(output.object(0).point_id_size(), 6);
  EXPECT_EQ(output.object(1).point_id_size(), 6);
  EXPECT_NE(output.point_decision(6).object_id(),
            output.point_decision(12).object_id());
}

TEST_F(FusionTest, PiecewiseGroundFitsKnownSlopeAndRefusesDegenerateSeeds) {
  for (int i = 0; i < cloud.point_size(); ++i) {
    auto* p = cloud.mutable_point(i);
    p->set_z(0.1 * p->x());
  }
  Add(2, 1.4, 1, 0);
  Sync();
  Run(nullptr, nullptr);
  ASSERT_TRUE(output.point_evidence(6).ground_valid());
  EXPECT_NEAR(output.point_evidence(6).z_ground(), 0.2, 1e-6);
  EXPECT_NEAR(output.point_evidence(6).height_above_ground(), 0.8, 1e-6);
  for (int i = 0; i < 6; ++i) {
    cloud.mutable_point(i)->set_y(0.2);
  }
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(6).ground_valid());
}

TEST_F(FusionTest, RobustGroundRetainsDominantSurfaceAndRaisedReturns) {
  Add(1.6, 0.5, 0.25, 9);
  Sync();
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  config.mutable_quality()->set_robust_ground(true);
  Run(nullptr, nullptr);
  ASSERT_TRUE(output.point_evidence(0).ground_valid());
  EXPECT_NEAR(output.point_evidence(0).z_ground(), 0, 1e-8);
  EXPECT_EQ(output.point_evidence(0).ground_seed_count(), 6U);
  EXPECT_FALSE(output.point_evidence(6).is_ground_geo());
  EXPECT_EQ(output.point_decision(6).explanation(), UNEXPLAINED);
  EXPECT_TRUE(std::any_of(
      output.space_map().cell().begin(), output.space_map().cell().end(),
      [](const auto& cell) { return cell.occupancy_obs(); }));
  Run(nullptr, &seg);
  EXPECT_NEAR(output.point_evidence(0).z_ground(), 0, 1e-8);
  EXPECT_FALSE(output.point_evidence(6).is_ground_geo());
}

TEST_F(FusionTest, RobustGroundRejectsCompetingSurfacesAndDegenerateSupport) {
  config.mutable_quality()->set_robust_ground(true);
  for (double y : {0.2, 0.8}) {
    for (double x : {1.0, 1.6, 2.2}) {
      Add(x, y, 0.25, 9);
    }
  }
  Sync();
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  cloud.clear_point();
  seg.clear_point_label();
  for (int i = 0; i < 10; ++i) {
    Add(1 + i * 0.1, 0.5, i < 8 ? 0 : 0.25, 9);
  }
  Sync();
  Run(nullptr, nullptr);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  config.mutable_quality()->set_ground_inlier_ratio(0.5);
  SingleFrameFusion fusion;
  std::string error;
  EXPECT_FALSE(fusion.Init(config, &error));
}

TEST_F(FusionTest, RobustSemanticRefinementCannotSelectRaisedMinoritySurface) {
  config.mutable_quality()->set_robust_ground(true);
  for (int i = 0; i < seg.point_label_size(); ++i) {
    seg.mutable_point_label(i)->set_semantic_label(0);
  }
  for (double y : {0.4, 0.6, 1.0}) {
    for (double x : {1.1, 1.4, 1.7, 2.0}) {
      Add(x, y, 0, 0);
    }
  }
  for (double y : {0.3, 0.7}) {
    for (double x : {1.2, 1.8, 2.4}) {
      Add(x, y, 0.25, 9);
    }
  }
  Sync();
  Run(nullptr, &seg);
  ASSERT_TRUE(output.point_evidence(0).ground_valid());
  EXPECT_NEAR(output.point_evidence(0).z_ground(), 0, 1e-8);
  EXPECT_FALSE(output.point_evidence(18).is_ground_geo());
  EXPECT_EQ(output.point_decision(18).explanation(), UNEXPLAINED);
}

TEST_F(FusionTest, InstanceRejectionDoesNotClearMeasuredOrInferredOccupancy) {
  Obstacle(0);
  Box(0.15F);
  Run(&det, &seg);
  ASSERT_EQ(output.hypothesis_size(), 1);
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_TRUE(std::any_of(output.space_map().cell().begin(),
                          output.space_map().cell().end(),
                          [](const auto& c) { return c.occupancy_obs(); }));
  EXPECT_TRUE(std::any_of(output.space_map().cell().begin(),
                          output.space_map().cell().end(),
                          [](const auto& c) { return c.occupancy_det(); }));
}

TEST_F(FusionTest, SharedVoxelGraphRecomputesConnectivityAfterOwnershipMask) {
  config.set_association_margin(0);
  config.set_core_min_points(1);
  config.set_unknown_min_points(1);
  for (double x : {1.1, 1.3, 1.5, 1.7, 1.9, 2.1}) {
    Add(x, 1.4, 0.8, 0);
  }
  Sync();
  Box();
  auto* box = det.mutable_detection(0);
  box->set_center_x(1.7);
  box->set_size_x(0.1);
  Run(&det, &seg);
  ASSERT_EQ(output.object_size(), 1);
  ASSERT_EQ(output.object(0).point_id_size(), 1);
  ASSERT_EQ(output.unknown_size(), 2);
  EXPECT_EQ(output.unknown(0).point_id_size(), 3);
  EXPECT_EQ(output.unknown(1).point_id_size(), 2);
  Add(1.78, 1.4, 0.8, 0);
  Sync();
  Run(&det, &seg);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).point_id_size(), 6);
}

TEST_F(FusionTest, VoxelAdjacencyRetainsDiagonalAndReverseConnections) {
  cloud.clear_point();
  seg.clear_point_label();
  config.set_min_x(-4);
  config.set_unknown_min_points(1);
  // The lexicographically first voxel reaches the second through the last.
  Add(-0.1, -0.1, 0.9, 0);
  Add(-0.3, 0.1, 0.7, 0);
  Add(-0.3, -0.3, 0.7, 0);
  Add(1.1, 2.1, 0.9, 0);
  Sync();
  Run(nullptr, nullptr);
  ASSERT_EQ(output.unknown_size(), 2);
  ASSERT_EQ(output.unknown(0).point_id_size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(output.unknown(0).point_id(i), i);
  }
  ASSERT_EQ(output.unknown(1).point_id_size(), 1);
  EXPECT_EQ(output.unknown(1).point_id(0), 3U);
}

TEST_F(FusionTest, LocalRelationsDoNotInheritWholeSceneReferences) {
  config.set_resolution(0.05);
  config.set_association_margin(0);
  config.set_core_min_points(1);
  config.set_unknown_min_points(1);
  config.mutable_quality()->set_point_supported_unknown(true);
  config.mutable_candidates()->set_adjacent_distance(0);
  for (double x : {1.1, 1.3, 1.5, 1.7, 1.9, 2.1}) {
    Add(x, 1.4, 0.8, 0);
  }
  Sync();
  Box();
  det.mutable_detection(0)->set_center_x(1.7);
  det.mutable_detection(0)->set_size_x(0.1);
  Run(&det, &seg);
  const auto baseline = output;
  ASSERT_EQ(output.unknown_size(), 2);
  for (const auto& unknown : output.unknown()) {
    ASSERT_EQ(unknown.related_object_id_size(), 1);
  }
  config.mutable_candidates()->set_local_object_relations(true);
  Run(&det, &seg);
  ASSERT_EQ(output.unknown_size(), 2);
  for (const auto& unknown : output.unknown()) {
    EXPECT_EQ(unknown.related_object_id_size(), 0);
    EXPECT_EQ(unknown.relation_size(), 0);
  }
  EXPECT_EQ(output.space_map().SerializeAsString(),
            baseline.space_map().SerializeAsString());
  EXPECT_EQ(output.object(0).SerializeAsString(),
            baseline.object(0).SerializeAsString());
  for (int i = 0; i < output.point_evidence_size(); ++i) {
    EXPECT_EQ(output.point_evidence(i).SerializeAsString(),
              baseline.point_evidence(i).SerializeAsString());
  }
}

TEST_F(FusionTest, UnsupportedGroundIsConservativeAndNeverAssumedZero) {
  cloud.clear_point();
  seg.clear_point_label();
  Add(2, 1, 0.8, 9);
  Sync();
  Run(nullptr, &seg);
  EXPECT_FALSE(output.point_evidence(0).ground_valid());
  EXPECT_FALSE(output.point_evidence(0).has_z_ground());
  ASSERT_EQ(output.unknown_size(), 1);
  int observed = 0;
  for (const auto& cell : output.space_map().cell()) {
    observed += cell.occupancy_obs();
    EXPECT_TRUE(cell.occupancy_unknown());
    EXPECT_FALSE(cell.occupancy_det());
    EXPECT_EQ(cell.traversability(), OCCUPIED);
  }
  EXPECT_EQ(observed, 1);
}

TEST_F(FusionTest, RaisedSemanticRoadIsNotGroundOrFree) {
  Add(2, 1.4, 0.8, 9);
  Sync();
  Run(nullptr, &seg);
  const auto& p = output.point_evidence(6);
  ASSERT_TRUE(p.ground_valid());
  EXPECT_NEAR(p.height_above_ground(), 0.8, 1e-6);
  EXPECT_FALSE(p.is_ground_geo());
  EXPECT_EQ(output.point_decision(6).explanation(), UNEXPLAINED);
  ASSERT_EQ(output.unknown_size(), 1);
  EXPECT_EQ(output.unknown(0).reason(0), SEM_GEO_CONFLICT);
}

TEST_F(FusionTest, NearGroundThingKeepsEvidenceWithoutInventingOccupancy) {
  config.mutable_spatial()->set_point_views(true);
  const int index = cloud.point_size();
  Add(2, 1.4, 0.125, 1);
  Sync();
  Run(nullptr, &seg);
  const auto& point = output.point_evidence(index);
  ASSERT_TRUE(point.ground_valid());
  EXPECT_FALSE(point.is_ground_geo());
  EXPECT_NEAR(point.height_above_ground(), 0.125, 1e-6);
  EXPECT_EQ(point.unified_class(), CAR);
  EXPECT_EQ(output.point_decision(index).explanation(), UNEXPLAINED);
  EXPECT_FALSE(output.point_decision(index).has_object_id());
  EXPECT_EQ(output.object_size(), 0);
  EXPECT_EQ(output.unknown_size(), 0);
  const uint32_t x =
      std::floor((point.position().x() - config.min_x()) / config.resolution());
  const uint32_t y =
      std::floor((point.position().y() - config.min_y()) / config.resolution());
  const auto& cells = output.space_map().cell();
  const auto found =
      std::find_if(cells.begin(), cells.end(), [&](const SpaceCell& cell) {
        return cell.x_index() == x && cell.y_index() == y;
      });
  ASSERT_NE(found, cells.end());
  EXPECT_TRUE(found->endpoint_observed());
  EXPECT_FALSE(found->occupancy_obs());
  EXPECT_FALSE(found->occupancy_det());
  EXPECT_FALSE(found->occupancy_unknown());
  EXPECT_EQ(found->traversability(), UNKNOWN);
  const auto& risk = output.point_view(CORRIDOR_RISK_POINTS).point_id();
  EXPECT_NE(std::find(risk.begin(), risk.end(), index), risk.end());
}

TEST_F(FusionTest, NearGroundEndpointCannotEraseSameCellMeasuredObstacle) {
  Add(2, 1.4, 0, 9);
  Sync();
  const auto ground_cloud = cloud;
  const auto ground_seg = seg;
  std::string space;
  for (bool obstacle_first : {true, false}) {
    cloud = ground_cloud;
    seg = ground_seg;
    for (double height :
         {obstacle_first ? 0.8 : 0.0625, obstacle_first ? 0.0625 : 0.8}) {
      Add(2, 1.4, height, 1);
    }
    Sync();
    Run(nullptr, &seg);
    const int low = ground_cloud.point_size() + (obstacle_first ? 1 : 0);
    EXPECT_TRUE(output.point_evidence(low).is_ground_geo());
    int observed = 0;
    for (const auto& cell : output.space_map().cell()) {
      if (cell.occupancy_obs()) {
        ++observed;
        EXPECT_TRUE(cell.ground_supported());
        EXPECT_EQ(cell.traversability(), OCCUPIED);
        EXPECT_FALSE(cell.reachable());
        EXPECT_FALSE(cell.physically_traversable());
      }
    }
    EXPECT_EQ(observed, 1);
    if (obstacle_first) {
      space = output.space_map().SerializeAsString();
    } else {
      EXPECT_EQ(output.space_map().SerializeAsString(), space);
    }
  }
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

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo
