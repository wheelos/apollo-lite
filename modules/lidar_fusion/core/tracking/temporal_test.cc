// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/lidar_fusion/core/tracking/temporal.h"

#include <cmath>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace lidar_fusion {
namespace {

class TemporalTest : public testing::Test {
 protected:
  void SetUp() override {
    c.set_pointcloud_channel("/cloud");
    c.set_detection_channel("/det");
    c.set_segmentation_channel("/seg");
    c.set_output_channel("/fused");
    c.set_sensor_frame("lidar");
    c.set_base_frame("base");
    c.set_min_x(0);
    c.set_max_x(12);
    c.set_min_y(-4);
    c.set_max_y(4);
    c.mutable_sensor_to_base()->set_version("test");
    c.mutable_sensor_to_base()->set_x(0);
    c.mutable_sensor_to_base()->set_y(0);
    c.mutable_sensor_to_base()->set_z(0);
    c.mutable_sensor_to_base()->set_roll(0);
    c.mutable_sensor_to_base()->set_pitch(0);
    c.mutable_sensor_to_base()->set_yaw(0);
    c.set_detection_backend(
        lidar_object_detection::LIDAR_OBJECT_DETECTION_BACKEND_POINTPILLARS);
    auto* mapping = c.add_detection_mapping();
    mapping->set_raw_id(0);
    mapping->set_group(THING);
    mapping->set_unified_class(CAR);
    mapping = c.add_semantic_mapping();
    mapping->set_raw_id(0);
    mapping->set_group(GENERIC_OBS);
    mapping = c.add_semantic_mapping();
    mapping->set_raw_id(1);
    mapping->set_group(STATIC_STRUCT);
    c.mutable_temporal()->set_enabled(true);
    c.mutable_temporal()->set_odom_frame("odom");
    c.mutable_temporal()->set_confirmation_hits(2);
    c.mutable_temporal()->set_feedback(true);
    c.mutable_temporal()->set_static_map(true);
    c.mutable_temporal()->set_conflict_frames(2);
    c.mutable_temporal()->set_max_dt(0.5);
    pose.set_valid(true);
    pose.set_epoch(1);
    pose.set_odom_frame("odom");
    pose.set_base_frame("base");
    pose.mutable_translation()->set_x(0);
    cloud.set_frame_id("lidar");
    cloud.mutable_header()->set_frame_id("lidar");
    for (double y : {0.2, 0.8}) {
      for (double x : {1., 1.5, 2.}) {
        Point(x, y, 0);
      }
    }
    for (double x : {1.8, 1.9, 2.0}) {
      for (double y : {1.2, 1.3}) {
        Point(x, y, 0.8);
      }
    }
    Next(100);
    auto* box = det.add_detection();
    box->set_center_x(1.9);
    box->set_center_y(1.25);
    box->set_center_z(0.8);
    box->set_size_x(0.8);
    box->set_size_y(0.6);
    box->set_size_z(0.8);
    box->set_yaw(0);
    box->set_class_id(0);
    box->set_score(0.9);
  }
  void Point(double x, double y, double z) {
    auto* p = cloud.add_point();
    p->set_x(x);
    p->set_y(y);
    p->set_z(z);
  }
  void Next(double stamp) {
    cloud.set_measurement_time(stamp);
    cloud.mutable_header()->set_timestamp_sec(stamp);
    cloud.mutable_header()->set_sequence_num(++sequence);
    pose.set_stamp(stamp);
    *det.mutable_header() = cloud.header();
    det.set_backend(c.detection_backend());
    det.set_model_revision("test");
    det.set_sensor_frame_id("lidar");
    det.set_input_point_count(cloud.point_size());
  }
  void Init() {
    std::string error;
    ASSERT_TRUE(fusion.Init(c, &error)) << error;
  }
  void Run(bool detection = true, bool with_pose = true) {
    std::string error;
    ASSERT_TRUE(fusion.Process(cloud, detection ? &det : nullptr, nullptr,
                               with_pose ? &pose : nullptr, &out, &error))
        << error;
  }
  FusionConfig c;
  StatefulFusion fusion;
  drivers::PointCloud cloud;
  DetectionFrame det;
  LocalPose pose;
  FusionOutput out;
  uint32_t sequence = 0;
};

TEST_F(TemporalTest, StableIdRecoveryNeedsActualCurrentPoints) {
  Init();
  Run();
  ASSERT_EQ(out.object_size(), 1);
  const auto id = out.object(0).track_id();
  EXPECT_GT(id, 0U);
  EXPECT_FALSE(out.object(0).track_confirmed());
  Next(100.1);
  Run();
  ASSERT_EQ(out.object_size(), 1);
  EXPECT_EQ(out.object(0).track_id(), id);
  EXPECT_TRUE(out.object(0).track_confirmed());
  Next(100.2);
  Run(false);
  ASSERT_EQ(out.object_size(), 1);
  EXPECT_EQ(out.object(0).source(), TRACK_PRIOR);
  EXPECT_EQ(out.object(0).track_id(), id);
  EXPECT_FALSE(out.object(0).has_raw_score());
  EXPECT_TRUE(out.track(0).observed());
  cloud.mutable_point()->DeleteSubrange(6, 6);
  Next(100.3);
  Run(false);
  EXPECT_EQ(out.object_size(), 0);
  ASSERT_EQ(out.track_size(), 1);
  EXPECT_EQ(out.track(0).id(), id);
  EXPECT_FALSE(out.track(0).observed());
  for (const auto& cell : out.space_map().cell()) {
    EXPECT_FALSE(cell.occupancy_det());
  }
}

TEST_F(TemporalTest,
       SmallerCurrentDetectionDoesNotRecoverDuplicateOuterSupport) {
  det.mutable_detection(0)->set_size_x(3.0);
  for (double x : {3.0, 3.1, 3.2}) {
    for (double y : {1.2, 1.3}) {
      Point(x, y, 0.8);
    }
  }
  Next(100);
  Init();
  Run();
  ASSERT_EQ(out.object_size(), 1);
  const auto id = out.object(0).track_id();
  Next(100.1);
  Run();
  ASSERT_TRUE(out.object(0).track_confirmed());
  det.mutable_detection(0)->set_size_x(0.8);
  Next(100.2);
  Run();
  ASSERT_EQ(out.object_size(), 1);
  EXPECT_EQ(out.object(0).source(), DET);
  EXPECT_EQ(out.object(0).track_id(), id);
  uint32_t object_tracks = 0;
  for (const auto& track : out.track()) {
    if (!track.unknown()) {
      ++object_tracks;
      EXPECT_EQ(track.id(), id);
    }
  }
  EXPECT_EQ(object_tracks, 1U);
  // Outer current returns remain measured evidence, not a second prior object.
  for (int i = 12; i < cloud.point_size(); ++i) {
    EXPECT_FALSE(out.point_decision(i).has_object_id());
  }
}

TEST_F(TemporalTest, MissingStaleEpochRewindAndGapResetWithoutReusingIds) {
  Init();
  Run();
  uint64_t id = out.object(0).track_id();
  Next(100.1);
  Run(true, false);
  EXPECT_FALSE(out.health().temporal_enabled());
  EXPECT_EQ(out.track_size(), 0);
  Next(100.2);
  Run();
  EXPECT_GT(out.object(0).track_id(), id);
  id = out.object(0).track_id();
  Next(100.3);
  pose.set_stamp(100);
  Run();
  EXPECT_FALSE(out.health().pose_valid());
  Next(100.4);
  Run();
  EXPECT_GT(out.object(0).track_id(), id);
  id = out.object(0).track_id();
  Next(100.5);
  pose.set_epoch(2);
  Run();
  EXPECT_GT(out.object(0).track_id(), id);
  id = out.object(0).track_id();
  Next(99);
  Run();
  EXPECT_GT(out.object(0).track_id(), id);
  id = out.object(0).track_id();
  Next(110);
  Run();
  EXPECT_GT(out.object(0).track_id(), id);
}

TEST_F(TemporalTest, EgoTranslationDoesNotCreateObjectVelocity) {
  Init();
  Run();
  const auto id = out.object(0).track_id();
  for (int step = 1; step <= 5; ++step) {
    Next(100 + step * 0.1);
    pose.mutable_translation()->set_x(step * 0.1);
    det.mutable_detection(0)->set_center_x(1.9 - step * 0.1);
    for (int i = 0; i < cloud.point_size(); ++i) {
      cloud.mutable_point(i)->set_x(cloud.point(i).x() - 0.1);
    }
    Run();
    ASSERT_EQ(out.track_size(), 1);
    EXPECT_EQ(out.track(0).id(), id);
    EXPECT_NEAR(out.track(0).velocity().x(), 0, 1e-5);
    EXPECT_NEAR(out.track(0).box().center().x(), 1.9 - step * 0.1, 1e-5);
  }
}

TEST_F(TemporalTest, SameEpochPoseDiscontinuitiesResetRatherThanInventMotion) {
  Init();
  Run();
  auto id = out.object(0).track_id();
  Next(100.1);
  pose.mutable_translation()->set_x(6);
  Run();
  ASSERT_EQ(out.object_size(), 1);
  EXPECT_GT(out.object(0).track_id(), id);
  EXPECT_NEAR(out.track(0).velocity().x(), 0, 1e-8);
  id = out.object(0).track_id();
  Next(100.2);
  pose.set_qz(std::sin(0.3));
  pose.set_qw(std::cos(0.3));
  Run();
  EXPECT_GT(out.object(0).track_id(), id);
  EXPECT_NEAR(out.track(0).velocity().y(), 0, 1e-8);
}

TEST_F(TemporalTest, EgoRotationDoesNotCreatePhysicalObjectVelocity) {
  Init();
  Run();
  const auto id = out.object(0).track_id();
  for (int step = 1; step <= 4; ++step) {
    const double yaw = step * 0.05;
    Next(100 + step * 0.1);
    pose.set_qz(std::sin(yaw / 2));
    pose.set_qw(std::cos(yaw / 2));
    auto* box = det.mutable_detection(0);
    box->set_center_x(std::cos(yaw) * 1.9 + std::sin(yaw) * 1.25);
    box->set_center_y(-std::sin(yaw) * 1.9 + std::cos(yaw) * 1.25);
    box->set_yaw(-yaw);
    for (auto& point : *cloud.mutable_point()) {
      const double x = point.x(), y = point.y();
      point.set_x(std::cos(0.05) * x + std::sin(0.05) * y);
      point.set_y(-std::sin(0.05) * x + std::cos(0.05) * y);
    }
    Run();
    ASSERT_EQ(out.object_size(), 1);
    EXPECT_EQ(out.object(0).track_id(), id);
    bool found = false;
    for (const auto& track : out.track()) {
      if (track.id() == id) {
        found = true;
        EXPECT_NEAR(track.velocity().x(), 0, 1e-5);
        EXPECT_NEAR(track.velocity().y(), 0, 1e-5);
      }
    }
    EXPECT_TRUE(found);
  }
}

TEST_F(TemporalTest,
       UnknownRiskIsImmediateButStableInstanceRequiresConfirmation) {
  Init();
  Run(false);
  ASSERT_EQ(out.unknown_size(), 1);
  const auto id = out.unknown(0).track_id();
  EXPECT_FALSE(out.unknown(0).confirmed());
  bool risk = false;
  for (const auto& cell : out.space_map().cell()) {
    risk |= cell.occupancy_unknown();
  }
  EXPECT_TRUE(risk);
  Next(100.1);
  Run(false);
  ASSERT_EQ(out.unknown_size(), 1);
  EXPECT_EQ(out.unknown(0).track_id(), id);
  EXPECT_TRUE(out.unknown(0).confirmed());
  ASSERT_EQ(out.track_size(), 1);
  EXPECT_EQ(out.track(0).source(), PROMOTED_UNKNOWN);
}

TEST_F(TemporalTest, ClassConflictChangesOnlyTemporalIdentityAfterPersistence) {
  Init();
  Run();
  Next(100.1);
  Run();
  EXPECT_EQ(out.object(0).temporal_class(), CAR);
  SemanticFrame seg;
  auto process_conflict = [&]() {
    *seg.mutable_header() = cloud.header();
    seg.set_source_frame_id("lidar");
    seg.set_source_topic(c.pointcloud_channel());
    seg.set_point_count(cloud.point_size());
    seg.clear_point_label();
    for (int i = 0; i < cloud.point_size(); ++i) {
      auto* p = seg.add_point_label();
      p->set_point_index(i);
      p->set_semantic_label(1);
      p->set_confidence(0.95);
    }
    std::string error;
    ASSERT_TRUE(fusion.Process(cloud, &det, &seg, &pose, &out, &error))
        << error;
  };
  Next(100.2);
  process_conflict();
  ASSERT_EQ(out.object_size(), 1);
  EXPECT_TRUE(out.object(0).class_conflict());
  EXPECT_EQ(out.object(0).temporal_class(), CAR);
  Next(100.3);
  process_conflict();
  ASSERT_EQ(out.object_size(), 1);
  EXPECT_EQ(out.object(0).unified_class(), CAR);
  EXPECT_EQ(out.object(0).temporal_class(), CLASS_UNKNOWN);
  EXPECT_TRUE(out.object(0).class_conflict());
}

TEST_F(TemporalTest,
       ConflictConfirmationCannotRestoreIdentityOnGeometryFeedback) {
  Init();
  SemanticFrame seg;
  for (int step = 0; step < 2; ++step) {
    Next(100 + step * 0.1);
    *seg.mutable_header() = cloud.header();
    seg.set_source_frame_id("lidar");
    seg.set_source_topic(c.pointcloud_channel());
    seg.set_point_count(cloud.point_size());
    seg.clear_point_label();
    for (int i = 0; i < cloud.point_size(); ++i) {
      auto* point = seg.add_point_label();
      point->set_point_index(i);
      point->set_semantic_label(1);
      point->set_confidence(0.95);
    }
    std::string error;
    ASSERT_TRUE(fusion.Process(cloud, &det, &seg, &pose, &out, &error))
        << error;
    ASSERT_EQ(out.object_size(), 1);
    EXPECT_TRUE(out.object(0).class_conflict());
    EXPECT_EQ(out.object(0).classification_use(), IDENTITY_UNCERTAIN);
    EXPECT_EQ(out.object(0).track_confirmed(), step == 1);
  }
  const auto id = out.object(0).track_id();
  for (int step = 2; step < 4; ++step) {
    pose.mutable_translation()->set_x((step - 1) * 0.1);
    det.mutable_detection(0)->set_center_x(1.9 - (step - 1) * 0.1);
    for (auto& point : *cloud.mutable_point()) {
      point.set_x(point.x() - 0.1);
    }
    Next(100 + step * 0.1);
    Run(false);
    ASSERT_EQ(out.object_size(), 1);
    const auto& object = out.object(0);
    EXPECT_EQ(object.track_id(), id);
    EXPECT_EQ(object.source(), TRACK_PRIOR);
    EXPECT_TRUE(object.track_confirmed());
    EXPECT_TRUE(object.class_conflict());
    EXPECT_EQ(object.classification_use(), IDENTITY_UNCERTAIN);
    EXPECT_EQ(object.unified_class(), CLASS_UNKNOWN);
    EXPECT_EQ(object.temporal_class(), CLASS_UNKNOWN);
    EXPECT_FALSE(object.has_raw_score());
    bool conflict_risk = false;
    for (auto reason : out.risk_event()) {
      conflict_risk |= reason == DET_CLASS_CONFLICT;
    }
    EXPECT_TRUE(conflict_risk);
  }
  Next(100.4);
  Run();
  ASSERT_EQ(out.object_size(), 1);
  EXPECT_EQ(out.object(0).track_id(), id);
  EXPECT_EQ(out.object(0).source(), DET);
  EXPECT_FALSE(out.object(0).class_conflict());
  EXPECT_EQ(out.object(0).classification_use(), CLASSIFICATION_CANDIDATE);
  EXPECT_EQ(out.object(0).temporal_class(), CAR);
  EXPECT_DOUBLE_EQ(out.object(0).raw_score(), det.detection(0).score());
}

TEST_F(TemporalTest, PointSupportedUnknownPreservesUnsampledGroundHistory) {
  c.mutable_quality()->set_point_supported_unknown(true);
  cloud.clear_point();
  auto ground = [&]() {
    for (double x : {1.125, 1.625, 2.125, 2.625, 3.125}) {
      for (double y : {-0.875, -0.375, 0.125, 0.625, 1.125}) {
        Point(x, y, 0);
      }
    }
  };
  auto unknown = [&](double shift) {
    for (int i = 0; i <= 8; ++i) {
      Point(1.125 + shift, -0.875 + i * 0.25, 0.8);
      Point(3.125 + shift, -0.875 + i * 0.25, 0.8);
      if (i > 0 && i < 8) {
        Point(1.125 + shift + i * 0.25, -0.875, 0.8);
      }
    }
  };
  auto ground_history = [&](double x, double y) {
    for (const auto& cell : out.history_cell()) {
      if (cell.x_index() == std::floor(x / c.resolution()) &&
          cell.y_index() == std::floor(y / c.resolution())) {
        return cell.has_ground_z();
      }
    }
    return false;
  };
  ground();
  Next(100);
  Init();
  Run(false);
  ASSERT_TRUE(ground_history(2.125, 0.625));
  ASSERT_TRUE(ground_history(1.125, 0.125));
  uint64_t id = 0;
  for (int step = 1; step <= 3; ++step) {
    cloud.clear_point();
    ground();
    unknown(step == 3 ? 0.25 : 0);
    Next(100 + step * 0.1);
    Run(false);
    ASSERT_EQ(out.unknown_size(), 1);
    EXPECT_TRUE(out.unknown(0).point_supported_footprint());
    if (step == 1) {
      id = out.unknown(0).track_id();
    }
    EXPECT_EQ(out.unknown(0).track_id(), id);
    EXPECT_EQ(out.unknown(0).confirmed(), step >= 2);
    EXPECT_TRUE(ground_history(2.125, 0.625));
    // At step 3 the old support is ground-only, but its previous footprint
    // must still be excluded alongside the new actual support.
    EXPECT_FALSE(ground_history(1.125, 0.125));
    EXPECT_FALSE(ground_history(step == 3 ? 1.375 : 1.125, 0.125));
    EXPECT_FALSE(ground_history(3.125, -0.875));
  }
  cloud.clear_point();
  ground();
  for (double stamp : {100.4, 100.8, 101.2}) {
    Next(stamp);
    Run(false);
    ASSERT_EQ(out.unknown_size(), 0);
    ASSERT_EQ(out.track_size(), 1);
    EXPECT_FALSE(out.track(0).observed());
    EXPECT_TRUE(ground_history(2.125, 0.625));
    EXPECT_FALSE(ground_history(3.125, -0.875));
  }
  Next(101.4);
  Run(false);
  EXPECT_EQ(out.track_size(), 0);
  EXPECT_TRUE(ground_history(3.125, -0.875));
}

TEST_F(TemporalTest,
       StaticHistoryIsMeasuredBoundedExpiredAndNotDetectionInference) {
  c.mutable_temporal()->set_max_history_cells(2);
  c.mutable_temporal()->set_history_seconds(0.15);
  Init();
  Run();
  EXPECT_LE(out.history_cell_size(), 2);
  for (const auto& cell : out.history_cell()) {
    EXPECT_DOUBLE_EQ(cell.log_odds(), 0);
  }
  Next(100.1);
  Run();
  EXPECT_LE(out.history_cell_size(), 2);
  cloud.clear_point();
  Point(8, 3, 1);
  Next(100.3);
  det.clear_detection();
  Run();
  ASSERT_EQ(out.history_cell_size(), 1);
  EXPECT_NEAR(out.history_cell(0).last_observed_stamp(), 100.3, 1e-10);
  EXPECT_GT(out.history_cell(0).log_odds(), 0);
}

TEST_F(TemporalTest, ResourceFailureReturnsCurrentEvidenceButClearsHistory) {
  c.mutable_temporal()->set_max_tracks(1);
  c.mutable_temporal()->set_confirmation_hits(1);
  Init();
  Run();
  det.mutable_detection(0)->set_center_x(9);
  Next(100.1);
  Run();
  EXPECT_EQ(out.object_size(), 2);
  EXPECT_EQ(out.track_size(), 0);
  EXPECT_FALSE(out.health().temporal_enabled());
  EXPECT_EQ(out.history_cell_size(), 0);
  for (const auto& object : out.object()) {
    EXPECT_FALSE(object.has_track_id());
  }
}

TEST_F(TemporalTest, MovingTracksErasePreviousStaticMeasuredOccupancy) {
  Init();
  Run();
  Next(100.1);
  Run();
  bool measured = false;
  for (const auto& cell : out.history_cell()) {
    measured |= cell.log_odds() > 0;
  }
  ASSERT_TRUE(measured);
  Next(100.2);
  det.mutable_detection(0)->set_center_x(2.9);
  for (int i = 6; i < cloud.point_size(); ++i) {
    cloud.mutable_point(i)->set_x(cloud.point(i).x() + 1);
  }
  Run();
  for (const auto& cell : out.history_cell()) {
    EXPECT_LE(cell.log_odds(), 0)
        << cell.DebugString() << out.track(0).DebugString();
  }
}

}  // namespace
}  // namespace lidar_fusion
}  // namespace apollo
