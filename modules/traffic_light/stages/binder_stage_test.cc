// Copyright 2026 The Apollo Authors. All Rights Reserved.
#include "modules/traffic_light/stages/binder_stage.h"

#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace traffic_light {
namespace {

class BinderStageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    context_.timestamp = 10000000000ULL;
    CameraFrameState camera;
    camera.camera_name = "front";
    camera.timestamp_ns = context_.timestamp;
    camera.image.cols = 1920;
    camera.image.rows = 1080;
    context_.camera_frames.push_back(camera);
    context_.primary_camera_name = camera.camera_name;
    context_.lane_context.measurement_time_sec = 9.5;
    context_.lane_context.valid_until_sec = 10.5;
    LaneCandidate lane;
    lane.lane_id = "lane";
    lane.approach_id = "approach";
    lane.movement_mask = kMovementMaskStraight;
    lane.confidence = 0.95f;
    context_.lane_context.candidates.push_back(lane);

    SignalCandidate signal;
    signal.signal_id = "signal";
    signal.lane_id = lane.lane_id;
    signal.approach_id = lane.approach_id;
    signal.signal_group_id = "control_group";
    signal.camera_name = camera.camera_name;
    signal.movement_mask = kMovementMaskStraight;
    signal.confidence = 0.9f;
    signal.topology_confidence = 0.95f;
    signal.projection_roi = Rect2f{100.0f, 100.0f, 20.0f, 40.0f};
    context_.map_signals.push_back(signal);

    TrackedLight track;
    track.track_id = 7;
    track.confirmed = true;
    track.last_visible_timestamp_sec = 10.0;
    track.current_state.visual_light.track_id = 7;
    track.current_state.visual_light.bbox = signal.projection_roi;
    track.current_state.visual_light.shape = LightShape::ARROW_STRAIGHT;
    track.current_state.visual_light.color = LightColor::RED;
    track.current_state.visual_light.existence_confidence = 0.9f;
    track.current_state.visual_light.camera_name = camera.camera_name;
    context_.tracked_lights.push_back(track);
  }

  const BoundLight& Bind() {
    EXPECT_TRUE(stage_.Process(&context_));
    return context_.tracked_lights.front().current_state;
  }

  PipelineContext context_;
  BinderStage stage_{BinderOptions()};
};

TEST_F(BinderStageTest, SemanticBindingSurvivesUnreliableAndWrongProjection) {
  auto& signal = context_.map_signals.front();
  signal.projection_roi = Rect2f{1500.0f, 800.0f, 5.0f, 5.0f};
  const auto& bound = Bind();
  EXPECT_TRUE(bound.binding_valid);
  EXPECT_TRUE(bound.controlling_ego_lane);
  EXPECT_EQ(bound.visual_light.signal_id, "signal");
  EXPECT_EQ(context_.tracked_lights.front().track_id, 7);
  EXPECT_EQ(bound.visual_light.track_id, 7);
  ASSERT_EQ(context_.bound_lights.size(), 1u);
  // Even a claimed reliable projection cannot veto independent lane semantics.
  signal.projection_reliable = true;
  EXPECT_TRUE(Bind().binding_valid);
}

TEST_F(BinderStageTest, NavigationDesireAndPromptIdDoNotProveLaneIdentity) {
  context_.lane_context = LaneContext();
  context_.nav_topology.ego_lane_intent = LaneIntent::STRAIGHT;
  context_.tracked_lights.front().current_state.visual_light.signal_id =
      "signal";
  const auto& unbound = Bind();
  EXPECT_FALSE(unbound.binding_valid);
  EXPECT_FALSE(unbound.controlling_ego_lane);
  EXPECT_TRUE(unbound.visual_light.signal_id.empty());
  EXPECT_EQ(context_.tracked_lights.size(), 1u);
  EXPECT_TRUE(context_.bound_lights.empty());

  context_.map_signals.front().projection_reliable = true;
  const auto& projected = Bind();
  EXPECT_TRUE(projected.binding_valid);
  EXPECT_FALSE(projected.controlling_ego_lane);
}

TEST_F(BinderStageTest, ExpiredAndFutureLaneContextsDoNotAuthorizeControl) {
  context_.lane_context.valid_until_sec = 9.9;
  EXPECT_FALSE(Bind().binding_valid);
  context_.lane_context.measurement_time_sec = 10.1;
  context_.lane_context.valid_until_sec = 11.0;
  EXPECT_FALSE(Bind().binding_valid);
  context_.map_signals.front().projection_reliable = true;
  EXPECT_TRUE(Bind().binding_valid);
  EXPECT_FALSE(Bind().controlling_ego_lane);
}

TEST_F(BinderStageTest, PlausibleAlternativeLaneRemainsAmbiguous) {
  auto lane = context_.lane_context.candidates.front();
  lane.lane_id = "alternative_lane";
  lane.confidence = 0.92f;
  context_.lane_context.candidates.push_back(lane);
  const auto& bound = Bind();
  EXPECT_TRUE(bound.binding_ambiguous);
  EXPECT_FALSE(bound.binding_valid);
  EXPECT_FALSE(bound.controlling_ego_lane);
  EXPECT_EQ(context_.tracked_lights.size(), 1u);
}

TEST_F(BinderStageTest, WeakLaneEvidenceCannotAuthorizeControl) {
  context_.lane_context.candidates.front().confidence = 0.1f;
  EXPECT_FALSE(Bind().binding_valid);
  context_.map_signals.front().projection_reliable = true;
  EXPECT_TRUE(Bind().binding_valid);
  EXPECT_FALSE(Bind().controlling_ego_lane);
}

TEST_F(BinderStageTest, LaneAndApproachMustAgreeWithTopology) {
  context_.map_signals.front().approach_id = "opposite_approach";
  EXPECT_FALSE(Bind().binding_valid);
  context_.map_signals.front().approach_id = "approach";
  context_.map_signals.front().lane_id = "another_lane";
  EXPECT_FALSE(Bind().binding_valid);
}

TEST_F(BinderStageTest,
       ReliableArrowConflictRejectsCombinedSignalForWrongLane) {
  context_.tracked_lights.front().current_state.visual_light.shape =
      LightShape::ARROW_LEFT;
  context_.map_signals.front().movement_mask =
      kMovementMaskStraight | kMovementMaskLeft;
  EXPECT_FALSE(Bind().binding_valid);
  context_.lane_context.candidates.front().movement_mask = kMovementMaskLeft;
  context_.map_signals.front().movement_mask = kMovementMaskStraight;
  EXPECT_FALSE(Bind().binding_valid);
}

TEST_F(BinderStageTest,
       UnknownAndCircleDoNotProvideAffirmativeMovementEvidence) {
  auto& visual = context_.tracked_lights.front().current_state.visual_light;
  visual.shape = LightShape::UNKNOWN;
  visual.intended_movement = LaneIntent::STRAIGHT;
  EXPECT_TRUE(Bind().binding_valid);
  EXPECT_FLOAT_EQ(Bind().movement_match_score, 0.0f);
  visual.shape = LightShape::CIRCLE;
  visual.movement_mask = kMovementMaskStraight | kMovementMaskLeft |
                         kMovementMaskRight | kMovementMaskUTurn;
  EXPECT_FLOAT_EQ(Bind().movement_match_score, 0.0f);
}

TEST_F(BinderStageTest, PromptSignalIdDoesNotBreakNearEqualCandidateTie) {
  auto alternative = context_.map_signals.front();
  alternative.signal_id = "other_signal";
  alternative.signal_group_id = "other_control_group";
  context_.map_signals.push_back(alternative);
  auto& visual = context_.tracked_lights.front().current_state.visual_light;
  visual.signal_id = "signal";
  visual.prompt_source = PromptSource::MAP;
  const auto& bound = Bind();
  EXPECT_TRUE(bound.binding_ambiguous);
  EXPECT_FALSE(bound.binding_valid);
  EXPECT_TRUE(context_.bound_lights.empty());
}

TEST_F(BinderStageTest, GroupCountMismatchIsSoftAndColorsCannotChangeBinding) {
  auto& visual = context_.tracked_lights.front().current_state.visual_light;
  visual.observed_group_id = "observed_cluster";
  visual.observed_group_size = 3;
  context_.map_signals.front().expected_group_size = 8;
  const auto first = Bind();
  EXPECT_TRUE(first.binding_valid);
  visual.color = LightColor::GREEN;
  const auto& second = Bind();
  EXPECT_TRUE(second.binding_valid);
  EXPECT_FLOAT_EQ(first.bind_score, second.bind_score);
  EXPECT_EQ(first.signal_group_id, second.signal_group_id);
}

TEST_F(BinderStageTest, ClearObservedCardinalitySelectsWithoutProjection) {
  auto alternative = context_.map_signals.front();
  alternative.signal_id = "many_heads";
  alternative.signal_group_id = "large_group";
  alternative.expected_group_size = 8;
  context_.map_signals.push_back(alternative);
  context_.map_signals.front().expected_group_size = 2;
  context_.tracked_lights.front()
      .current_state.visual_light.observed_group_size = 2;
  const auto& bound = Bind();
  EXPECT_TRUE(bound.binding_valid);
  EXPECT_FALSE(bound.binding_ambiguous);
  EXPECT_EQ(bound.visual_light.signal_id, "signal");
}

TEST_F(BinderStageTest, ExactObservedArrowPrefersSpecificMovement) {
  auto alternative = context_.map_signals.front();
  alternative.signal_id = "combined_signal";
  alternative.movement_mask = kMovementMaskStraight | kMovementMaskLeft;
  context_.map_signals.push_back(alternative);
  const auto& bound = Bind();
  EXPECT_TRUE(bound.binding_valid);
  EXPECT_FALSE(bound.binding_ambiguous);
  EXPECT_EQ(bound.visual_light.signal_id, "signal");
}

TEST_F(BinderStageTest, GroupOnlyIdentityCannotBecomePublishableBinding) {
  context_.map_signals.front().signal_id.clear();
  const auto& bound = Bind();
  EXPECT_FALSE(bound.binding_valid);
  EXPECT_FALSE(bound.controlling_ego_lane);
  EXPECT_TRUE(context_.bound_lights.empty());
  EXPECT_NE(context_.status.degrade_reason.find("missing external identity"),
            std::string::npos);
}

TEST_F(BinderStageTest,
       PredictedAndUnavailableTargetsCannotAcquireFreshBinding) {
  auto& track = context_.tracked_lights.front();
  track.lost_frames = 1;
  EXPECT_FALSE(Bind().binding_valid);
  EXPECT_FALSE(Bind().controlling_ego_lane);
  EXPECT_EQ(context_.tracked_lights.size(), 1u);
  EXPECT_NE(
      context_.status.degrade_reason.find("not observed in current frame"),
      std::string::npos);
  track.lost_frames = 0;
  track.last_visible_timestamp_sec = 9.9;
  EXPECT_FALSE(Bind().binding_valid);
  track.last_visible_timestamp_sec = 0.0;
  EXPECT_FALSE(Bind().binding_valid);
}

TEST_F(BinderStageTest, AmbiguousRelationSurfacesContextDegradation) {
  auto alternative = context_.map_signals.front();
  alternative.signal_id = "ambiguous";
  context_.map_signals.push_back(alternative);
  EXPECT_TRUE(Bind().binding_ambiguous);
  EXPECT_NE(context_.status.degrade_reason.find("near-equal signal candidates"),
            std::string::npos);
}

TEST_F(BinderStageTest, ObservedClusterIsNotASharedControlPhase) {
  context_.lane_context.candidates.front().movement_mask =
      kMovementMaskStraight | kMovementMaskLeft;
  auto left_signal = context_.map_signals.front();
  left_signal.signal_id = "left_signal";
  left_signal.signal_group_id = "left_control_group";
  left_signal.movement_mask = kMovementMaskLeft;
  context_.map_signals.push_back(left_signal);
  auto left_track = context_.tracked_lights.front();
  left_track.track_id = 8;
  left_track.current_state.visual_light.track_id = 8;
  left_track.current_state.visual_light.bbox.x += 30.0f;
  left_track.current_state.visual_light.shape = LightShape::ARROW_LEFT;
  left_track.current_state.visual_light.color = LightColor::GREEN;
  context_.tracked_lights.push_back(left_track);
  ASSERT_TRUE(stage_.Process(&context_));
  ASSERT_EQ(context_.bound_lights.size(), 2u);
  const auto& straight = context_.tracked_lights[0].current_state;
  const auto& left = context_.tracked_lights[1].current_state;
  EXPECT_EQ(straight.visual_light.observed_group_size, 2);
  EXPECT_EQ(left.visual_light.observed_group_size, 2);
  EXPECT_EQ(straight.signal_group_id, "control_group");
  EXPECT_EQ(left.signal_group_id, "left_control_group");
  EXPECT_EQ(straight.visual_light.color, LightColor::RED);
  EXPECT_EQ(left.visual_light.color, LightColor::GREEN);
}

TEST_F(BinderStageTest, MultipleHeadsMayBindToSameSignalGroup) {
  auto second = context_.tracked_lights.front();
  second.track_id = 8;
  second.current_state.visual_light.track_id = 8;
  second.current_state.visual_light.bbox.x += 30.0f;
  second.current_state.visual_light.observed_group_id = "cluster";
  context_.tracked_lights.front().current_state.visual_light.observed_group_id =
      "cluster";
  context_.tracked_lights.push_back(second);
  ASSERT_TRUE(stage_.Process(&context_));
  ASSERT_EQ(context_.bound_lights.size(), 2u);
  EXPECT_EQ(context_.bound_lights[0].signal_group_id,
            context_.bound_lights[1].signal_group_id);
  EXPECT_EQ(
      context_.tracked_lights[1].current_state.visual_light.observed_group_size,
      2);
  context_.tracked_lights.front()
      .current_state.visual_light.observed_group_size = 0;
  context_.tracked_lights[1].lost_frames = 1;
  ASSERT_TRUE(stage_.Process(&context_));
  EXPECT_EQ(context_.tracked_lights.front()
                .current_state.visual_light.observed_group_size,
            1);
  EXPECT_FALSE(context_.tracked_lights[1].current_state.binding_valid);
}

TEST_F(BinderStageTest, CamerasCannotShareProjectionOrObservedGroups) {
  auto other = context_.tracked_lights.front();
  other.track_id = 8;
  other.current_state.visual_light.track_id = 8;
  other.current_state.visual_light.camera_name = "rear";
  context_.tracked_lights.push_back(other);
  ASSERT_TRUE(stage_.Process(&context_));
  EXPECT_TRUE(context_.tracked_lights[0].current_state.binding_valid);
  EXPECT_FALSE(context_.tracked_lights[1].current_state.binding_valid);
  EXPECT_EQ(
      context_.tracked_lights[0].current_state.visual_light.observed_group_size,
      1);
  EXPECT_EQ(
      context_.tracked_lights[1].current_state.visual_light.observed_group_size,
      1);
  context_.lane_context = LaneContext();
  context_.map_signals.front().projection_reliable = true;
  context_.map_signals.front().camera_name.clear();
  EXPECT_FALSE(Bind().binding_valid);
}

TEST_F(BinderStageTest, MissingTopologyNeverFabricatesLaneOrGroup) {
  context_.map_signals.clear();
  const auto& bound = Bind();
  EXPECT_FALSE(bound.binding_valid);
  EXPECT_FALSE(bound.controlling_ego_lane);
  EXPECT_TRUE(bound.lane_id.empty());
  EXPECT_TRUE(bound.signal_group_id.empty());
  EXPECT_EQ(context_.tracked_lights.size(), 1u);
}

TEST_F(BinderStageTest, HistoryCannotRenewItselfOrSurviveAnEpochChange) {
  ASSERT_TRUE(Bind().binding_valid);
  auto alternative = context_.map_signals.front();
  alternative.signal_id = "alternative";
  context_.map_signals.push_back(alternative);
  context_.timestamp = 10200000000ULL;
  context_.camera_frames.front().timestamp_ns = context_.timestamp;
  context_.tracked_lights.front().last_visible_timestamp_sec = 10.2;
  EXPECT_TRUE(Bind().binding_valid);
  context_.timestamp = 10700000000ULL;
  context_.camera_frames.front().timestamp_ns = context_.timestamp;
  context_.tracked_lights.front().last_visible_timestamp_sec = 10.7;
  context_.lane_context.valid_until_sec = 11.0;
  EXPECT_TRUE(Bind().binding_ambiguous);
  context_.map_signals.pop_back();
  ASSERT_TRUE(Bind().binding_valid);
  context_.map_signals.push_back(alternative);
  ++context_.lane_context.epoch;
  EXPECT_TRUE(Bind().binding_ambiguous);
}

TEST_F(BinderStageTest, IsolatedVisualInputUsesSameSemanticContract) {
  context_.visual_lights.push_back(
      context_.tracked_lights.front().current_state.visual_light);
  context_.tracked_lights.clear();
  ASSERT_TRUE(stage_.Process(&context_));
  ASSERT_EQ(context_.bound_lights.size(), 1u);
  EXPECT_TRUE(context_.bound_lights.front().binding_valid);
  EXPECT_EQ(context_.visual_lights.front().track_id, 7);
}

}  // namespace
}  // namespace traffic_light
}  // namespace apollo
