#include "modules/perception/traffic_light/stages/detector_stage.h"

#include <vector>

#include "gtest/gtest.h"
#include "modules/perception/traffic_light/stages/binder_stage.h"
#include "modules/perception/traffic_light/stages/fusion_stage.h"
#include "modules/perception/traffic_light/stages/heuristic_stage.h"
#include "modules/perception/traffic_light/stages/prompter_stage.h"
#include "modules/perception/traffic_light/stages/tracker_stage.h"

namespace apollo {
namespace perception {
namespace traffic_light {
namespace {

TEST(TrafficLightStageTest, PrompterCombinesHistoryMapAndFallback) {
  PipelineContext context;
  RuntimeState runtime;
  context.runtime_state = &runtime;
  context.primary_camera_name = "front_6mm";
  context.nav_topology.distance_to_intersection_m = 40.0;
  CameraFrameState frame;
  frame.camera_name = "front_6mm";
  frame.image.cols = 1920;
  frame.image.rows = 1080;
  context.camera_frames.push_back(frame);

  TrackedLight track;
  track.track_id = 1;
  track.confirmed = true;
  track.current_state.visual_light.camera_name = "front_6mm";
  track.current_state.visual_light.signal_id = "tl_hist";
  track.current_state.visual_light.bbox = Rect2f{100.0f, 120.0f, 20.0f, 40.0f};
  runtime.tracked_memory.push_back(track);

  SignalCandidate signal;
  signal.signal_id = "tl_map";
  signal.camera_name = "front_6mm";
  signal.projection_roi = Rect2f{200.0f, 100.0f, 25.0f, 50.0f};
  signal.confidence = 0.9f;
  context.map_signals.push_back(signal);

  PrompterStage stage{PrompterOptions()};
  ASSERT_TRUE(stage.Process(&context));
  EXPECT_GE(context.prompts.size(), 3u);
  EXPECT_EQ(context.prompts.front().source, PromptSource::HISTORY);
  EXPECT_TRUE(context.metrics.prompter.fallback_used);
}

TEST(TrafficLightStageTest, DetectorDoesNotFallbackWithoutNeuralOutputInYoloMode) {
  std::vector<uint8_t> image_data(12 * 12 * 3, 0);

  PipelineContext context;
  context.primary_camera_name = "front_6mm";
  CameraFrameState frame;
  frame.camera_name = "front_6mm";
  frame.image.data = image_data.data();
  frame.image.rows = 12;
  frame.image.cols = 12;
  frame.image.channels = 3;
  frame.image.encoding = "rgb8";
  context.camera_frames.push_back(frame);
  context.prompts.push_back(
      PromptRegion{Rect2f{2.0f, 2.0f, 8.0f, 8.0f}, 0.95f, PromptSource::MAP,
                   "tl_1", "front_6mm", LaneIntent::STRAIGHT});

  DetectorOptions options;
  options.backend = DetectorBackendType::YOLO;
  DetectorStage stage{options};
  EXPECT_TRUE(stage.Process(&context));
  EXPECT_TRUE(context.visual_lights.empty());
  EXPECT_TRUE(context.status.detector_ran);
  EXPECT_NE(context.status.degrade_reason.find("neural detector unavailable"),
            std::string::npos);
}

TEST(TrafficLightStageTest, DetectorDecodesRawYoloSemanticOutput) {
  PipelineContext context;
  context.primary_camera_name = "front_6mm";
  CameraFrameState frame;
  frame.camera_name = "front_6mm";
  frame.image.rows = 1080;
  frame.image.cols = 1920;
  context.camera_frames.push_back(frame);

  SignalCandidate signal;
  signal.signal_id = "sig_left_straight";
  signal.camera_name = "front_6mm";
  signal.projection_roi = Rect2f{100.0f, 60.0f, 32.0f, 72.0f};
  signal.confidence = 0.9f;
  signal.topology_confidence = 0.95f;
  signal.movement_mask = kMovementMaskLeft | kMovementMaskStraight;
  context.map_signals.push_back(signal);

  YoloLightCandidate candidate;
  candidate.bbox = Rect2f{102.0f, 62.0f, 30.0f, 70.0f};
  candidate.objectness = 0.91f;
  candidate.semantic_confidence = 0.94f;
  candidate.class_id = 7;
  candidate.class_name = "green_left_straight";
  candidate.color = LightColor::GREEN;
  candidate.shape = LightShape::UNKNOWN;
  candidate.movement_mask = kMovementMaskLeft | kMovementMaskStraight;
  candidate.signal_id = "sig_left_straight";
  candidate.camera_name = "front_6mm";
  context.raw_yolo_lights.push_back(candidate);
  context.status.neural_detector_ran = true;

  DetectorOptions options;
  options.backend = DetectorBackendType::YOLO;
  DetectorStage stage{options};
  ASSERT_TRUE(stage.Process(&context));
  ASSERT_EQ(context.visual_lights.size(), 1u);
  EXPECT_EQ(context.visual_lights.front().color, LightColor::GREEN);
  EXPECT_EQ(context.visual_lights.front().signal_id, "sig_left_straight");
  EXPECT_EQ(context.visual_lights.front().yolo_class_name, "green_left_straight");
  EXPECT_EQ(context.visual_lights.front().movement_mask,
            kMovementMaskLeft | kMovementMaskStraight);
}

TEST(TrafficLightStageTest, BinderRejectsFarFalseAssociation) {
  PipelineContext context;
  context.nav_topology.ego_lane_intent = LaneIntent::STRAIGHT;

  VisualLight visual;
  visual.bbox = Rect2f{500.0f, 500.0f, 20.0f, 20.0f};
  visual.color = LightColor::RED;
  visual.confidence = 0.95f;
  visual.existence_confidence = 0.95f;
  visual.state_confidence = 0.90f;
  visual.shape = LightShape::ARROW_STRAIGHT;
  context.visual_lights.push_back(visual);

  SignalCandidate signal;
  signal.signal_id = "sig_1";
  signal.projection_roi = Rect2f{50.0f, 50.0f, 20.0f, 20.0f};
  signal.confidence = 0.8f;
  signal.topology_confidence = 0.9f;
  signal.intended_movement = LaneIntent::STRAIGHT;
  context.map_signals.push_back(signal);

  BinderStage stage{BinderOptions()};
  ASSERT_TRUE(stage.Process(&context));
  EXPECT_TRUE(context.bound_lights.empty());
  EXPECT_EQ(context.metrics.binder.rejected_count, 1);
}

TEST(TrafficLightStageTest, BinderMatchesCombinedSignalIntentToEgoIntent) {
  PipelineContext context;
  context.nav_topology.ego_lane_intent = LaneIntent::LEFT;

  VisualLight visual;
  visual.bbox = Rect2f{102.0f, 102.0f, 18.0f, 42.0f};
  visual.color = LightColor::GREEN;
  visual.confidence = 0.9f;
  visual.existence_confidence = 0.92f;
  visual.state_confidence = 0.91f;
  visual.movement_mask = kMovementMaskLeft | kMovementMaskStraight;
  context.visual_lights.push_back(visual);

  SignalCandidate signal;
  signal.signal_id = "sig_combo";
  signal.projection_roi = Rect2f{100.0f, 100.0f, 20.0f, 40.0f};
  signal.confidence = 0.85f;
  signal.topology_confidence = 0.95f;
  signal.movement_mask = kMovementMaskLeft | kMovementMaskStraight;
  context.map_signals.push_back(signal);

  BinderStage stage{BinderOptions()};
  ASSERT_TRUE(stage.Process(&context));
  ASSERT_EQ(context.bound_lights.size(), 1u);
  EXPECT_TRUE(context.bound_lights.front().controlling_ego_lane);
  EXPECT_EQ(context.bound_lights.front().signal_movement_mask,
            kMovementMaskLeft | kMovementMaskStraight);
  EXPECT_GT(context.bound_lights.front().movement_match_score, 0.9f);
}

TEST(TrafficLightStageTest, TrackerKeepsTrackAcrossShortOcclusion) {
  PipelineContext context;
  RuntimeState runtime;
  context.runtime_state = &runtime;
  context.timestamp = 1000000000ULL;

  BoundLight bound;
  bound.visual_light.bbox = Rect2f{10.0f, 10.0f, 10.0f, 20.0f};
  bound.visual_light.color = LightColor::RED;
  bound.visual_light.confidence = 0.9f;
  bound.visual_light.existence_confidence = 0.9f;
  bound.visual_light.state_confidence = 0.9f;
  bound.visual_light.signal_id = "track_1";
  bound.bind_score = 0.9f;
  bound.topology_confidence = 0.9f;
  context.bound_lights.push_back(bound);

  TrackerStage stage{TrackerOptions()};
  ASSERT_TRUE(stage.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);

  context.bound_lights.clear();
  context.timestamp = 1100000000ULL;
  ASSERT_TRUE(stage.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  EXPECT_EQ(context.tracked_lights.front().lost_frames, 1);
}

TEST(TrafficLightStageTest, HeuristicNeedsTopologyAndMultipleAgents) {
  PipelineContext context;
  context.nav_topology.ego_lane_intent = LaneIntent::STRAIGHT;
  context.nav_topology.distance_to_intersection_m = 20.0;

  AgentState agent;
  agent.intent = LaneIntent::STRAIGHT;
  agent.is_starting = true;
  context.surrounding_agents.push_back(agent);

  HeuristicStage stage{HeuristicOptions()};
  ASSERT_TRUE(stage.Process(&context));
  EXPECT_EQ(context.heuristic_state.inferred_color, LightColor::UNKNOWN);

  SignalCandidate signal;
  signal.signal_id = "sig_1";
  context.map_signals.push_back(signal);
  context.surrounding_agents.push_back(agent);
  ASSERT_TRUE(stage.Process(&context));
  EXPECT_EQ(context.heuristic_state.inferred_color, LightColor::GREEN);
}

TEST(TrafficLightStageTest, FusionPrefersStrongV2XOverWeakVision) {
  PipelineContext context;
  context.timestamp = 1000000000ULL;
  context.nav_topology.ego_lane_intent = LaneIntent::STRAIGHT;

  TrackedLight track;
  track.track_id = 7;
  track.stabilized_color = LightColor::RED;
  track.stabilized_confidence = 0.3f;
  track.current_state.bound_intent = LaneIntent::STRAIGHT;
  track.current_state.stopline_distance_m = 18.0;
  track.current_state.visual_light.signal_id = "tl_1";
  track.current_state.visual_light.shape = LightShape::CIRCLE;
  track.current_state.visual_light.camera_name = "front_6mm";
  context.tracked_lights.push_back(track);

  V2XLightEvidence v2x;
  v2x.signal_id = "tl_1";
  v2x.movement = LaneIntent::STRAIGHT;
  v2x.movement_mask = kMovementMaskStraight;
  v2x.color = LightColor::GREEN;
  v2x.confidence = 0.95f;
  v2x.timestamp_sec = 1.0;
  context.v2x_lights.push_back(v2x);

  FusionStage stage{FusionOptions()};
  ASSERT_TRUE(stage.Process(&context));
  ASSERT_FALSE(context.final_lights.empty());
  EXPECT_EQ(context.primary_decision.color, LightColor::GREEN);
  EXPECT_EQ(context.primary_decision.source, EvidenceSource::V2X);
}

TEST(TrafficLightStageTest, FusionKeepsStrongVisionAgainstConflictingV2X) {
  PipelineContext context;
  context.timestamp = 1000000000ULL;
  context.nav_topology.ego_lane_intent = LaneIntent::STRAIGHT;

  TrackedLight track;
  track.track_id = 3;
  track.stabilized_color = LightColor::RED;
  track.stabilized_confidence = 0.95f;
  track.current_state.bound_intent = LaneIntent::STRAIGHT;
  track.current_state.topology_confidence = 0.95f;
  track.current_state.controlling_ego_lane = true;
  track.current_state.visual_light.signal_id = "tl_2";
  track.current_state.visual_light.existence_confidence = 0.95f;
  track.current_state.visual_light.shape = LightShape::CIRCLE;
  track.last_visible_timestamp_sec = 1.0;
  context.tracked_lights.push_back(track);

  V2XLightEvidence v2x;
  v2x.signal_id = "tl_2";
  v2x.movement = LaneIntent::STRAIGHT;
  v2x.movement_mask = kMovementMaskStraight;
  v2x.color = LightColor::GREEN;
  v2x.confidence = 0.9f;
  v2x.timestamp_sec = 1.0;
  context.v2x_lights.push_back(v2x);

  FusionStage stage{FusionOptions()};
  ASSERT_TRUE(stage.Process(&context));
  EXPECT_EQ(context.primary_decision.color, LightColor::RED);
  EXPECT_EQ(context.primary_decision.source, EvidenceSource::VISION);
}

TEST(TrafficLightStageTest, FusionPublishesDegradedUnknownForMappedSignal) {
  PipelineContext context;
  context.nav_topology.ego_lane_intent = LaneIntent::STRAIGHT;
  context.AppendDegradeReason("neural detector unavailable");

  SignalCandidate signal;
  signal.signal_id = "Signal_1";
  signal.lane_id = "Lane_83";
  signal.camera_name = "front_6mm";
  signal.topology_confidence = 0.9f;
  signal.intended_movement = LaneIntent::STRAIGHT;
  signal.movement_mask = kMovementMaskStraight;
  signal.stopline_distance_m = 18.0;
  context.map_signals.push_back(signal);

  FusionStage stage{FusionOptions()};
  ASSERT_TRUE(stage.Process(&context));
  ASSERT_EQ(context.final_lights.size(), 1u);
  EXPECT_EQ(context.final_lights.front().signal_id, "Signal_1");
  EXPECT_EQ(context.final_lights.front().color, LightColor::UNKNOWN);
  EXPECT_TRUE(context.final_lights.front().is_degraded);
  EXPECT_TRUE(context.final_lights.front().controlling_ego_lane);
}

}  // namespace
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
