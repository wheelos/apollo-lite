// Copyright 2026 The Apollo Authors. All Rights Reserved.
#include "modules/traffic_light/stages/tracker_stage.h"

#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace apollo {
namespace traffic_light {
namespace {

void SetFrame(double timestamp, PipelineContext* context,
              const std::string& camera = "front") {
  ++context->frame_id;
  context->timestamp = static_cast<uint64_t>(timestamp * 1e9);
  context->primary_camera_name = camera;
  context->camera_frames.clear();
  CameraFrameState frame;
  frame.camera_name = camera;
  frame.timestamp_ns = context->timestamp;
  frame.image.rows = 1080;
  frame.image.cols = 1920;
  context->camera_frames.push_back(frame);
  context->visual_lights.clear();
  context->status.neural_detector_ran = true;
}

VisualLight Detection(float x = 100.0f, float score = 0.9f,
                      LightColor color = LightColor::RED,
                      const std::string& camera = "front") {
  VisualLight light;
  light.bbox = Rect2f{x, 100.0f, 20.0f, 40.0f};
  light.camera_name = camera;
  light.existence_confidence = score;
  light.state_confidence = 0.9f;
  light.confidence = 0.9f;
  light.color = color;
  return light;
}

TrackerOptions ImmediateConfirmation() {
  TrackerOptions options;
  options.min_confirmed_visible_count = 1;
  return options;
}

TEST(TrafficLightTrackerTest, PredictUsesRealDtWithoutDoubleAdvance) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  const int id = context.visual_lights.front().track_id;
  SetFrame(1.1, &context);
  context.visual_lights.push_back(Detection(104.0f));
  context.visual_lights.front().bbox.width = 22.0f;
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  const float observed_x =
      context.tracked_lights.front().current_state.visual_light.bbox.x;
  SetFrame(1.2, &context);
  ASSERT_TRUE(tracker.Predict(&context));
  ASSERT_EQ(context.track_predictions.size(), 1u);
  const auto prediction = context.track_predictions.front();
  EXPECT_EQ(prediction.track_id, id);
  EXPECT_GT(prediction.bbox.x, observed_x);
  EXPECT_GT(
      prediction.bbox.width,
      context.tracked_lights.front().current_state.visual_light.bbox.width);
  EXPECT_DOUBLE_EQ(prediction.last_observed_sec, 1.1);
  EXPECT_DOUBLE_EQ(prediction.timestamp_sec, 1.2);
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_FLOAT_EQ(
      context.tracked_lights.front().current_state.visual_light.bbox.x,
      prediction.bbox.x);
  SetFrame(1.4, &context);
  ASSERT_TRUE(tracker.Predict(&context));
  ASSERT_EQ(context.track_predictions.size(), 1u);
  // Constant velocity extrapolates twice the displacement over twice the dt.
  EXPECT_NEAR(context.track_predictions.front().bbox.x - prediction.bbox.x,
              2.0f * (prediction.bbox.x - observed_x), 1e-4f);
  EXPECT_GT(context.track_predictions.front().position_stddev_px,
            prediction.position_stddev_px);
  EXPECT_DOUBLE_EQ(context.track_predictions.front().last_observed_sec, 1.1);
}

TEST(TrafficLightTrackerTest, GlobalAssignmentDoesNotStarveNearbyTrack) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  context.visual_lights = {Detection(100.0f), Detection(115.0f)};
  ASSERT_TRUE(tracker.Process(&context));
  const int left_id = context.visual_lights[0].track_id;
  const int right_id = context.visual_lights[1].track_id;
  SetFrame(1.1, &context);
  context.visual_lights = {Detection(105.0f), Detection(90.0f)};
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 2u);
  EXPECT_EQ(context.visual_lights[0].track_id, right_id);
  EXPECT_EQ(context.visual_lights[1].track_id, left_id);
}

TEST(TrafficLightTrackerTest, LowOnlyContinuesConfirmedAndHighMatchesFirst) {
  TrackerStage tracker{TrackerOptions()};
  PipelineContext context;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection(100.0f, 0.2f));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights.front().track_id, 0);
  SetFrame(1.1, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  const int id = context.visual_lights.front().track_id;
  EXPECT_TRUE(context.tracked_lights.empty());
  SetFrame(1.2, &context);
  context.visual_lights.push_back(Detection(100.0f, 0.2f));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights.front().track_id, 0);
  EXPECT_TRUE(context.tracked_lights.empty());
  SetFrame(1.3, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  EXPECT_EQ(context.visual_lights.front().track_id, id);
  SetFrame(1.4, &context);
  context.visual_lights.push_back(Detection(100.0f, 0.2f));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights.front().track_id, id);
  SetFrame(1.5, &context);
  context.visual_lights.push_back(Detection(100.0f, 0.2f));
  context.visual_lights.push_back(Detection(101.0f));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights[0].track_id, 0);
  EXPECT_EQ(context.visual_lights[1].track_id, id);
}

TEST(TrafficLightTrackerTest, AssociationIsOneToOneWithoutSemanticGates) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  const int id = context.visual_lights.front().track_id;
  SetFrame(1.1, &context);
  context.visual_lights.push_back(Detection(100.0f, 0.9f, LightColor::GREEN));
  context.visual_lights.front().shape = LightShape::ARROW_LEFT;
  context.visual_lights.front().signal_id = "new-map-association";
  context.visual_lights.push_back(Detection(102.0f));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights[0].track_id, id);
  EXPECT_NE(context.visual_lights[1].track_id, id);
  EXPECT_GT(context.visual_lights[1].track_id, 0);
  ASSERT_EQ(context.tracked_lights.size(), 2u);
  EXPECT_EQ(context.tracked_lights.front().stabilized_color, LightColor::GREEN);
  EXPECT_FALSE(context.tracked_lights.front().current_state.binding_valid);
}

TEST(TrafficLightTrackerTest, MissingColorExpiresWithoutDestroyingGeometry) {
  TrackerStage tracker{ImmediateConfirmation()};
  RuntimeState runtime;
  PipelineContext context;
  context.runtime_state = &runtime;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  const int id = context.tracked_lights.front().track_id;
  EXPECT_GT(context.tracked_lights.front().stabilized_confidence, 0.0f);
  EXPECT_LT(context.tracked_lights.front().stabilized_confidence, 0.9f);
  SetFrame(1.4, &context);
  context.visual_lights.push_back(Detection(100.0f, 0.9f, LightColor::UNKNOWN));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_DOUBLE_EQ(context.tracked_lights.front().last_state_observed_sec, 1.0);
  SetFrame(1.7, &context);
  context.status.neural_detector_ran = false;
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  const auto& track = context.tracked_lights.front();
  EXPECT_EQ(track.track_id, id);
  EXPECT_EQ(track.lost_frames, 0);
  EXPECT_EQ(track.stabilized_color, LightColor::UNKNOWN);
  EXPECT_EQ(track.current_state.visual_light.color, LightColor::UNKNOWN);
  EXPECT_FLOAT_EQ(track.stabilized_confidence, 0.0f);
  EXPECT_FALSE(track.blink);
  ASSERT_EQ(runtime.tracked_memory.size(), 1u);
  EXPECT_EQ(runtime.tracked_memory.front().track_id, id);
  SetFrame(1.8, &context);
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.tracked_lights.front().lost_frames, 1);
  SetFrame(3.0, &context);
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_TRUE(context.tracked_lights.empty());
  EXPECT_TRUE(runtime.tracked_memory.empty());
}

TEST(TrafficLightTrackerTest, GeometryWithoutStateStartsUnknown) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  auto light = Detection();
  light.state_confidence = 0.0f;
  context.visual_lights.push_back(light);
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  EXPECT_EQ(context.tracked_lights.front().stabilized_color,
            LightColor::UNKNOWN);
  EXPECT_FLOAT_EQ(context.tracked_lights.front().stabilized_confidence, 0.0f);
}

TEST(TrafficLightTrackerTest, LowPassColorCannotRefreshState) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  const int id = context.visual_lights.front().track_id;
  for (double timestamp : {1.3, 1.7}) {
    SetFrame(timestamp, &context);
    context.visual_lights.push_back(Detection(102.0f, 0.2f, LightColor::GREEN));
    ASSERT_TRUE(tracker.Process(&context));
    ASSERT_EQ(context.tracked_lights.size(), 1u);
    const auto& track = context.tracked_lights.front();
    EXPECT_EQ(track.track_id, id);
    EXPECT_DOUBLE_EQ(track.last_visible_timestamp_sec, timestamp);
    EXPECT_DOUBLE_EQ(track.last_state_observed_sec, 1.0);
    EXPECT_EQ(track.stabilized_color,
              timestamp < 1.6 ? LightColor::RED : LightColor::UNKNOWN);
  }
}

TEST(TrafficLightTrackerTest, HighGeometryWithWeakStateCannotRefreshColor) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  auto weak = Detection();
  weak.state_confidence = 0.2f;
  context.visual_lights.push_back(weak);
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  EXPECT_EQ(context.tracked_lights.front().stabilized_color,
            LightColor::UNKNOWN);
  SetFrame(1.1, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  SetFrame(1.8, &context);
  context.visual_lights.push_back(weak);
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(context.tracked_lights.size(), 1u);
  EXPECT_DOUBLE_EQ(context.tracked_lights.front().last_visible_timestamp_sec,
                   1.8);
  EXPECT_DOUBLE_EQ(context.tracked_lights.front().last_state_observed_sec, 1.1);
  EXPECT_EQ(context.tracked_lights.front().stabilized_color,
            LightColor::UNKNOWN);
}

TEST(TrafficLightTrackerTest, CachedSecondaryFramesDoNotUpdateCameraPools) {
  TrackerStage tracker{ImmediateConfirmation()};
  RuntimeState runtime;
  PipelineContext context;
  context.runtime_state = &runtime;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  const int front_id = context.visual_lights.front().track_id;
  const auto cached_front = context.camera_frames.front();
  SetFrame(1.1, &context, "tele");
  const auto cached_tele = context.camera_frames.front();
  context.camera_frames.push_back(cached_front);
  context.visual_lights.push_back(
      Detection(100.0f, 0.9f, LightColor::RED, "tele"));
  ASSERT_TRUE(tracker.Process(&context));
  const int tele_id = context.visual_lights.front().track_id;
  ASSERT_EQ(runtime.tracked_memory.size(), 2u);
  for (const auto& track : runtime.tracked_memory) {
    EXPECT_EQ(track.age, 1);
    EXPECT_EQ(track.lost_frames, 0);
  }
  SetFrame(1.2, &context);
  const auto newer_front = context.camera_frames.front();
  context.camera_frames.push_back(cached_tele);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Predict(&context));
  ASSERT_EQ(context.track_predictions.size(), 1u);
  EXPECT_EQ(context.track_predictions.front().track_id, front_id);
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights.front().track_id, front_id);
  SetFrame(1.3, &context, "tele");
  context.camera_frames.push_back(newer_front);
  context.visual_lights.push_back(
      Detection(100.0f, 0.9f, LightColor::RED, "tele"));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights.front().track_id, tele_id);
  ASSERT_EQ(runtime.tracked_memory.size(), 2u);
  for (const auto& track : runtime.tracked_memory) {
    EXPECT_EQ(track.age, 2);
    EXPECT_EQ(track.lost_frames, 0);
  }
  EXPECT_EQ(context.status.degrade_reason.find("time reset"),
            std::string::npos);
}

TEST(TrafficLightTrackerTest, CamerasAreIsolatedAndDiscontinuitiesReset) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  const int front_id = context.visual_lights.front().track_id;
  SetFrame(1.1, &context, "tele");
  context.visual_lights.push_back(
      Detection(100.0f, 0.9f, LightColor::RED, "tele"));
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_NE(context.visual_lights.front().track_id, front_id);
  SetFrame(1.2, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_EQ(context.visual_lights.front().track_id, front_id);
  SetFrame(1.3, &context);
  context.camera_frames.front().image.cols = 1280;
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  const int resized_id = context.visual_lights.front().track_id;
  EXPECT_NE(resized_id, front_id);
  SetFrame(1.1, &context);
  context.camera_frames.front().image.cols = 1280;
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_NE(context.visual_lights.front().track_id, resized_id);
  EXPECT_NE(context.status.degrade_reason.find("time reset"),
            std::string::npos);
}

TEST(TrafficLightTrackerTest, AbsentCameraMemoryExpires) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  RuntimeState runtime;
  context.runtime_state = &runtime;
  SetFrame(1.0, &context);
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(runtime.tracked_memory.size(), 1u);
  SetFrame(1.7, &context, "tele");
  ASSERT_TRUE(tracker.Process(&context));
  ASSERT_EQ(runtime.tracked_memory.size(), 1u);
  EXPECT_EQ(runtime.tracked_memory.front().stabilized_color,
            LightColor::UNKNOWN);
  SetFrame(2.6, &context, "tele");
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_TRUE(runtime.tracked_memory.empty());
}

TEST(TrafficLightTrackerTest, RejectsNonfiniteObservationAndBadImage) {
  TrackerStage tracker{ImmediateConfirmation()};
  PipelineContext context;
  SetFrame(1.0, &context);
  auto light = Detection();
  light.bbox.x = std::numeric_limits<float>::quiet_NaN();
  context.visual_lights.push_back(light);
  light = Detection();
  light.state_confidence = std::numeric_limits<float>::infinity();
  context.visual_lights.push_back(light);
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_TRUE(context.tracked_lights.empty());
  EXPECT_EQ(context.metrics.tracker.rejected_count, 2);
  EXPECT_NE(context.status.degrade_reason.find("invalid visual observation"),
            std::string::npos);
  SetFrame(1.1, &context);
  context.camera_frames.front().image.rows = 0;
  context.visual_lights.push_back(Detection());
  ASSERT_TRUE(tracker.Process(&context));
  EXPECT_TRUE(context.tracked_lights.empty());
  EXPECT_NE(context.status.degrade_reason.find("invalid image dimensions"),
            std::string::npos);
}

}  // namespace
}  // namespace traffic_light
}  // namespace apollo
