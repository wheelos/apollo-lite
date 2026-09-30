#include "modules/perception/fusion/app/fusion_runtime.h"

#include <limits>
#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace fusion {
namespace {

bool CompleteTick(FusionFrame* frame) {
  if (frame->publish_tick) {
    frame->ready = true;
    frame->degraded = false;
    frame->scene_ptr = std::make_shared<Scene>();
  }
  return true;
}

TEST(FusionRuntimeTest, ObservationDoesNotDrivePublication) {
  FusionRuntime runtime;
  int observations = 0;
  int ticks = 0;
  ASSERT_TRUE(runtime.Init("lidar", 0.05, [&](FusionFrame* frame) {
    if (frame->publish_tick) {
      ++ticks;
      frame->ready = true;
      frame->scene_ptr = std::make_shared<Scene>();
    } else {
      ++observations;
    }
    return true;
  }));
  auto observation = std::make_shared<base::Frame>();
  observation->timestamp = 10.0;
  observation->sensor_info.name = "lidar";
  ASSERT_TRUE(runtime.Add(observation));
  EXPECT_EQ(observations, 1);
  EXPECT_EQ(ticks, 0);
  FusionFrame output;
  ASSERT_TRUE(runtime.Tick(10.1, &output));
  EXPECT_NEAR(output.frame->timestamp, 10.05, 1e-8);
  EXPECT_EQ(ticks, 1);
  ASSERT_TRUE(runtime.Tick(10.2, &output));
  EXPECT_EQ(ticks, 2);
  EXPECT_EQ(observations, 1);
}

TEST(FusionRuntimeTest, RecoversOnlyAfterProcessingANewerObservation) {
  FusionRuntime runtime;
  base::FrameConstPtr pending;
  ASSERT_TRUE(runtime.Init("lidar", 0, [&](FusionFrame* frame) {
    if (!frame->publish_tick) {
      pending = frame->frame;
      frame->admitted = true;
      return true;
    }
    CompleteTick(frame);
    if (pending && pending->timestamp <= frame->frame->timestamp) {
      frame->sensor_frames.push_back(std::make_shared<SensorFrame>(pending));
      pending.reset();
    }
    return true;
  }));
  ASSERT_TRUE(runtime.ReportSourceError("lidar", 10.0));
  FusionFrame output;
  ASSERT_TRUE(runtime.Tick(10.0, &output));
  EXPECT_TRUE(output.degraded);
  auto observation = std::make_shared<base::Frame>();
  observation->sensor_info.name = "lidar";
  observation->sensor2world_pose.setIdentity();
  observation->timestamp = 10.1;
  ASSERT_TRUE(runtime.Add(observation));
  ASSERT_TRUE(runtime.Tick(10.05, &output));
  EXPECT_TRUE(output.degraded);
  ASSERT_TRUE(runtime.Tick(10.2, &output));
  EXPECT_FALSE(output.degraded);
  EXPECT_TRUE(output.has_publish_pose);
  ASSERT_TRUE(runtime.ReportSourceError("lidar", 10.0));
  ASSERT_TRUE(runtime.Tick(10.3, &output));
  EXPECT_FALSE(output.degraded);
  EXPECT_TRUE(output.has_publish_pose);
}

TEST(FusionRuntimeTest, RejectsUnknownErrorsAndIncompleteSuccessfulCycles) {
  FusionRuntime runtime;
  ASSERT_TRUE(runtime.Init("lidar", 0, CompleteTick,
      [](const std::string& source) { return source == "lidar"; }));
  EXPECT_FALSE(runtime.ReportSourceError("unknown", 10.0));
  EXPECT_FALSE(runtime.ReportSourceError(
      "lidar", std::numeric_limits<double>::quiet_NaN()));
  ASSERT_TRUE(runtime.Init("lidar", 0, [](FusionFrame*) { return true; }));
  FusionFrame output;
  EXPECT_FALSE(runtime.Tick(10.0, &output));
  EXPECT_TRUE(output.degraded);
  EXPECT_TRUE(output.fused_objects.empty());
}

TEST(FusionRuntimeTest, FaultWithholdsPartialOutputAndDoesNotRetryPipeline) {
  FusionRuntime runtime;
  int cycles = 0;
  ASSERT_TRUE(runtime.Init("lidar", 0, [&](FusionFrame* frame) {
    ++cycles;
    frame->fused_objects.push_back(std::make_shared<base::Object>());
    return false;
  }));
  FusionFrame output;
  EXPECT_FALSE(runtime.Tick(10.0, &output));
  EXPECT_TRUE(output.degraded);
  EXPECT_TRUE(output.fused_objects.empty());
  EXPECT_FALSE(runtime.Tick(10.1, &output));
  EXPECT_EQ(cycles, 1);
  EXPECT_TRUE(output.fused_objects.empty());
}

TEST(FusionRuntimeTest, RejectsInvalidConfigurationAndClockRollback) {
  FusionRuntime runtime;
  EXPECT_FALSE(runtime.Init("", 0, [](FusionFrame*) { return true; }));
  EXPECT_FALSE(runtime.Init("lidar", -1, [](FusionFrame*) { return true; }));
  ASSERT_TRUE(runtime.Init("lidar", 0, CompleteTick));
  FusionFrame output;
  ASSERT_TRUE(runtime.Tick(10.0, &output));
  EXPECT_FALSE(runtime.Tick(10.0, &output));
  EXPECT_FALSE(runtime.Tick(9.0, &output));
  EXPECT_FALSE(runtime.Tick(std::numeric_limits<double>::quiet_NaN(), &output));
  EXPECT_FALSE(runtime.Add(nullptr));
}

}  // namespace
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
