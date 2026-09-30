#include "modules/perception/fusion/base/frame_scheduler.h"

#include <limits>
#include <memory>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace fusion {
namespace {

base::FramePtr Frame(double timestamp, const std::string& sensor = "lidar") {
  auto frame = std::make_shared<base::Frame>();
  frame->timestamp = timestamp;
  frame->sensor_info.name = sensor;
  frame->sensor2world_pose = Eigen::Affine3d::Identity();
  return frame;
}

TEST(FrameSchedulerTest, OrdersEveryObservationAndRetainsFutureFrames) {
  FrameScheduler scheduler;
  ASSERT_TRUE(scheduler.Add(Frame(1.03)));
  ASSERT_TRUE(scheduler.Add(Frame(1.01)));
  ASSERT_TRUE(scheduler.Add(Frame(1.02, "radar")));
  ASSERT_TRUE(scheduler.Add(Frame(1.02)));
  std::vector<base::FrameConstPtr> frames;
  ASSERT_TRUE(scheduler.Drain(1.02, &frames));
  ASSERT_EQ(frames.size(), 3);
  EXPECT_DOUBLE_EQ(frames.front()->timestamp, 1.01);
  EXPECT_DOUBLE_EQ(frames.back()->timestamp, 1.02);
  ASSERT_TRUE(scheduler.Drain(1.04, &frames));
  ASSERT_EQ(frames.size(), 1);
  EXPECT_DOUBLE_EQ(frames.front()->timestamp, 1.03);
}

TEST(FrameSchedulerTest, RejectsLateDuplicateInvalidAndOverflowObservations) {
  FrameScheduler scheduler(2);
  ASSERT_TRUE(scheduler.Add(Frame(1.0)));
  EXPECT_FALSE(scheduler.Add(Frame(1.0)));
  ASSERT_TRUE(scheduler.Add(Frame(1.1)));
  EXPECT_FALSE(scheduler.Add(Frame(1.2)));
  EXPECT_FALSE(scheduler.Add(nullptr));
  EXPECT_FALSE(scheduler.Add(Frame(std::numeric_limits<double>::quiet_NaN())));
  std::vector<base::FrameConstPtr> frames;
  ASSERT_TRUE(scheduler.Drain(1.1, &frames));
  EXPECT_FALSE(scheduler.Add(Frame(1.05, "radar")));
  EXPECT_FALSE(scheduler.Drain(1.1, &frames));
  EXPECT_FALSE(scheduler.Drain(1.0, &frames));
  EXPECT_FALSE(scheduler.Drain(1.2, nullptr));
}

TEST(FrameSchedulerTest, ReportsProcessedSourceFreshnessNotArrivalFreshness) {
  FrameScheduler scheduler;
  ASSERT_TRUE(scheduler.Add(Frame(1.0)));
  EXPECT_FALSE(scheduler.HasFreshSensor("lidar", 1.0, 0.25));
  std::vector<base::FrameConstPtr> frames;
  ASSERT_TRUE(scheduler.Drain(1.0, &frames));
  EXPECT_TRUE(scheduler.HasFreshSensor("lidar", 1.2, 0.25));
  EXPECT_FALSE(scheduler.HasFreshSensor("lidar", 1.3, 0.25));
  EXPECT_FALSE(scheduler.HasFreshSensor("radar", 1.0, 0.25));
}

TEST(FrameSchedulerTest, OwnsSnapshotAndRejectsInvalidObjects) {
  FrameScheduler scheduler;
  auto input = Frame(1.0);
  input->objects.push_back(std::make_shared<base::Object>());
  input->objects[0]->center.x() = 1.0;
  ASSERT_TRUE(scheduler.Add(input));
  input->objects[0]->center.x() = 99.0;
  std::vector<base::FrameConstPtr> frames;
  ASSERT_TRUE(scheduler.Drain(1.0, &frames));
  ASSERT_EQ(frames.size(), 1);
  EXPECT_DOUBLE_EQ(frames[0]->objects[0]->center.x(), 1.0);
  auto invalid = Frame(1.1);
  invalid->objects.push_back(nullptr);
  EXPECT_FALSE(scheduler.Add(invalid));
}

TEST(FrameSchedulerTest, RejectsNonRigidPosesNegativeTimeAndDuplicateLocalIds) {
  FrameScheduler scheduler;
  EXPECT_FALSE(scheduler.Add(Frame(-1.0)));
  auto frame = Frame(1.0);
  frame->sensor2world_pose.linear().setZero();
  EXPECT_FALSE(scheduler.Add(frame));
  frame->sensor2world_pose.setIdentity();
  frame->objects.push_back(std::make_shared<base::Object>());
  frame->objects.push_back(std::make_shared<base::Object>());
  frame->objects[0]->track_id = frame->objects[1]->track_id = 7;
  EXPECT_FALSE(scheduler.Add(frame));
  frame->objects[1]->track_id = 8;
  ASSERT_TRUE(scheduler.Add(frame));
}

}  // namespace
}  // namespace fusion
}  // namespace perception
}  // namespace apollo
