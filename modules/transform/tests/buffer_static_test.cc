// Copyright 2026 WheelOS. All Rights Reserved.
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>

#include "gtest/gtest.h"

#include "cyber/time/clock.h"
#include "modules/transform/buffer.h"
#include "modules/transform/ground_reference.h"
#include "modules/transform/tests/buffer_test_peer.h"
#include "modules/transform/transform_query.h"

#ifdef TF2_BUFFER_CORE_H
#error "Public transform headers must not include the tf2 backend."
#endif

namespace apollo {
namespace transform {
namespace {

template <typename T, typename = void>
struct HasSetTf : std::false_type {};

template <typename T>
struct HasSetTf<T, std::void_t<decltype(&T::setTransform)>> : std::true_type {};

static_assert(!HasSetTf<Buffer>::value,
              "Buffer must not expose backend writes.");
static_assert(std::is_base_of<BufferInterface, Buffer>::value,
              "Buffer must implement the injectable read interface.");

TEST(StaticBufferTest, TranslatesBackendErrorsAtThePublicBoundary) {
  Buffer buffer(false);
  EXPECT_THROW(buffer.lookupTransform("missing", "source", cyber::Time(1.0), 0),
               TransformException);
  EXPECT_THROW(buffer.lookupTransform("missing", cyber::Time(1.0), "source",
                                      cyber::Time(1.0), "fixed", 0),
               TransformException);
  TransformQuery query(&buffer);
  Eigen::Affine3d output = Eigen::Affine3d::Identity();
  std::string error;
  EXPECT_FALSE(query.LookupTransformToAffine(
      "missing", "source", cyber::Time(1.0), &output, 0, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(output.matrix().isIdentity());
}

std::shared_ptr<TransformStampeds> Edge(const std::string& parent,
                                        const std::string& child, double x) {
  auto message = std::make_shared<TransformStampeds>();
  auto* transform = message->add_transforms();
  transform->mutable_header()->set_frame_id(parent);
  transform->mutable_header()->set_timestamp_sec(1);
  transform->set_child_frame_id(child);
  transform->mutable_transform()->mutable_translation()->set_x(x);
  transform->mutable_transform()->mutable_translation()->set_y(0);
  transform->mutable_transform()->mutable_translation()->set_z(0);
  transform->mutable_transform()->mutable_rotation()->set_qx(0);
  transform->mutable_transform()->mutable_rotation()->set_qy(0);
  transform->mutable_transform()->mutable_rotation()->set_qz(0);
  transform->mutable_transform()->mutable_rotation()->set_qw(1);
  return message;
}

TEST(StaticBufferTest, ComposesStaticChainsButNeverDynamicEdges) {
  Buffer buffer(false);
  BufferTestPeer::ReceiveStatic(buffer, Edge("base_link", "mount", 1));
  BufferTestPeer::ReceiveStatic(buffer, Edge("mount", "camera", 2));
  BufferTestPeer::ReceiveDynamic(buffer, Edge("odom", "base_link", 10));
  TransformStamped transform;
  ASSERT_TRUE(
      buffer.GetLatestStaticTransform("base_link", "camera", &transform));
  EXPECT_DOUBLE_EQ(transform.transform().translation().x(), 3);
  ASSERT_TRUE(
      buffer.GetLatestStaticTransform("camera", "base_link", &transform));
  EXPECT_DOUBLE_EQ(transform.transform().translation().x(), -3);
  EXPECT_FALSE(buffer.GetLatestStaticTransform("odom", "camera", &transform));
  EXPECT_FALSE(buffer.GetLatestStaticTransform("base_link", "camera", nullptr));
}

TEST(StaticBufferTest, RejectsChangedStaticChildAndInvalidCalibration) {
  Buffer buffer(false);
  BufferTestPeer::ReceiveStatic(buffer, Edge("base_link", "camera", 1));
  BufferTestPeer::ReceiveStatic(buffer, Edge("base_link", "camera", 4));
  auto invalid = Edge("base_link", "camera", 100);
  invalid->mutable_transforms(0)
      ->mutable_transform()
      ->mutable_rotation()
      ->set_qw(0);
  BufferTestPeer::ReceiveStatic(buffer, invalid);
  TransformStamped transform;
  ASSERT_TRUE(
      buffer.GetLatestStaticTransform("base_link", "camera", &transform));
  EXPECT_DOUBLE_EQ(transform.transform().translation().x(), 1);
  EXPECT_EQ(buffer.GetDiagnosticsSnapshot().rejected_message_count, 2);
}

TEST(StaticBufferTest,
     AllowsSameAuthorityRepeatButRejectsReparentAndTypeChange) {
  Buffer buffer(false);
  auto original = Edge("base_link", "camera", 1);
  original->mutable_header()->set_module_name("selected_calibration");
  BufferTestPeer::ReceiveStatic(buffer, original);
  auto duplicate = std::make_shared<TransformStampeds>(*original);
  duplicate->mutable_transforms(0)
      ->mutable_transform()
      ->mutable_rotation()
      ->set_qw(-1);
  BufferTestPeer::ReceiveStatic(buffer, duplicate);
  auto other_authority = std::make_shared<TransformStampeds>(*original);
  other_authority->mutable_header()->set_module_name("other");
  BufferTestPeer::ReceiveStatic(buffer, other_authority);
  BufferTestPeer::ReceiveStatic(buffer, Edge("mount", "camera", 1));
  BufferTestPeer::ReceiveDynamic(buffer, Edge("base_link", "camera", 99));
  TransformStamped transform;
  ASSERT_TRUE(
      buffer.GetLatestStaticTransform("base_link", "camera", &transform));
  EXPECT_DOUBLE_EQ(transform.transform().translation().x(), 1);
  EXPECT_DOUBLE_EQ(
      buffer.lookupTransform("base_link", "camera", cyber::Time(5.0))
          .transform()
          .translation()
          .x(),
      1);
  EXPECT_EQ(buffer.GetDiagnosticsSnapshot().rejected_message_count, 3);

  Buffer dynamic_first(false);
  BufferTestPeer::ReceiveDynamic(dynamic_first, Edge("base_link", "lidar", 2));
  BufferTestPeer::ReceiveStatic(dynamic_first, Edge("base_link", "lidar", 3));
  EXPECT_FALSE(
      dynamic_first.GetLatestStaticTransform("base_link", "lidar", &transform));
}

TEST(StaticBufferTest, RejectsWholeInvalidMessageAndCycles) {
  Buffer buffer(false);
  auto invalid = Edge("base_link", "camera", 1);
  *invalid->add_transforms() = Edge("camera", "mount", 2)->transforms(0);
  invalid->mutable_transforms(1)
      ->mutable_transform()
      ->mutable_translation()
      ->set_z(std::numeric_limits<double>::infinity());
  BufferTestPeer::ReceiveStatic(buffer, invalid);
  TransformStamped transform;
  EXPECT_FALSE(
      buffer.GetLatestStaticTransform("base_link", "camera", &transform));
  auto cyclic = Edge("mount", "camera", 1);
  *cyclic->add_transforms() = Edge("camera", "mount", 2)->transforms(0);
  BufferTestPeer::ReceiveStatic(buffer, cyclic);
  EXPECT_FALSE(buffer.GetLatestStaticTransform("mount", "camera", &transform));
  EXPECT_EQ(buffer.GetDiagnosticsSnapshot().rejected_message_count, 2);
}

class MockClockBufferTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mode_ = cyber::Clock::mode();
    now_ = cyber::Clock::Now();
    cyber::Clock::SetMode(cyber::proto::ClockMode::MODE_MOCK);
    cyber::Clock::SetNow(cyber::Time(10.0));
  }
  void TearDown() override {
    cyber::Clock::SetMode(mode_);
    if (mode_ == cyber::proto::ClockMode::MODE_MOCK) {
      cyber::Clock::SetNow(now_);
    }
  }
  cyber::proto::ClockMode mode_;
  cyber::Time now_;
};

TEST_F(MockClockBufferTest, WaitsWithoutAdvancingFrozenClockInBothOverloads) {
  Buffer buffer(false);
  std::string error;
  for (bool advanced : {false, true}) {
    const auto start = std::chrono::steady_clock::now();
    if (advanced) {
      EXPECT_FALSE(buffer.canTransform("base_link", cyber::Time(1.0), "missing",
                                       cyber::Time(1.0), "map", 0.005f,
                                       &error));
    } else {
      EXPECT_FALSE(buffer.canTransform("base_link", "missing", cyber::Time(1.0),
                                       0.005f, &error));
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_GE(elapsed, std::chrono::milliseconds(5));
    EXPECT_LT(elapsed, std::chrono::seconds(1));
    EXPECT_DOUBLE_EQ(cyber::Clock::Now().ToSecond(), 10);
  }
  for (float timeout : {-1.0f, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()}) {
    EXPECT_FALSE(buffer.canTransform("base_link", "missing", cyber::Time(1.0),
                                     timeout, &error));
    EXPECT_EQ(error, "invalid transform timeout");
  }
}

TEST_F(MockClockBufferTest, WaitsForDelayedDynamicArrival) {
  Buffer buffer(false);
  std::thread producer([&buffer]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    BufferTestPeer::ReceiveDynamic(buffer, Edge("odom", "base_link", 2));
  });
  EXPECT_TRUE(buffer.canTransform("odom", "base_link", cyber::Time(1.0), 0.2f));
  producer.join();
  EXPECT_DOUBLE_EQ(cyber::Clock::Now().ToSecond(), 10);
}

TEST_F(MockClockBufferTest, EpochResetPreservesStaticAndClearsDynamicHistory) {
  Buffer buffer(false);
  BufferTestPeer::ReceiveStatic(buffer, Edge("base_link", "lidar", 1));
  BufferTestPeer::ReceiveDynamic(buffer, Edge("odom", "base_link", 2));
  const auto epoch = buffer.GetTimeEpoch();
  cyber::Clock::SetNow(cyber::Time(2.0));
  EXPECT_GT(buffer.GetTimeEpoch(), epoch);
  EXPECT_FALSE(buffer.canTransform("odom", "base_link", cyber::Time(1.0), 0));
  TransformStamped transform;
  EXPECT_TRUE(
      buffer.GetLatestStaticTransform("base_link", "lidar", &transform));
  EXPECT_DOUBLE_EQ(
      buffer.lookupTransform("base_link", "lidar", cyber::Time(1.0), 0)
          .transform()
          .translation()
          .x(),
      1);
  const auto next = buffer.GetTimeEpoch();
  buffer.clear();
  EXPECT_GT(buffer.GetTimeEpoch(), next);
  EXPECT_TRUE(
      buffer.GetLatestStaticTransform("base_link", "lidar", &transform));
  BufferTestPeer::ReceiveDynamic(buffer, Edge("odom", "base_link", 5));
  EXPECT_DOUBLE_EQ(buffer.lookupTransform("odom", "lidar", cyber::Time(1.0), 0)
                       .transform()
                       .translation()
                       .x(),
                   6);
}

TEST(StaticBufferTest, ConcurrentCallbacksAndDiagnostics) {
  Buffer buffer(false);
  std::thread static_producer([&buffer]() {
    for (int i = 0; i < 100; ++i) {
      BufferTestPeer::ReceiveStatic(buffer, Edge("base_link", "lidar", 1));
    }
  });
  std::thread dynamic_producer([&buffer]() {
    for (int i = 0; i < 100; ++i) {
      auto message = Edge("odom", "base_link", i);
      message->mutable_transforms(0)->mutable_header()->set_timestamp_sec(i +
                                                                          1);
      BufferTestPeer::ReceiveDynamic(buffer, message);
    }
  });
  for (int i = 0; i < 100; ++i) {
    buffer.GetDiagnosticsSnapshot();
  }
  static_producer.join();
  dynamic_producer.join();
  EXPECT_EQ(buffer.GetDiagnosticsSnapshot().static_transform_count, 100);
  EXPECT_EQ(buffer.GetDiagnosticsSnapshot().dynamic_transform_count, 100);
}

TEST(StaticBufferTest, GroundReferenceUsesBaseZWithTiltedLidar) {
  Buffer buffer(false);
  auto lidar = Edge("base_link", "lidar", 1);
  auto* transform = lidar->mutable_transforms(0)->mutable_transform();
  transform->mutable_translation()->set_z(1.5);
  transform->mutable_rotation()->set_qy(0.7071067811865476);
  transform->mutable_rotation()->set_qw(0.7071067811865476);
  BufferTestPeer::ReceiveStatic(buffer, lidar);
  auto camera = Edge("base_link", "camera", 2);
  camera->mutable_transforms(0)
      ->mutable_transform()
      ->mutable_translation()
      ->set_z(1.2);
  BufferTestPeer::ReceiveStatic(buffer, camera);
  BufferTestPeer::ReceiveDynamic(buffer, Edge("odom", "base_link", 10));
  TransformQuery query(&buffer);
  Eigen::Affine3d ground_from_source;
  ASSERT_TRUE(
      QueryGroundTransform(query, "lidar", 1.8, "camera", &ground_from_source));
  EXPECT_NEAR(ground_from_source.translation().z(), 1.5, 1e-9);
  ASSERT_TRUE(QueryGroundTransform(query, "lidar", 1.8, "base_link",
                                   &ground_from_source));
  EXPECT_NEAR(ground_from_source.translation().z(), 0.3, 1e-9);
  ASSERT_TRUE(
      QueryGroundTransform(query, "lidar", 1.8, "lidar", &ground_from_source));
  EXPECT_NEAR(ground_from_source.translation().z(), 1.8, 1e-9);
  const auto saved = ground_from_source.matrix().eval();
  EXPECT_FALSE(QueryGroundTransform(query, "lidar", 1.8, "unknown",
                                    &ground_from_source));
  EXPECT_FALSE(QueryGroundTransform(query, "lidar", 1.8, "odom",
                                    &ground_from_source, 0.005f));
  EXPECT_FALSE(
      QueryGroundTransform(query, "lidar", -1, "camera", &ground_from_source));
  EXPECT_TRUE(ground_from_source.matrix().isApprox(saved));
}

}  // namespace
}  // namespace transform
}  // namespace apollo
