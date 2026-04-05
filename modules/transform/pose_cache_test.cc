#include "modules/transform/pose_cache.h"

#include <map>
#include <stdexcept>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

namespace apollo {
namespace transform {

namespace {

class MockBuffer : public BufferInterface {
 public:
  using Key = std::tuple<std::string, std::string, uint64_t>;

  void AddTransform(const std::string& target_frame,
                    const std::string& source_frame, double timestamp_sec,
                    const Eigen::Affine3d& transform) {
    const auto key = Key(target_frame, source_frame,
                         cyber::Time(timestamp_sec).ToNanosecond());
    TransformStamped stamped;
    stamped.mutable_header()->set_frame_id(target_frame);
    stamped.mutable_header()->set_timestamp_sec(timestamp_sec);
    stamped.set_child_frame_id(source_frame);
    stamped.mutable_transform()->mutable_translation()->set_x(
        transform.translation().x());
    stamped.mutable_transform()->mutable_translation()->set_y(
        transform.translation().y());
    stamped.mutable_transform()->mutable_translation()->set_z(
        transform.translation().z());
    const Eigen::Quaterniond rotation(transform.linear());
    stamped.mutable_transform()->mutable_rotation()->set_qw(rotation.w());
    stamped.mutable_transform()->mutable_rotation()->set_qx(rotation.x());
    stamped.mutable_transform()->mutable_rotation()->set_qy(rotation.y());
    stamped.mutable_transform()->mutable_rotation()->set_qz(rotation.z());
    transforms_[key] = stamped;
  }

  TransformStamped lookupTransform(const std::string& target_frame,
                                   const std::string& source_frame,
                                   const cyber::Time& time,
                                   const float timeout_second) const override {
    (void)timeout_second;
    const auto key = Key(target_frame, source_frame, time.ToNanosecond());
    const auto it = transforms_.find(key);
    if (it == transforms_.end()) {
      throw std::runtime_error("missing transform");
    }
    return it->second;
  }

  TransformStamped lookupTransform(const std::string& target_frame,
                                   const cyber::Time& target_time,
                                   const std::string& source_frame,
                                   const cyber::Time& source_time,
                                   const std::string& fixed_frame,
                                   const float timeout_second) const override {
    (void)target_time;
    (void)fixed_frame;
    return lookupTransform(target_frame, source_frame, source_time,
                           timeout_second);
  }

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame, const cyber::Time& time,
                    const float timeout_second,
                    std::string* errstr) const override {
    (void)timeout_second;
    const auto key = Key(target_frame, source_frame, time.ToNanosecond());
    const bool found = transforms_.count(key) > 0;
    if (!found && errstr != nullptr) {
      *errstr = "mock transform missing";
    }
    return found;
  }

  bool canTransform(const std::string& target_frame,
                    const cyber::Time& target_time,
                    const std::string& source_frame,
                    const cyber::Time& source_time,
                    const std::string& fixed_frame, const float timeout_second,
                    std::string* errstr) const override {
    (void)target_time;
    (void)fixed_frame;
    return canTransform(target_frame, source_frame, source_time, timeout_second,
                        errstr);
  }

 private:
  std::map<Key, TransformStamped> transforms_;
};

}  // namespace

TEST(PoseCacheTest, InterpolatesBetweenCachedSamples) {
  PoseCache cache(2.0);
  ASSERT_TRUE(cache.Insert(10.0, Eigen::Translation3d(0.0, 0.0, 0.0) *
                                     Eigen::Quaterniond::Identity()));
  ASSERT_TRUE(cache.Insert(12.0, Eigen::Translation3d(2.0, 0.0, 0.0) *
                                     Eigen::Quaterniond::Identity()));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  PoseCacheStatus status = PoseCacheStatus::kEmpty;
  ASSERT_TRUE(cache.Query(11.0, 0.2, &pose, &status));
  EXPECT_EQ(status, PoseCacheStatus::kOk);
  EXPECT_DOUBLE_EQ(pose.translation().x(), 1.0);
}

TEST(PoseCacheTest, RejectsQueriesBeyondExtrapolationWindow) {
  PoseCache cache(2.0);
  ASSERT_TRUE(cache.Insert(10.0, Eigen::Translation3d(1.0, 0.0, 0.0) *
                                     Eigen::Quaterniond::Identity()));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  PoseCacheStatus status = PoseCacheStatus::kEmpty;
  EXPECT_FALSE(cache.Query(10.5, 0.1, &pose, &status));
  EXPECT_EQ(status, PoseCacheStatus::kTooOld);
}

TEST(TransformFrameCacheTest, PrefetchesAndServesCachedInterpolatedPoses) {
  MockBuffer buffer;
  buffer.AddTransform(
      "world", "lidar", 10.0,
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  buffer.AddTransform(
      "world", "lidar", 12.0,
      Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  TransformFrameCache cache(&buffer, "world", "lidar",
                            PoseCacheOptions{2.0, 0.2, 0.01f});
  ASSERT_TRUE(cache.PrefetchBatch({12.0, 10.0}));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  PoseCacheStatus status = PoseCacheStatus::kEmpty;
  ASSERT_TRUE(cache.QueryCached(11.0, &pose, &status));
  EXPECT_EQ(status, PoseCacheStatus::kOk);
  EXPECT_DOUBLE_EQ(pose.translation().x(), 1.0);
}

TEST(TransformFrameCacheTest, StoresOutOfOrderPrefetchesInSortedWindow) {
  MockBuffer buffer;
  buffer.AddTransform(
      "world", "lidar", 10.0,
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  buffer.AddTransform(
      "world", "lidar", 11.0,
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  TransformFrameCache cache(&buffer, "world", "lidar",
                            PoseCacheOptions{2.0, 0.2, 0.01f});
  ASSERT_TRUE(cache.Prefetch(11.0));
  ASSERT_TRUE(cache.Prefetch(10.0));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  ASSERT_TRUE(cache.QueryCached(10.5, &pose));
  EXPECT_DOUBLE_EQ(pose.translation().x(), 0.5);
}

TEST(TransformFrameCacheTest,
     PrefetchBatchDoesNotTreatForwardExtrapolationAsCacheHit) {
  MockBuffer buffer;
  buffer.AddTransform(
      "world", "lidar", 10.0,
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  buffer.AddTransform(
      "world", "lidar", 10.1,
      Eigen::Translation3d(1.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  TransformFrameCache cache(&buffer, "world", "lidar",
                            PoseCacheOptions{2.0, 0.2, 0.01f});
  ASSERT_TRUE(cache.PrefetchBatch({10.0, 10.1}));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  ASSERT_TRUE(cache.QueryCachedStrict(10.1, &pose));
  EXPECT_DOUBLE_EQ(pose.translation().x(), 1.0);
}

TEST(TransformFrameCacheTest, StrictQueryRejectsForwardExtrapolationOnlyCache) {
  TransformFrameCache cache(nullptr, "world", "lidar",
                            PoseCacheOptions{2.0, 0.2, 0.01f});
  ASSERT_TRUE(cache.StorePose(
      10.0, Eigen::Translation3d(1.0, 0.0, 0.0) *
                Eigen::Quaterniond::Identity()));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  PoseCacheStatus status = PoseCacheStatus::kEmpty;
  EXPECT_FALSE(cache.QueryCachedStrict(10.1, &pose, &status));
  EXPECT_EQ(status, PoseCacheStatus::kTooOld);
  ASSERT_TRUE(cache.QueryCached(10.1, &pose, &status));
  EXPECT_EQ(status, PoseCacheStatus::kOk);
  EXPECT_DOUBLE_EQ(pose.translation().x(), 1.0);
}

}  // namespace transform
}  // namespace apollo
