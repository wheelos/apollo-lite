/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/transform/timed_transform_resolver.h"

#include <map>
#include <tuple>

#include "gtest/gtest.h"

#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace transform {
namespace {

TransformStamped MakeTransformMessage(double timestamp_sec, double x, double y,
                                      double z) {
  TransformStamped transform;
  transform.mutable_header()->set_timestamp_sec(timestamp_sec);
  transform.mutable_header()->set_frame_id("map");
  transform.set_child_frame_id("base_link");
  transform.mutable_transform()->mutable_translation()->set_x(x);
  transform.mutable_transform()->mutable_translation()->set_y(y);
  transform.mutable_transform()->mutable_translation()->set_z(z);
  transform.mutable_transform()->mutable_rotation()->set_qw(1.0);
  transform.mutable_transform()->mutable_rotation()->set_qx(0.0);
  transform.mutable_transform()->mutable_rotation()->set_qy(0.0);
  transform.mutable_transform()->mutable_rotation()->set_qz(0.0);
  return transform;
}

class FakeBuffer : public BufferInterface {
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
                                   float timeout_second) const override {
    last_target_frame_ = target_frame;
    last_source_frame_ = source_frame;
    last_query_time_ = time;
    last_timeout_sec_ = timeout_second;
    ++lookup_call_count_;
    const auto it = transforms_.find(
        Key(target_frame, source_frame, time.ToNanosecond()));
    if (it != transforms_.end()) {
      return it->second;
    }
    return lookup_transform_;
  }

  TransformStamped lookupTransform(const std::string& target_frame,
                                   const cyber::Time& target_time,
                                   const std::string& source_frame,
                                   const cyber::Time& source_time,
                                   const std::string& fixed_frame,
                                   float timeout_second) const override {
    last_target_frame_ = target_frame;
    last_source_frame_ = source_frame;
    last_target_time_ = target_time;
    last_source_time_ = source_time;
    last_fixed_frame_ = fixed_frame;
    last_timeout_sec_ = timeout_second;
    ++lookup_call_count_;
    return lookup_transform_;
  }

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame, const cyber::Time& time,
                    float timeout_second,
                    std::string* errstr) const override {
    last_target_frame_ = target_frame;
    last_source_frame_ = source_frame;
    last_query_time_ = time;
    last_timeout_sec_ = timeout_second;
    ++can_transform_call_count_;
    if (transforms_.count(
            Key(target_frame, source_frame, time.ToNanosecond())) > 0) {
      return true;
    }
    if (!can_transform_result_ && errstr != nullptr) {
      *errstr = "can transform failed";
    }
    return can_transform_result_;
  }

  bool canTransform(const std::string& target_frame,
                    const cyber::Time& target_time,
                    const std::string& source_frame,
                    const cyber::Time& source_time,
                    const std::string& fixed_frame, float timeout_second,
                    std::string* errstr) const override {
    last_target_frame_ = target_frame;
    last_source_frame_ = source_frame;
    last_target_time_ = target_time;
    last_source_time_ = source_time;
    last_fixed_frame_ = fixed_frame;
    last_timeout_sec_ = timeout_second;
    ++can_transform_call_count_;
    if (!can_transform_result_ && errstr != nullptr) {
      *errstr = "can transform failed";
    }
    return can_transform_result_;
  }

  bool GetLatestStaticTransform(const std::string& target_frame,
                                const std::string& source_frame,
                                TransformStamped* transform) const override {
    last_target_frame_ = target_frame;
    last_source_frame_ = source_frame;
    *transform = static_transform_;
    return true;
  }

  mutable std::string last_target_frame_;
  mutable std::string last_source_frame_;
  mutable std::string last_fixed_frame_;
  mutable cyber::Time last_query_time_;
  mutable cyber::Time last_target_time_;
  mutable cyber::Time last_source_time_;
  mutable float last_timeout_sec_ = 0.0f;
  mutable int can_transform_call_count_ = 0;
  mutable int lookup_call_count_ = 0;

  bool can_transform_result_ = true;
  TransformStamped lookup_transform_;
  TransformStamped static_transform_;
  std::map<Key, TransformStamped> transforms_;
};

TEST(TimedTransformResolverTest, QueryCachedInterpolatesBetweenSamples) {
  FakeBuffer buffer;
  buffer.AddTransform(
      "map", "base_link", 10.0,
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  buffer.AddTransform(
      "map", "base_link", 12.0,
      Eigen::Translation3d(10.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  TimedTransformResolver resolver(
      &buffer, "map", "base_link",
      TimedTransformResolverOptions{0.01f, 5.0, 1.0, 0.015, true, true});
  ASSERT_TRUE(resolver.PrefetchBatch({10.0, 12.0}));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformResolveStatus status = TransformResolveStatus::kEmpty;
  EXPECT_TRUE(resolver.QueryCached(11.0, &pose, &status));
  EXPECT_EQ(status, TransformResolveStatus::kOk);
  EXPECT_NEAR(pose.translation().x(), 5.0, 1e-9);
}

TEST(TimedTransformResolverTest, QueryCachedExtrapolatesWithinLimit) {
  FakeBuffer buffer;
  buffer.AddTransform(
      "map", "base_link", 10.0,
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());
  buffer.AddTransform(
      "map", "base_link", 12.0,
      Eigen::Translation3d(10.0, 0.0, 0.0) * Eigen::Quaterniond::Identity());

  TimedTransformResolver resolver(
      &buffer, "map", "base_link",
      TimedTransformResolverOptions{0.01f, 5.0, 1.0, 0.015, true, true});
  ASSERT_TRUE(resolver.PrefetchBatch({10.0, 12.0}));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformResolveStatus status = TransformResolveStatus::kEmpty;
  EXPECT_FALSE(resolver.QueryCached(13.5, &pose, &status));
  EXPECT_EQ(status, TransformResolveStatus::kTooOld);
  EXPECT_TRUE(resolver.QueryCached(12.5, &pose, &status));
  EXPECT_EQ(status, TransformResolveStatus::kOk);
  EXPECT_NEAR(pose.translation().x(), 12.5, 1e-9);
}

TEST(TimedTransformResolverTest, ResolveUsesInjectedTransformQueryOnce) {
  FakeBuffer buffer;
  buffer.lookup_transform_ = MakeTransformMessage(20.0, 1.0, 2.0, 3.0);

  TransformQuery query(&buffer);
  TimedTransformResolver resolver(&query);
  TimedTransformResolverOptions options;
  options.enable_extrapolation = false;
  resolver.ConfigureFrames("map", "base_link");
  resolver.SetOptions(options);

  StampedTransform resolved;
  EXPECT_TRUE(resolver.Resolve(20.0, &resolved));
  EXPECT_EQ(buffer.can_transform_call_count_, 1);
  EXPECT_EQ(buffer.lookup_call_count_, 1);
  EXPECT_EQ(buffer.last_target_frame_, "map");
  EXPECT_EQ(buffer.last_source_frame_, "base_link");
  EXPECT_NEAR(resolved.translation.x(), 1.0, 1e-9);
  EXPECT_NEAR(resolved.translation.y(), 2.0, 1e-9);
  EXPECT_NEAR(resolved.translation.z(), 3.0, 1e-9);
}

}  // namespace
}  // namespace transform
}  // namespace apollo