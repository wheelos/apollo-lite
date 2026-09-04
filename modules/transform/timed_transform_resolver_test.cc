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

#include <cstdlib>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "tools/cpp/runfiles/runfiles.h"

#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace transform {
namespace {

StampedTransform MakeStampedTransform(double timestamp_sec, double x, double y,
                                      double z) {
  StampedTransform transform;
  transform.timestamp = timestamp_sec;
  transform.translation = Eigen::Translation3d(x, y, z);
  transform.rotation = Eigen::Quaterniond::Identity();
  return transform;
}

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
  TransformStamped lookupTransform(const std::string& target_frame,
                                   const std::string& source_frame,
                                   const cyber::Time& time,
                                   float timeout_second) const override {
    last_target_frame_ = target_frame;
    last_source_frame_ = source_frame;
    last_query_time_ = time;
    last_timeout_sec_ = timeout_second;
    ++lookup_call_count_;
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
};

TEST(TransformCacheTest, InterpolatesBetweenSamples) {
  TransformCache cache;
  cache.SetCacheDuration(5.0);
  cache.AddTransform(MakeStampedTransform(10.0, 0.0, 0.0, 0.0));
  cache.AddTransform(MakeStampedTransform(12.0, 10.0, 0.0, 0.0));

  StampedTransform resolved;
  EXPECT_TRUE(cache.QueryTransform(11.0, &resolved, 0.0));
  EXPECT_DOUBLE_EQ(resolved.timestamp, 11.0);
  EXPECT_NEAR(resolved.translation.x(), 5.0, 1e-9);
}

TEST(TransformCacheTest, RejectsExtrapolationBeyondLimit) {
  TransformCache cache;
  cache.SetCacheDuration(5.0);
  cache.AddTransform(MakeStampedTransform(10.0, 0.0, 0.0, 0.0));
  cache.AddTransform(MakeStampedTransform(12.0, 10.0, 0.0, 0.0));

  StampedTransform resolved;
  EXPECT_FALSE(cache.QueryTransform(13.5, &resolved, 1.0));
  EXPECT_TRUE(cache.QueryTransform(12.5, &resolved, 1.0));
  EXPECT_NEAR(resolved.translation.x(), 12.5, 1e-9);
}

TEST(TimedTransformResolverTest, ResolveUsesInjectedTransformQueryOnce) {
  FakeBuffer buffer;
  buffer.lookup_transform_ = MakeTransformMessage(20.0, 1.0, 2.0, 3.0);

  TransformQuery query(&buffer);
  TimedTransformResolver resolver(&query);
  TimedTransformResolverOptions options;
  options.enable_extrapolation = false;
  resolver.SetOptions(options);

  StampedTransform resolved;
  EXPECT_TRUE(resolver.Resolve(20.0, "map", "base_link", &resolved));
  EXPECT_EQ(buffer.can_transform_call_count_, 1);
  EXPECT_EQ(buffer.lookup_call_count_, 1);
  EXPECT_EQ(buffer.last_target_frame_, "map");
  EXPECT_EQ(buffer.last_source_frame_, "base_link");
  EXPECT_NEAR(resolved.translation.x(), 1.0, 1e-9);
  EXPECT_NEAR(resolved.translation.y(), 2.0, 1e-9);
  EXPECT_NEAR(resolved.translation.z(), 3.0, 1e-9);

  const auto diagnostics = resolver.GetDiagnosticsSnapshot();
  EXPECT_EQ(diagnostics.resolve_calls, 1);
  EXPECT_EQ(diagnostics.tf2_lookup_success, 1);
  EXPECT_EQ(diagnostics.last_status, TimedTransformResolveStatus::kTf2Lookup);
}

TEST(TimedTransformResolverTest, DiagnosticsTrackLatestFallback) {
  std::string runfiles_error;
  std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles(
      bazel::tools::cpp::runfiles::Runfiles::CreateForTest(
          BAZEL_CURRENT_REPOSITORY, &runfiles_error));
  ASSERT_NE(runfiles, nullptr) << runfiles_error;
  const std::string cyber_config = runfiles->Rlocation(
      "core/cyber/conf/cyber.pb.conf", BAZEL_CURRENT_REPOSITORY);
  ASSERT_FALSE(cyber_config.empty());
  const std::string cyber_path =
      cyber_config.substr(0, cyber_config.size() - std::string(
          "/conf/cyber.pb.conf").size());
  setenv("CYBER_PATH", cyber_path.c_str(), 1);

  FakeBuffer buffer;
  TransformQuery query(&buffer);
  TimedTransformResolver resolver(&query);
  TimedTransformResolverOptions options;
  options.enable_extrapolation = true;
  options.max_extrapolation_latency_sec = 1.0;
  options.hardware_trigger = false;
  options.latest_lookup_fallback_tolerance_sec = 1.0;
  resolver.SetOptions(options);

  buffer.lookup_transform_ = MakeTransformMessage(10.0, 1.0, 0.0, 0.0);
  StampedTransform resolved;
  EXPECT_TRUE(resolver.Resolve(10.0, "map", "base_link", &resolved));

  buffer.lookup_transform_ = MakeTransformMessage(12.0, 3.0, 0.0, 0.0);
  EXPECT_TRUE(resolver.Resolve(12.0, "map", "base_link", &resolved));

  buffer.can_transform_result_ = false;
  EXPECT_TRUE(resolver.Resolve(12.5, "map", "base_link", &resolved));
  EXPECT_NEAR(resolved.translation.x(), 3.0, 1e-9);

  const auto diagnostics = resolver.GetDiagnosticsSnapshot();
  EXPECT_EQ(diagnostics.resolve_calls, 3);
  EXPECT_EQ(diagnostics.tf2_lookup_success, 2);
  EXPECT_EQ(diagnostics.latest_fallback_success, 1);
  EXPECT_EQ(diagnostics.last_status,
            TimedTransformResolveStatus::kLatestFallback);
}

TEST(TimedTransformResolverTest, ResolveDirectToAffine) {
  FakeBuffer buffer;
  buffer.lookup_transform_ = MakeTransformMessage(30.0, 4.0, 5.0, 6.0);

  TransformQuery query(&buffer);
  TimedTransformResolver resolver(&query);
  TimedTransformResolverOptions options;
  options.enable_extrapolation = false;
  resolver.SetOptions(options);

  Eigen::Affine3d affine;
  EXPECT_TRUE(resolver.Resolve(30.0, "map", "base_link", &affine));
  EXPECT_NEAR(affine.translation().x(), 4.0, 1e-9);
  EXPECT_NEAR(affine.translation().y(), 5.0, 1e-9);
  EXPECT_NEAR(affine.translation().z(), 6.0, 1e-9);
}

}  // namespace
}  // namespace transform
}  // namespace apollo
