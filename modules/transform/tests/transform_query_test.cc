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

#include "modules/transform/transform_query.h"

#include <chrono>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "Eigen/Geometry"
#include "nlohmann/json.hpp"

#include "modules/transform/buffer_interface.h"

namespace apollo {
namespace transform {
namespace {

TransformStamped MakeTransformStamped(double timestamp_sec, double x, double y,
                                      double z,
                                      const Eigen::Quaterniond& rotation) {
  TransformStamped transform;
  transform.mutable_header()->set_timestamp_sec(timestamp_sec);
  transform.mutable_header()->set_frame_id("target");
  transform.set_child_frame_id("source");
  transform.mutable_transform()->mutable_translation()->set_x(x);
  transform.mutable_transform()->mutable_translation()->set_y(y);
  transform.mutable_transform()->mutable_translation()->set_z(z);
  transform.mutable_transform()->mutable_rotation()->set_qw(rotation.w());
  transform.mutable_transform()->mutable_rotation()->set_qx(rotation.x());
  transform.mutable_transform()->mutable_rotation()->set_qy(rotation.y());
  transform.mutable_transform()->mutable_rotation()->set_qz(rotation.z());
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
    if (throw_lookup_) {
      throw TransformException("lookup failed");
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
    if (throw_lookup_) {
      throw TransformException("lookup failed");
    }
    return lookup_transform_;
  }

  bool canTransform(const std::string& target_frame,
                    const std::string& source_frame, const cyber::Time& time,
                    float timeout_second, std::string* errstr) const override {
    last_target_frame_ = target_frame;
    last_source_frame_ = source_frame;
    last_query_time_ = time;
    last_timeout_sec_ = timeout_second;
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
    ++static_attempts_;
    if (static_attempts_ == 1) {
      static_ready_at_ = std::chrono::steady_clock::now() + static_delay_;
    }
    if (!has_static_transform_ ||
        std::chrono::steady_clock::now() < static_ready_at_) {
      return false;
    }
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
  mutable int static_attempts_ = 0;
  mutable std::chrono::steady_clock::time_point static_ready_at_;
  std::chrono::milliseconds static_delay_{0};

  bool can_transform_result_ = true;
  bool throw_lookup_ = false;
  bool has_static_transform_ = true;
  TransformStamped lookup_transform_;
  TransformStamped static_transform_;
};

TEST(TransformQueryTest, LookupTransformToAffineUsesInjectedBuffer) {
  FakeBuffer buffer;
  const Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  buffer.lookup_transform_ =
      MakeTransformStamped(12.0, 1.0, 2.0, 3.0, rotation);

  TransformQuery query(&buffer);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  std::string err_msg;
  EXPECT_TRUE(query.LookupTransformToAffine("map", "lidar", cyber::Time(12.0),
                                            &transform, 0.25f, &err_msg));
  EXPECT_TRUE(err_msg.empty());
  EXPECT_EQ(buffer.last_target_frame_, "map");
  EXPECT_EQ(buffer.last_source_frame_, "lidar");
  EXPECT_FLOAT_EQ(buffer.last_timeout_sec_, 0.25f);
  EXPECT_NEAR(transform.translation().x(), 1.0, 1e-9);
  EXPECT_NEAR(transform.translation().y(), 2.0, 1e-9);
  EXPECT_NEAR(transform.translation().z(), 3.0, 1e-9);
  EXPECT_NEAR(transform.linear()(0, 1), -1.0, 1e-9);
  EXPECT_NEAR(transform.linear()(1, 0), 1.0, 1e-9);
}

TEST(TransformQueryTest, LookupTransformReturnsErrorWhenBufferThrows) {
  FakeBuffer buffer;
  buffer.throw_lookup_ = true;

  TransformQuery query(&buffer);
  TransformStamped transform;
  std::string err_msg;
  EXPECT_FALSE(query.LookupTransform("map", "lidar", cyber::Time(9.0),
                                     &transform, 0.1f, &err_msg));
  EXPECT_EQ(err_msg, "lookup failed");
  const auto diagnostics = query.GetDiagnosticsSnapshot();
  EXPECT_EQ(diagnostics.lookup_transform_calls, 1);
  EXPECT_EQ(diagnostics.lookup_transform_success, 0);
  EXPECT_EQ(diagnostics.last_error, "lookup failed");
}

TEST(TransformQueryTest, StaticLookupUsesStaticInterface) {
  FakeBuffer buffer;
  buffer.static_transform_ =
      MakeTransformStamped(0.0, 4.0, 5.0, 6.0, Eigen::Quaterniond::Identity());

  TransformQuery query(&buffer);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  EXPECT_TRUE(query.GetLatestStaticTransformToAffine("base_link", "velodyne128",
                                                     &transform));
  EXPECT_EQ(buffer.last_target_frame_, "base_link");
  EXPECT_EQ(buffer.last_source_frame_, "velodyne128");
  EXPECT_NEAR(transform.translation().x(), 4.0, 1e-9);
  EXPECT_NEAR(transform.translation().y(), 5.0, 1e-9);
  EXPECT_NEAR(transform.translation().z(), 6.0, 1e-9);
}

TEST(TransformQueryTest, DiagnosticsTrackCanLookupAffineAndStaticQueries) {
  FakeBuffer buffer;
  buffer.lookup_transform_ =
      MakeTransformStamped(12.0, 1.0, 2.0, 3.0, Eigen::Quaterniond::Identity());
  buffer.static_transform_ =
      MakeTransformStamped(0.0, 4.0, 5.0, 6.0, Eigen::Quaterniond::Identity());

  TransformQuery query(&buffer);
  EXPECT_TRUE(query.CanTransform("map", "lidar", cyber::Time(12.0)));

  TransformStamped stamped_transform;
  EXPECT_TRUE(query.LookupTransform("map", "lidar", cyber::Time(12.0),
                                    &stamped_transform));

  Eigen::Affine3d affine_transform = Eigen::Affine3d::Identity();
  EXPECT_TRUE(query.LookupTransformToAffine("map", "lidar", cyber::Time(12.0),
                                            &affine_transform));
  EXPECT_TRUE(query.GetLatestStaticTransformToAffine("base_link", "velodyne128",
                                                     &affine_transform));

  const auto diagnostics = query.GetDiagnosticsSnapshot();
  EXPECT_EQ(diagnostics.can_transform_calls, 1);
  EXPECT_EQ(diagnostics.can_transform_success, 1);
  EXPECT_EQ(diagnostics.lookup_transform_calls, 2);
  EXPECT_EQ(diagnostics.lookup_transform_success, 2);
  EXPECT_EQ(diagnostics.affine_lookup_calls, 2);
  EXPECT_EQ(diagnostics.affine_lookup_success, 2);
  EXPECT_EQ(diagnostics.static_lookup_calls, 1);
  EXPECT_EQ(diagnostics.static_lookup_success, 1);
  EXPECT_TRUE(diagnostics.last_error.empty());
}

TEST(TransformQueryTest, WaitsForDelayedStaticArrivalWithoutDynamicLookup) {
  FakeBuffer buffer;
  buffer.static_transform_ =
      MakeTransformStamped(0.0, 1.0, 2.0, 3.0, Eigen::Quaterniond::Identity());
  buffer.static_delay_ = std::chrono::milliseconds(5);
  TransformQuery query(&buffer);
  Eigen::Affine3d transform;
  ASSERT_TRUE(query.GetLatestStaticTransformToAffine("base_link", "camera",
                                                     &transform, 0.1f));
  EXPECT_GT(buffer.static_attempts_, 1);
  EXPECT_DOUBLE_EQ(transform.translation().z(), 3.0);
  const auto diagnostics = query.GetDiagnosticsSnapshot();
  EXPECT_EQ(diagnostics.static_lookup_calls, 1);
  EXPECT_EQ(diagnostics.static_lookup_success, 1);
  EXPECT_EQ(diagnostics.lookup_transform_calls, 0);
}

TEST(TransformQueryTest, StaticTimeoutIsBoundedAndPreservesOutput) {
  FakeBuffer buffer;
  buffer.has_static_transform_ = false;
  TransformQuery query(&buffer);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(query.GetLatestStaticTransformToAffine("base_link", "camera",
                                                      &transform, 0.005f));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_GE(elapsed, std::chrono::milliseconds(5));
  EXPECT_LT(elapsed, std::chrono::seconds(1));
  EXPECT_TRUE(transform.matrix().isIdentity());
  const auto diagnostics = query.GetDiagnosticsSnapshot();
  EXPECT_EQ(diagnostics.static_lookup_calls, 1);
  EXPECT_EQ(diagnostics.static_lookup_success, 0);
  EXPECT_EQ(diagnostics.lookup_transform_calls, 0);
  EXPECT_FALSE(query.GetLatestStaticTransformToAffine("base_link", "camera",
                                                      &transform, -1.0f));
  EXPECT_FALSE(query.GetLatestStaticTransformToAffine(
      "base_link", "camera", &transform,
      std::numeric_limits<float>::quiet_NaN()));
}

TEST(TransformQueryTest, DiagnosticsEscapeJsonStrings) {
  const std::string text = "frame\"\\\n\t\r";
  TransformQueryDiagnostics query;
  query.last_error = text;
  EXPECT_EQ(nlohmann::json::parse(query.ToJsonString())["last_error"], text);
  TimedTransformResolverDiagnostics timed;
  timed.last_error = text;
  EXPECT_EQ(nlohmann::json::parse(timed.ToJsonString())["last_error"], text);
  TransformEdgeDiagnostics edge;
  edge.frame_id = text;
  edge.child_frame_id = text;
  const auto parsed = nlohmann::json::parse(edge.ToJsonString());
  EXPECT_EQ(parsed["frame_id"], text);
  EXPECT_EQ(parsed["child_frame_id"], text);
}

}  // namespace
}  // namespace transform
}  // namespace apollo
