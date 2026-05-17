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

#include <string>

#include "Eigen/Geometry"
#include "gtest/gtest.h"
#include "tf2/exceptions.h"

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
      throw tf2::LookupException("lookup failed");
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
      throw tf2::LookupException("lookup failed");
    }
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
    if (!has_static_transform_) {
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
}

TEST(TransformQueryTest, StaticLookupUsesStaticInterface) {
  FakeBuffer buffer;
  buffer.static_transform_ = MakeTransformStamped(
      0.0, 4.0, 5.0, 6.0, Eigen::Quaterniond::Identity());

  TransformQuery query(&buffer);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  EXPECT_TRUE(
      query.GetLatestStaticTransformToAffine("base_link", "velodyne128",
                                             &transform));
  EXPECT_EQ(buffer.last_target_frame_, "base_link");
  EXPECT_EQ(buffer.last_source_frame_, "velodyne128");
  EXPECT_NEAR(transform.translation().x(), 4.0, 1e-9);
  EXPECT_NEAR(transform.translation().y(), 5.0, 1e-9);
  EXPECT_NEAR(transform.translation().z(), 6.0, 1e-9);
}

}  // namespace
}  // namespace transform
}  // namespace apollo