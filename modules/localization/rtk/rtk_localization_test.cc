/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/localization/rtk/rtk_localization.h"

#include <cmath>
#include <memory>
#include <string>

#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/init.h"
#include "modules/common/util/util.h"
#include "wheelos_msgs/sensor_msgs/gnss_best_pose.pb.h"

namespace apollo {
namespace localization {

class RTKLocalizationTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    cyber::Init("rtk_localization_test");
    rtk_localization_.reset(new RTKLocalization());
  }

 protected:
  template <class T>
  void load_data(const std::string &filename, T *data) {
    ACHECK(cyber::common::GetProtoFromFile(filename, data))
        << "Failed to open file " << filename;
  }

  std::unique_ptr<RTKLocalization> rtk_localization_;
};

TEST_F(RTKLocalizationTest, InterpolateIMU) {
  // timestamp in between + time_diff is big enough(>0.001), interpolate
  {
    apollo::localization::CorrectedImu imu1;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &imu1);

    apollo::localization::CorrectedImu imu2;
    load_data("modules/localization/testdata/1_imu_2.pb.txt", &imu2);

    apollo::localization::CorrectedImu expected_result;
    load_data("modules/localization/testdata/1_imu_result.pb.txt",
              &expected_result);

    apollo::localization::CorrectedImu imu;
    double timestamp = 1173545122.69;
    rtk_localization_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }

  // timestamp in between + time_diff is too small(<0.001), no interpolate
  {
    apollo::localization::CorrectedImu imu1;
    load_data("modules/localization/testdata/2_imu_1.pb.txt", &imu1);

    apollo::localization::CorrectedImu imu2;
    load_data("modules/localization/testdata/2_imu_2.pb.txt", &imu2);

    apollo::localization::CorrectedImu expected_result;
    load_data("modules/localization/testdata/2_imu_result.pb.txt",
              &expected_result);

    apollo::localization::CorrectedImu imu;
    double timestamp = 1173545122.2001;
    rtk_localization_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }

  // timestamp < imu1.timestamp
  {
    apollo::localization::CorrectedImu imu1;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &imu1);

    apollo::localization::CorrectedImu imu2;
    load_data("modules/localization/testdata/1_imu_2.pb.txt", &imu2);

    apollo::localization::CorrectedImu expected_result;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &expected_result);

    apollo::localization::CorrectedImu imu;
    double timestamp = 1173545122;
    rtk_localization_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }

  // timestamp > imu2.timestamp
  {
    apollo::localization::CorrectedImu imu1;
    load_data("modules/localization/testdata/1_imu_1.pb.txt", &imu1);

    apollo::localization::CorrectedImu imu2;
    load_data("modules/localization/testdata/1_imu_2.pb.txt", &imu2);

    apollo::localization::CorrectedImu expected_result;
    load_data("modules/localization/testdata/1_imu_2.pb.txt", &expected_result);

    apollo::localization::CorrectedImu imu;
    double timestamp = 1173545122.70;
    rtk_localization_->InterpolateIMU(imu1, imu2, timestamp, &imu);

    EXPECT_EQ(expected_result.DebugString(), imu.DebugString());
  }
}

TEST_F(RTKLocalizationTest, ComposeLocalizationMsg) {
  {
    apollo::localization::Gps gps;
    load_data("modules/localization/testdata/3_gps_1.pb.txt", &gps);

    apollo::localization::CorrectedImu imu;
    load_data("modules/localization/testdata/3_imu_1.pb.txt", &imu);

    apollo::localization::LocalizationEstimate expected_result;
    load_data("modules/localization/testdata/3_localization_result_2.pb.txt",
              &expected_result);

    apollo::localization::LocalizationEstimate localization;
    rtk_localization_->ComposeLocalizationMsg(gps, imu, &localization);

    EXPECT_EQ(1, localization.header().sequence_num());
    EXPECT_STREQ("localization", localization.header().module_name().c_str());
    EXPECT_NEAR(expected_result.pose().position().x(),
                localization.pose().position().x(), 1.0e-7);
    EXPECT_NEAR(expected_result.pose().position().y(),
                localization.pose().position().y(), 1.0e-7);
    EXPECT_NEAR(expected_result.pose().position().z(),
                localization.pose().position().z(), 1.0e-7);
  }

  // TODO(Qi Luo) Update test once got new imu data for euler angle.
}

TEST_F(RTKLocalizationTest, InterpolateEulerAngle) {
  // Test angle wrap-around across pi and -pi
  apollo::common::Point3D p1;
  p1.set_x(3.10);   // ~ 177.6 deg
  p1.set_y(0.1);
  p1.set_z(-3.10);  // ~ -177.6 deg

  apollo::common::Point3D p2;
  p2.set_x(-3.10);  // ~ -177.6 deg
  p2.set_y(0.3);
  p2.set_z(3.10);   // ~ 177.6 deg

  auto mid = rtk_localization_->InterpolateEulerAngle(p1, p2, 0.5);
  // Shortest path between 3.10 and -3.10 goes through pi/-pi
  EXPECT_NEAR(std::abs(mid.x()), M_PI, 0.05);
  EXPECT_NEAR(mid.y(), 0.2, 1.0e-5);
  EXPECT_NEAR(std::abs(mid.z()), M_PI, 0.05);
}

TEST_F(RTKLocalizationTest, FindMatchingIMU) {
  CorrectedImu result;
  // Empty queue should return false
  EXPECT_FALSE(rtk_localization_->FindMatchingIMU(100.0, &result));

  // Push IMU messages
  auto imu1 = std::make_shared<CorrectedImu>();
  imu1->mutable_header()->set_timestamp_sec(100.0);
  imu1->mutable_imu()->mutable_linear_acceleration()->set_x(1.0);
  rtk_localization_->ImuCallback(imu1);

  auto imu2 = std::make_shared<CorrectedImu>();
  imu2->mutable_header()->set_timestamp_sec(102.0);
  imu2->mutable_imu()->mutable_linear_acceleration()->set_x(3.0);
  rtk_localization_->ImuCallback(imu2);

  // Match in between (interpolated)
  EXPECT_TRUE(rtk_localization_->FindMatchingIMU(101.0, &result));
  EXPECT_NEAR(result.imu().linear_acceleration().x(), 2.0, 1.0e-5);
}

TEST_F(RTKLocalizationTest, FindNearestGpsStatus) {
  drivers::gnss::InsStat status;
  // Empty list returns false
  EXPECT_FALSE(rtk_localization_->FindNearestGpsStatus(100.0, &status));

  auto s1 = std::make_shared<drivers::gnss::InsStat>();
  s1->mutable_header()->set_timestamp_sec(100.0);
  s1->set_pos_type(
      static_cast<uint32_t>(drivers::gnss::SolutionType::INS_RTKFIXED));
  rtk_localization_->GpsStatusCallback(s1);

  // Close timestamp returns true
  EXPECT_TRUE(rtk_localization_->FindNearestGpsStatus(100.2, &status));
  EXPECT_EQ(status.pos_type(),
            static_cast<uint32_t>(drivers::gnss::SolutionType::INS_RTKFIXED));

  // Beyond threshold (> 1.0s) returns false
  EXPECT_FALSE(rtk_localization_->FindNearestGpsStatus(102.5, &status));
}

}  // namespace localization
}  // namespace apollo
