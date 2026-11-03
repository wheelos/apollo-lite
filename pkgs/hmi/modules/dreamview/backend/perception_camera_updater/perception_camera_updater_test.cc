// Copyright 2026 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "modules/dreamview/backend/perception_camera_updater/perception_camera_updater.h"

#include "gtest/gtest.h"

namespace apollo {
namespace dreamview {

class PerceptionCameraUpdaterTestPeer {
 public:
  static void SetEnabled(PerceptionCameraUpdater* updater, bool enabled) {
    updater->enabled_ = enabled;
  }

  static void OnLocalization(
      PerceptionCameraUpdater* updater,
      const std::shared_ptr<apollo::localization::LocalizationEstimate>&
          localization) {
    updater->OnLocalization(localization);
  }

  static bool Enabled(const PerceptionCameraUpdater& updater) {
    return updater.enabled_;
  }

  static size_t LocalizationQueueSize(const PerceptionCameraUpdater& updater) {
    return updater.localization_queue_.size();
  }

  static double OldestMeasurementTime(const PerceptionCameraUpdater& updater) {
    return updater.localization_queue_.front()->measurement_time();
  }

  static double LatestMeasurementTime(const PerceptionCameraUpdater& updater) {
    return updater.localization_queue_.back()->measurement_time();
  }

  static size_t MaxLocalizationQueueSize() {
    return PerceptionCameraUpdater::kMaxLocalizationQueueSize;
  }
};

class PerceptionCameraUpdaterTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    cyber::GlobalData::Instance()->EnableSimulationMode();
  }
};

TEST_F(PerceptionCameraUpdaterTest, StartDoesNotEnableQueueing) {
  PerceptionCameraUpdater updater(nullptr);
  updater.Start([](const std::string&) { return true; });

  auto localization =
      std::make_shared<apollo::localization::LocalizationEstimate>();
  localization->set_measurement_time(1.0);
  PerceptionCameraUpdaterTestPeer::OnLocalization(&updater, localization);

  EXPECT_FALSE(PerceptionCameraUpdaterTestPeer::Enabled(updater));
  EXPECT_EQ(0u, PerceptionCameraUpdaterTestPeer::LocalizationQueueSize(updater));
}

TEST_F(PerceptionCameraUpdaterTest, LocalizationQueueIsBounded) {
  PerceptionCameraUpdater updater(nullptr);
  PerceptionCameraUpdaterTestPeer::SetEnabled(&updater, true);

  for (size_t i = 0;
       i < PerceptionCameraUpdaterTestPeer::MaxLocalizationQueueSize() + 50;
       ++i) {
    auto localization =
        std::make_shared<apollo::localization::LocalizationEstimate>();
    localization->set_measurement_time(static_cast<double>(i));
    PerceptionCameraUpdaterTestPeer::OnLocalization(&updater, localization);
  }

  ASSERT_EQ(PerceptionCameraUpdaterTestPeer::MaxLocalizationQueueSize(),
            PerceptionCameraUpdaterTestPeer::LocalizationQueueSize(updater));
  EXPECT_DOUBLE_EQ(50.0,
                   PerceptionCameraUpdaterTestPeer::OldestMeasurementTime(
                       updater));
  EXPECT_DOUBLE_EQ(
      static_cast<double>(
          PerceptionCameraUpdaterTestPeer::MaxLocalizationQueueSize() + 49),
      PerceptionCameraUpdaterTestPeer::LatestMeasurementTime(updater));
}

}  // namespace dreamview
}  // namespace apollo
