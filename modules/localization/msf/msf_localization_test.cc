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
#include "modules/localization/msf/msf_localization.h"

#include "gtest/gtest.h"
#include "modules/common/map/map_selection.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace localization {
namespace msf {

class MSFLocalizationTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    apollo::common::MapSelection::SetTestMapDirectory(
        "assets/sites/map/borregas_ave");
    msf_localizatoin_.reset(new MSFLocalization());
  }

  void TearDown() override {
    apollo::common::MapSelection::ClearTestMapDirectory();
  }

 protected:
  std::unique_ptr<MSFLocalization> msf_localizatoin_;
};

TEST_F(MSFLocalizationTest, InitParams) {
  FLAGS_imu_delay_time_threshold_1 = 5;
  msf_localizatoin_->InitParams();
  double imu_delay_time_threshold =
      (msf_localizatoin_->localization_param_).imu_delay_time_threshold_1;
  ASSERT_LT(std::abs(imu_delay_time_threshold - 5.0), 1e-5);
}

}  // namespace msf
}  // namespace localization
}  // namespace apollo
