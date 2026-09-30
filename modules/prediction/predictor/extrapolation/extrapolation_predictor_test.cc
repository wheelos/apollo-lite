// Copyright 2026 WheelOS All Rights Reserved.
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

#include "modules/prediction/predictor/extrapolation/extrapolation_predictor.h"

#include <cmath>

#include "gtest/gtest.h"

#include "wheelos_msgs/prediction_msgs/feature.pb.h"

#include "modules/prediction/common/prediction_gflags.h"

namespace apollo {
namespace prediction {

class ExtrapolationPredictorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    original_time_length_ = FLAGS_prediction_trajectory_time_length;
    original_time_resolution_ = FLAGS_prediction_trajectory_time_resolution;
    FLAGS_prediction_trajectory_time_length = 8.0;
    FLAGS_prediction_trajectory_time_resolution = 0.1;
  }

  void TearDown() override {
    FLAGS_prediction_trajectory_time_length = original_time_length_;
    FLAGS_prediction_trajectory_time_resolution = original_time_resolution_;
  }

  void ExtrapolateFreeMove(Trajectory* trajectory) {
    ExtrapolationPredictor predictor;
    predictor.ExtrapolateByFreeMove(5, 5.0, trajectory);
  }

 private:
  double original_time_length_ = 0.0;
  double original_time_resolution_ = 0.0;
};

TEST_F(ExtrapolationPredictorTest, FreeMoveIncludesExactPredictionHorizon) {
  constexpr int kHiVTFutureSteps = 30;

  Feature feature;
  Trajectory* trajectory = feature.add_predicted_trajectory();
  for (int i = 1; i <= kHiVTFutureSteps; ++i) {
    auto* point = trajectory->add_trajectory_point();
    point->mutable_path_point()->set_x(i * 0.5);
    point->mutable_path_point()->set_y(0.0);
    point->mutable_path_point()->set_theta(0.0);
    point->set_relative_time(i * FLAGS_prediction_trajectory_time_resolution);
    point->set_v(5.0);
    point->set_a(0.0);
  }
  trajectory->mutable_trajectory_point(kHiVTFutureSteps - 1)
      ->set_relative_time(std::nextafter(3.0, 4.0));

  ExtrapolateFreeMove(feature.mutable_predicted_trajectory(0));

  constexpr int kExpectedExtendedPoints = kHiVTFutureSteps + 50;
  ASSERT_EQ(trajectory->trajectory_point_size(), kExpectedExtendedPoints);
  EXPECT_DOUBLE_EQ(
      trajectory->trajectory_point(kExpectedExtendedPoints - 1).relative_time(),
      8.0);
  for (int i = 0; i < trajectory->trajectory_point_size(); ++i) {
    const auto& point = trajectory->trajectory_point(i);
    EXPECT_NEAR(point.relative_time(), (i + 1) * 0.1, 1e-9);
    EXPECT_TRUE(std::isfinite(point.path_point().x()));
    EXPECT_TRUE(std::isfinite(point.path_point().y()));
    EXPECT_TRUE(std::isfinite(point.v()));
    EXPECT_TRUE(std::isfinite(point.a()));
    if (i > 0) {
      EXPECT_GT(point.relative_time(),
                trajectory->trajectory_point(i - 1).relative_time());
    }
  }
}

}  // namespace prediction
}  // namespace apollo
