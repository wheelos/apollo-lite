/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include <sstream>

#define private public
#define protected public
#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.h"
#undef protected
#undef private

#include "gtest/gtest.h"

#include "modules/common/math/math_utils.h"
#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "cyber/init.h"
#include "modules/planning/proto/planning_config.pb.h"

namespace apollo {
namespace planning {

class OpenSpaceTrajectoryProviderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    apollo::cyber::Init("open_space_trajectory_provider_test");
    config_.set_task_type(TaskConfig::OPEN_SPACE_TRAJECTORY_PROVIDER);
    auto* planner_config = config_.mutable_open_space_trajectory_provider_config()
                               ->mutable_open_space_trajectory_optimizer_config()
                               ->mutable_planner_open_space_config();
    planner_config->set_is_near_destination_threshold(0.5);
    planner_config->set_is_near_destination_theta_threshold(0.2);
    injector_ = std::make_shared<DependencyInjector>();
    provider_ =
        std::make_unique<OpenSpaceTrajectoryProvider>(config_, injector_);
    frame_ = std::make_unique<Frame>(
        0, LocalView{}, common::TrajectoryPoint{}, common::VehicleState{},
        nullptr);
    provider_->frame_ = frame_.get();
  }

  protected:
  TaskConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
  std::unique_ptr<OpenSpaceTrajectoryProvider> provider_;
  std::unique_ptr<Frame> frame_;
};

TEST_F(OpenSpaceTrajectoryProviderTest, NearDestinationUsesOriginFrame) {
  common::VehicleState vehicle_state;
  const common::math::Vec2d origin(10.0, 20.0);
  const double origin_heading = 0.3;
  const std::vector<double> end_pose = {2.0, -1.0, 0.4, 0.0};

  common::math::Vec2d end_pose_world(end_pose[0], end_pose[1]);
  end_pose_world.SelfRotate(origin_heading);
  end_pose_world += origin;
  vehicle_state.set_x(end_pose_world.x() + 0.05);
  vehicle_state.set_y(end_pose_world.y() - 0.03);
  vehicle_state.set_heading(common::math::NormalizeAngle(end_pose[2] +
                                                         origin_heading +
                                                         0.02));

  EXPECT_TRUE(provider_->IsVehicleNearDestination(vehicle_state, end_pose,
                                                  origin_heading, origin));
  EXPECT_TRUE(frame_->open_space_info().destination_reached());
}

TEST_F(OpenSpaceTrajectoryProviderTest, FarDestinationRemainsFalse) {
  common::VehicleState vehicle_state;
  const common::math::Vec2d origin(10.0, 20.0);
  const double origin_heading = -0.25;
  const std::vector<double> end_pose = {2.0, 1.5, -0.6, 0.0};

  common::math::Vec2d end_pose_world(end_pose[0], end_pose[1]);
  end_pose_world.SelfRotate(origin_heading);
  end_pose_world += origin;
  vehicle_state.set_x(end_pose_world.x() + 2.0);
  vehicle_state.set_y(end_pose_world.y() + 1.0);
  vehicle_state.set_heading(common::math::NormalizeAngle(end_pose[2] +
                                                         origin_heading +
                                                         0.5));

  EXPECT_FALSE(provider_->IsVehicleNearDestination(vehicle_state, end_pose,
                                                   origin_heading, origin));
  EXPECT_FALSE(frame_->open_space_info().destination_reached());
}

}  // namespace planning
}  // namespace apollo
