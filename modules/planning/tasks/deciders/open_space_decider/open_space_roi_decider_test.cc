#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "cyber/init.h"

#define private public
#include "modules/planning/tasks/deciders/open_space_decider/open_space_roi_decider.h"
#undef private

namespace apollo {
namespace planning {
namespace {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

TaskConfig MakeParkingTaskConfig() {
  TaskConfig config;
  config.set_task_type(TaskConfig::OPEN_SPACE_ROI_DECIDER);
  auto* roi_config = config.mutable_open_space_roi_decider_config();
  roi_config->set_roi_type(OpenSpaceRoiDeciderConfig::PARKING);
  roi_config->set_roi_min_area(10.0);
  roi_config->set_roi_min_goal_clearance(0.2);
  return config;
}

Frame MakeFrame() {
  return Frame(0, LocalView{}, TrajectoryPoint{}, VehicleState{}, nullptr);
}

void PopulateParkingRoi(Frame* frame, const std::vector<Vec2d>& polygon,
                        const std::vector<double>& xy_boundary,
                        const std::vector<double>& end_pose) {
  auto* open_space_info = frame->mutable_open_space_info();
  *open_space_info->mutable_roi_parking_boundary_polygon() = polygon;
  *open_space_info->mutable_ROI_xy_boundary() = xy_boundary;
  *open_space_info->mutable_open_space_end_pose() = end_pose;
  open_space_info->set_roi_parking_area(60.0);
  open_space_info->set_roi_parking_aisle_width(5.0);
}

}  // namespace

class OpenSpaceRoiDeciderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    apollo::cyber::Init("open_space_roi_decider_test");
    injector_ = std::make_shared<DependencyInjector>();
    decider_ = std::make_unique<OpenSpaceRoiDecider>(MakeParkingTaskConfig(),
                                                     injector_);
  }

  std::shared_ptr<DependencyInjector> injector_;
  std::unique_ptr<OpenSpaceRoiDecider> decider_;
};

TEST_F(OpenSpaceRoiDeciderTest, ValidateParkingRoiAcceptsWorkflowOutput) {
  Frame frame = MakeFrame();
  PopulateParkingRoi(&frame,
                     {Vec2d(-2.0, -3.0), Vec2d(10.0, -3.0), Vec2d(10.0, 3.0),
                      Vec2d(-2.0, 3.0)},
                     {-2.0, 10.0, -3.0, 3.0},
                     {5.0, 0.0, 0.0, 0.0});

  EXPECT_TRUE(decider_->ValidateROIOnVertices(&frame));

  const auto debug_instance = frame.open_space_info().debug_instance();
  const auto& parking_roi =
      debug_instance.planning_data().open_space().parking_roi();
  EXPECT_TRUE(parking_roi.is_valid());
  EXPECT_EQ(parking_roi.invalid_reason(), "");
}

TEST_F(OpenSpaceRoiDeciderTest,
       ValidateParkingRoiReportsGoalNearBoundaryWithoutRejecting) {
  Frame frame = MakeFrame();
  PopulateParkingRoi(&frame,
                     {Vec2d(-2.0, -3.0), Vec2d(10.0, -3.0), Vec2d(10.0, 3.0),
                      Vec2d(-2.0, 3.0)},
                     {-2.0, 10.0, -3.0, 3.0},
                       {5.95, 0.0, 0.0, 0.0});

  EXPECT_TRUE(decider_->ValidateROIOnVertices(&frame));

  const auto debug_instance = frame.open_space_info().debug_instance();
  const auto& parking_roi =
      debug_instance.planning_data().open_space().parking_roi();
  EXPECT_TRUE(parking_roi.is_valid());
  EXPECT_EQ(parking_roi.invalid_reason(), "");
  EXPECT_GT(parking_roi.goal_clearance(), 0.0);
  EXPECT_LT(parking_roi.goal_clearance(), 0.2);
}

TEST_F(OpenSpaceRoiDeciderTest,
       ValidateParkingRoiPreservesWorkflowDebugSelection) {
  Frame frame = MakeFrame();
  PopulateParkingRoi(&frame,
                     {Vec2d(-2.0, -3.0), Vec2d(10.0, -3.0), Vec2d(10.0, 3.0),
                      Vec2d(-2.0, 3.0)},
                     {-2.0, 10.0, -3.0, 3.0},
                     {5.0, 0.0, 0.0, 0.0});
  auto* open_space_debug = frame.mutable_open_space_info()
                               ->mutable_debug_instance()
                               ->mutable_planning_data()
                               ->mutable_open_space();
  open_space_debug->set_selected_parking_pose("head_in");

  EXPECT_TRUE(decider_->ValidateROIOnVertices(&frame));
  const auto debug_instance = frame.open_space_info().debug_instance();
  EXPECT_EQ(debug_instance.planning_data().open_space().selected_parking_pose(),
            "head_in");
}

TEST_F(OpenSpaceRoiDeciderTest, PullOverEndPoseClearsPreviousValues) {
  Frame frame = MakeFrame();
  *frame.mutable_open_space_info()->mutable_open_space_end_pose() =
      {9.0, 8.0, 7.0, 6.0};
  frame.mutable_open_space_info()->set_origin_heading(0.0);
  frame.mutable_open_space_info()->mutable_origin_point()->set_x(10.0);
  frame.mutable_open_space_info()->mutable_origin_point()->set_y(20.0);

  auto* pull_over =
      injector_->planning_context()->mutable_planning_status()->mutable_pull_over();
  pull_over->mutable_position()->set_x(13.0);
  pull_over->mutable_position()->set_y(24.0);
  pull_over->set_theta(0.25);

  decider_->SetPullOverSpotEndPose(&frame);

  const auto& end_pose = frame.open_space_info().open_space_end_pose();
  ASSERT_EQ(end_pose.size(), 4U);
  EXPECT_DOUBLE_EQ(end_pose[0], 3.0);
  EXPECT_DOUBLE_EQ(end_pose[1], 4.0);
  EXPECT_DOUBLE_EQ(end_pose[2], 0.25);
  EXPECT_DOUBLE_EQ(end_pose[3], 0.0);
}

}  // namespace planning
}  // namespace apollo
