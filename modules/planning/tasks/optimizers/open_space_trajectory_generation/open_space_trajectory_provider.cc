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

#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.h"

#include <limits>
#include <memory>
#include <string>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"

#include "cyber/task/task.h"
#include "modules/common/math/polygon2d.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/common/trajectory_stitcher.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Vec2d;
using apollo::cyber::Clock;

namespace {

constexpr double kParkingSpotCompletionDistanceTolerance = 0.2;
constexpr double kParkingSpotCompletionHeadingTolerance = 0.2;
constexpr double kParkingSpotCompletionSpeedTolerance = 0.3;

struct ParkingSpotCompletionEvaluation {
  bool has_polygon = false;
  double speed = 0.0;
  double polygon_distance = std::numeric_limits<double>::infinity();
  double heading_error = std::numeric_limits<double>::infinity();
  bool footprint_inside = false;
  bool parked = false;
};

bool GetParkingPolygon(const Frame& frame,
                       std::vector<apollo::common::math::Vec2d>* points) {
  CHECK_NOTNULL(points);
  points->clear();

  const auto& routing = frame.local_view().routing;
  if (routing != nullptr && routing->routing_request().has_parking_info()) {
    const auto& parking_info = routing->routing_request().parking_info();
    if (parking_info.has_corner_point() &&
        parking_info.corner_point().point_size() >= 4) {
      points->reserve(parking_info.corner_point().point_size());
      for (const auto& point : parking_info.corner_point().point()) {
        points->emplace_back(point.x(), point.y());
      }
      return true;
    }
  }

  const auto& parking_spot_id =
      frame.open_space_info().target_parking_spot_id();
  if (parking_spot_id.empty()) {
    return false;
  }

  hdmap::Id id;
  id.set_id(parking_spot_id);
  const auto parking_spot =
      hdmap::HDMapUtil::BaseMapPtr()->GetParkingSpaceById(id);
  if (parking_spot == nullptr || parking_spot->polygon().points().size() < 4) {
    return false;
  }

  points->reserve(parking_spot->polygon().points().size());
  for (const auto& point : parking_spot->polygon().points()) {
    points->emplace_back(point.x(), point.y());
  }
  return true;
}

double ParkingHeading(const std::vector<apollo::common::math::Vec2d>& points) {
  CHECK_GE(points.size(), 4U);
  return (points[0] - points[3]).Angle();
}

bool IsVehicleParkedInTargetSpot(const Frame& frame,
                                 const common::VehicleState& vehicle_state,
                                 ParkingSpotCompletionEvaluation* evaluation) {
  CHECK_NOTNULL(evaluation);
  *evaluation = ParkingSpotCompletionEvaluation();
  evaluation->speed = std::abs(vehicle_state.linear_velocity());
  std::vector<apollo::common::math::Vec2d> parking_points;
  if (!GetParkingPolygon(frame, &parking_points)) {
    return false;
  }
  evaluation->has_polygon = true;

  const apollo::common::math::Polygon2d parking_polygon(
      std::move(parking_points));
  const apollo::common::math::Vec2d vehicle_center(vehicle_state.x(),
                                                   vehicle_state.y());
  const auto& vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  const double shift_distance =
      0.5 * vehicle_param.length() - vehicle_param.back_edge_to_center();
  const apollo::common::math::Vec2d vehicle_box_center(
      vehicle_center.x() + shift_distance * std::cos(vehicle_state.heading()),
      vehicle_center.y() + shift_distance * std::sin(vehicle_state.heading()));
  const apollo::common::math::Box2d vehicle_box(
      vehicle_box_center, vehicle_state.heading(), vehicle_param.length(),
      vehicle_param.width());
  const apollo::common::math::Polygon2d vehicle_polygon(vehicle_box);
  const double polygon_distance = parking_polygon.DistanceTo(vehicle_polygon);
  const double parking_heading = ParkingHeading(parking_polygon.points());
  const double heading_error =
      std::min(std::abs(common::math::AngleDiff(vehicle_state.heading(),
                                                parking_heading)),
               std::abs(common::math::AngleDiff(
                   vehicle_state.heading(),
                    common::math::NormalizeAngle(parking_heading + M_PI))));
  evaluation->polygon_distance = polygon_distance;
  evaluation->heading_error = heading_error;
  evaluation->footprint_inside = parking_polygon.Contains(vehicle_polygon);
  evaluation->parked = evaluation->footprint_inside &&
      evaluation->speed <= kParkingSpotCompletionSpeedTolerance &&
      polygon_distance <= kParkingSpotCompletionDistanceTolerance &&
      heading_error <= kParkingSpotCompletionHeadingTolerance;
  return evaluation->parked;
}

bool IsVehicleParkedInTargetSpot(const Frame& frame,
                                 const common::VehicleState& vehicle_state) {
  ParkingSpotCompletionEvaluation evaluation;
  return IsVehicleParkedInTargetSpot(frame, vehicle_state, &evaluation);
}

}  // namespace

OpenSpaceTrajectoryProvider::OpenSpaceTrajectoryProvider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : TrajectoryOptimizer(config, injector) {
  open_space_trajectory_optimizer_.reset(new OpenSpaceTrajectoryOptimizer(
      config.open_space_trajectory_provider_config()
          .open_space_trajectory_optimizer_config()));
  AINFO << config_.DebugString();
}

OpenSpaceTrajectoryProvider::~OpenSpaceTrajectoryProvider() {
  if (FLAGS_enable_open_space_planner_thread) {
    Stop();
  }
}

void OpenSpaceTrajectoryProvider::Stop() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
  is_planned_ = false;
  has_cached_trajectory_ = false;
  last_successful_trajectory_.clear();
}

void OpenSpaceTrajectoryProvider::Restart() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    is_generation_thread_stop_.store(false);
    thread_init_flag_ = false;
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
  is_planned_ = false;
  has_cached_trajectory_ = false;
  last_successful_trajectory_.clear();
}

Status OpenSpaceTrajectoryProvider::Process() {
  ADEBUG << "trajectory provider";
  auto trajectory_data =
      frame_->mutable_open_space_info()->mutable_stitched_trajectory_result();
  const bool use_planner_thread = FLAGS_enable_open_space_planner_thread;

  // generate stop trajectory at park_and_go check_stage
  if (injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_park_and_go()
          ->in_check_stage()) {
    ADEBUG << "ParkAndGo Stage Check.";
    GenerateStopTrajectory(trajectory_data);
    return Status::OK();
  }
  // Start thread when getting in Process() for the first time
  if (use_planner_thread && !thread_init_flag_) {
    task_future_ = cyber::Async(
        &OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this);
    thread_init_flag_ = true;
  }
  bool need_replan = false;
  // Get stitching trajectory from last frame
  const common::VehicleState vehicle_state = frame_->vehicle_state();
  auto* previous_frame = injector_->frame_history()->Latest();
  const bool has_reusable_open_space_plan =
      previous_frame != nullptr &&
      previous_frame->open_space_info().open_space_provider_success();
  const bool has_cached_open_space_plan =
      has_cached_trajectory_ && !last_successful_trajectory_.empty();
  const bool has_planned_trajectory =
      is_planned_ || (!use_planner_thread && (has_reusable_open_space_plan ||
                                              has_cached_open_space_plan));
  std::vector<TrajectoryPoint> stitching_trajectory;
  bool is_stop_due_to_fallback = false;
  if (previous_frame &&
      IsVehicleStopDueToFallBack(
          previous_frame->open_space_info().fallback_flag(), vehicle_state)) {
    is_stop_due_to_fallback = true;
  }
  if (!has_planned_trajectory || is_stop_due_to_fallback ||
      !injector_->planning_context()
           ->mutable_planning_status()
           ->mutable_open_space()
           ->position_init()) {
    AINFO << "need to fallback is_planned" << has_planned_trajectory
          << "is_stop_due_to_fallback" << is_stop_due_to_fallback;
    const double planning_cycle_time =
        1.0 / static_cast<double>(FLAGS_planning_loop_rate);
    stitching_trajectory = TrajectoryStitcher::ComputeReinitStitchingTrajectory(
        planning_cycle_time, vehicle_state);
    need_replan = true;
    auto* open_space_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();
    open_space_status->set_position_init(false);
    open_space_status->clear_partitioned_trajectories_index_history();
  }
  // Get open_space_info from current frame
  const auto& open_space_info = frame_->open_space_info();

  if (use_planner_thread) {
    ADEBUG << "Open space plan in multi-threads mode";

    if (is_generation_thread_stop_) {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Parking finished");
    }

    if (need_replan) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data_.stitching_trajectory = stitching_trajectory;
      thread_data_.end_pose = open_space_info.open_space_end_pose();
      thread_data_.rotate_angle = open_space_info.origin_heading();
      thread_data_.translate_origin = open_space_info.origin_point();
      thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
      thread_data_.obstacles_A = open_space_info.obstacles_A();
      thread_data_.obstacles_b = open_space_info.obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_info.obstacles_vertices_vec();
      thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
      thread_data_.has_required_final_gear =
          open_space_info.parking_enforce_final_gear();
      thread_data_.required_final_gear_forward =
          open_space_info.parking_head_in();
      data_ready_.store(true);
      is_planned_ = true;
    } else {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      data_ready_.store(false);
      AINFO << "SKIP BECAUSE HAS PLAN";
    }

    // Check vehicle state
    if (IsVehicleNearDestination(
            vehicle_state, open_space_info.open_space_end_pose(),
            open_space_info.origin_heading(), open_space_info.origin_point())) {
      GenerateStopTrajectory(trajectory_data);
      is_generation_thread_stop_.store(true);
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    // Check if trajectory updated
    if (trajectory_updated_) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      frame_->mutable_open_space_info()->set_time_latency(latest_time_latency_);
      LoadResult(trajectory_data);
      if (FLAGS_enable_record_debug) {
        // call merge debug ptr, open_space_trajectory_optimizer_
        auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
        open_space_trajectory_optimizer_->UpdateDebugInfo(
            ptr_debug->mutable_planning_data()->mutable_open_space());

        // sync debug instance
        frame_->mutable_open_space_info()->sync_debug_instance();
      }
      data_ready_.store(false);
      trajectory_updated_.store(false);
      return Status::OK();
    }

    if (trajectory_error_) {
      ++optimizer_thread_counter;
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      trajectory_error_.store(false);
      // TODO(Jinyun) Use other fallback mechanism when last iteration smoothing
      // result has out of bound pathpoint which is not allowed for next
      // iteration hybrid astar algorithm which requires start position to be
      // strictly in bound
      if (optimizer_thread_counter > 1000) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "open_space_optimizer failed too many times");
      }
    }

    if (previous_frame != nullptr &&
        previous_frame->open_space_info().open_space_provider_success()) {
      ReuseLastFrameResult(previous_frame, trajectory_data);
      if (FLAGS_enable_record_debug) {
        // copy previous debug to current frame
        ReuseLastFrameDebug(previous_frame);
      }
      // reuse last frame debug when use last frame traj
      return Status(ErrorCode::OK,
                    "Waiting for open_space_trajectory_optimizer in "
                    "open_space_trajectory_provider");
    } else {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Stop due to computation not finished");
    }
  } else {
    const auto& end_pose = open_space_info.open_space_end_pose();
    const auto& rotate_angle = open_space_info.origin_heading();
    const auto& translate_origin = open_space_info.origin_point();
    const auto& obstacles_edges_num = open_space_info.obstacles_edges_num();
    const auto& obstacles_A = open_space_info.obstacles_A();
    const auto& obstacles_b = open_space_info.obstacles_b();
    const auto& obstacles_vertices_vec =
        open_space_info.obstacles_vertices_vec();
    const auto& XYbounds = open_space_info.ROI_xy_boundary();
    const bool has_required_final_gear =
        open_space_info.parking_enforce_final_gear();
    const bool required_final_gear_forward = open_space_info.parking_head_in();

    // Check vehicle state
    if (IsVehicleNearDestination(vehicle_state, end_pose, rotate_angle,
                                 translate_origin)) {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    if (!need_replan) {
      if (has_cached_open_space_plan) {
        ReuseCachedResult(trajectory_data);
        if (FLAGS_enable_record_debug && has_reusable_open_space_plan) {
          ReuseLastFrameDebug(previous_frame);
        }
        return Status::OK();
      }
      if (has_reusable_open_space_plan) {
        ReuseLastFrameResult(previous_frame, trajectory_data);
        if (FLAGS_enable_record_debug) {
          ReuseLastFrameDebug(previous_frame);
        }
        return Status::OK();
      }
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK,
                    "Stop due to missing reusable open-space trajectory");
    }

    // Generate Trajectory;
    double time_latency;
    AINFO << "Parking final gear policy enforce=" << has_required_final_gear
          << " required="
          << (required_final_gear_forward ? "forward" : "reverse");
    Status status = open_space_trajectory_optimizer_->Plan(
        stitching_trajectory, end_pose, XYbounds, rotate_angle,
        translate_origin, obstacles_edges_num, obstacles_A, obstacles_b,
        obstacles_vertices_vec, &time_latency, has_required_final_gear,
        required_final_gear_forward);
    frame_->mutable_open_space_info()->set_time_latency(time_latency);

    // If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      LoadResult(trajectory_data);
      is_planned_ = true;
      return status;
    } else {
      is_planned_ = false;
      has_cached_trajectory_ = false;
      last_successful_trajectory_.clear();
      return status;
    }
  }
  return Status(ErrorCode::PLANNING_ERROR);
}

void OpenSpaceTrajectoryProvider::GenerateTrajectoryThread() {
  while (!is_generation_thread_stop_) {
    if (!trajectory_updated_ && data_ready_) {
      OpenSpaceTrajectoryThreadData thread_data;
      {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        thread_data = thread_data_;
      }
      double time_latency;
      Status status = open_space_trajectory_optimizer_->Plan(
          thread_data.stitching_trajectory, thread_data.end_pose,
          thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.obstacles_edges_num,
          thread_data.obstacles_A, thread_data.obstacles_b,
          thread_data.obstacles_vertices_vec, &time_latency,
          thread_data.has_required_final_gear,
          thread_data.required_final_gear_forward);
      if (status == Status::OK()) {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        latest_time_latency_ = time_latency;
        trajectory_updated_.store(true);
      } else {
        if (status.ok()) {
          std::lock_guard<std::mutex> lock(open_space_mutex_);
          trajectory_skipped_.store(true);
        } else {
          std::lock_guard<std::mutex> lock(open_space_mutex_);
          trajectory_error_.store(true);
        }
      }
    }
  }
}

bool OpenSpaceTrajectoryProvider::IsVehicleNearDestination(
    const common::VehicleState& vehicle_state,
    const std::vector<double>& end_pose, double rotate_angle,
    const Vec2d& translate_origin) {
  CHECK_EQ(end_pose.size(), 4U);
  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  double end_theta_to_world_frame = end_pose[2];
  end_theta_to_world_frame += rotate_angle;

  double distance_to_vehicle =
      std::sqrt((vehicle_state.x() - end_pose_to_world_frame.x()) *
                    (vehicle_state.x() - end_pose_to_world_frame.x()) +
                (vehicle_state.y() - end_pose_to_world_frame.y()) *
                    (vehicle_state.y() - end_pose_to_world_frame.y()));

  double theta_to_vehicle = std::abs(common::math::AngleDiff(
      vehicle_state.heading(), end_theta_to_world_frame));
  ADEBUG << "theta_to_vehicle" << theta_to_vehicle << "end_theta_to_world_frame"
         << end_theta_to_world_frame << "rotate_angle" << rotate_angle;
  ADEBUG << "is_near_destination_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_threshold();  // which config file
  ADEBUG << "is_near_destination_theta_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_theta_threshold();
  const bool is_near_end_pose =
      distance_to_vehicle < config_.open_space_trajectory_provider_config()
                                .open_space_trajectory_optimizer_config()
                                .planner_open_space_config()
                                .is_near_destination_threshold() &&
      theta_to_vehicle < config_.open_space_trajectory_provider_config()
                             .open_space_trajectory_optimizer_config()
                             .planner_open_space_config()
                             .is_near_destination_theta_threshold();
  if (is_near_end_pose) {
    std::vector<Vec2d> parking_points;
    ParkingSpotCompletionEvaluation evaluation;
    const bool parked_in_target =
        IsVehicleParkedInTargetSpot(*frame_, vehicle_state, &evaluation);
    if (!GetParkingPolygon(*frame_, &parking_points) || parked_in_target) {
      ADEBUG << "vehicle reach end_pose";
      frame_->mutable_open_space_info()->set_destination_reached(true);
      return true;
    }
    AINFO_EVERY(20) << "Vehicle is near parking end pose but not inside target "
                       "spot: speed="
                     << evaluation.speed
                     << " footprint_inside=" << evaluation.footprint_inside
                     << " polygon_distance=" << evaluation.polygon_distance
                     << " heading_error=" << evaluation.heading_error;
  }

  if (IsVehicleParkedInTargetSpot(*frame_, vehicle_state)) {
    frame_->mutable_open_space_info()->set_destination_reached(true);
    return true;
  }

  return false;
}

bool OpenSpaceTrajectoryProvider::IsVehicleStopDueToFallBack(
    const bool is_on_fallback, const common::VehicleState& vehicle_state) {
  if (!is_on_fallback) {
    return false;
  }
  static constexpr double kEpsilon = 1.0e-1;
  const double adc_speed = vehicle_state.linear_velocity();
  const double adc_acceleration = vehicle_state.linear_acceleration();
  if (std::abs(adc_speed) < kEpsilon && std::abs(adc_acceleration) < kEpsilon) {
    ADEBUG << "ADC stops due to fallback trajectory";
    return true;
  }
  return false;
}

void OpenSpaceTrajectoryProvider::GenerateStopTrajectory(
    DiscretizedTrajectory* const trajectory_data) {
  double relative_time = 0.0;
  // TODO(Jinyun) Move to conf
  static constexpr int stop_trajectory_length = 10;
  static constexpr double relative_stop_time = 0.1;
  static constexpr double vEpsilon = 0.00001;
  double standstill_acceleration =
      frame_->vehicle_state().linear_velocity() >= -vEpsilon
          ? -FLAGS_open_space_standstill_acceleration
          : FLAGS_open_space_standstill_acceleration;
  trajectory_data->clear();
  for (size_t i = 0; i < stop_trajectory_length; i++) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(0.0);
    point.set_a(standstill_acceleration);
    trajectory_data->emplace_back(point);
    relative_time += relative_stop_time;
  }
}

void OpenSpaceTrajectoryProvider::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  // Load unstitched two trajectories into frame for debug
  trajectory_data->clear();
  auto optimizer_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_optimizer_trajectory_data();
  auto stitching_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_stitching_trajectory_data();
  open_space_trajectory_optimizer_->GetOptimizedTrajectory(
      optimizer_trajectory_ptr);
  open_space_trajectory_optimizer_->GetStitchingTrajectory(
      stitching_trajectory_ptr);
  // Stitch two trajectories and load back to trajectory_data from frame
  size_t optimizer_trajectory_size = optimizer_trajectory_ptr->size();
  double stitching_point_relative_time =
      stitching_trajectory_ptr->back().relative_time();
  double stitching_point_relative_s =
      stitching_trajectory_ptr->back().path_point().s();
  for (size_t i = 0; i < optimizer_trajectory_size; ++i) {
    optimizer_trajectory_ptr->at(i).set_relative_time(
        optimizer_trajectory_ptr->at(i).relative_time() +
        stitching_point_relative_time);
    optimizer_trajectory_ptr->at(i).mutable_path_point()->set_s(
        optimizer_trajectory_ptr->at(i).path_point().s() +
        stitching_point_relative_s);
  }
  *(trajectory_data) = *(optimizer_trajectory_ptr);

  // Last point in stitching trajectory is already in optimized trajectory, so
  // it is deleted
  frame_->mutable_open_space_info()
      ->mutable_stitching_trajectory_data()
      ->pop_back();
  trajectory_data->PrependTrajectoryPoints(
      frame_->open_space_info().stitching_trajectory_data());
  last_successful_trajectory_ = *trajectory_data;
  has_cached_trajectory_ = !last_successful_trajectory_.empty();
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void OpenSpaceTrajectoryProvider::ReuseLastFrameResult(
    const Frame* last_frame, DiscretizedTrajectory* const trajectory_data) {
  *(trajectory_data) =
      last_frame->open_space_info().stitched_trajectory_result();
  last_successful_trajectory_ = *trajectory_data;
  has_cached_trajectory_ = !last_successful_trajectory_.empty();
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void OpenSpaceTrajectoryProvider::ReuseCachedResult(
    DiscretizedTrajectory* const trajectory_data) {
  *trajectory_data = last_successful_trajectory_;
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void OpenSpaceTrajectoryProvider::ReuseLastFrameDebug(const Frame* last_frame) {
  // reuse last frame's instance
  auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug_instance();
  ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
      last_frame->open_space_info()
          .debug_instance()
          .planning_data()
          .open_space());
}

}  // namespace planning
}  // namespace apollo
