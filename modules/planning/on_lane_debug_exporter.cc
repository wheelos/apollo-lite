/******************************************************************************
 * Copyright 2026 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/on_lane_debug_exporter.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "absl/strings/str_cat.h"

#include "modules/common_msgs/dreamview_msgs/chart.pb.h"

#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {

using apollo::cyber::Clock;
using apollo::dreamview::Chart;
using apollo::planning_internal::SLFrameDebug;
using apollo::planning_internal::SpeedPlan;
using apollo::planning_internal::STGraphDebug;

namespace {

void SetChartMinMax(apollo::dreamview::Chart* chart,
                    const std::string& label_name_x,
                    const std::string& label_name_y) {
  auto* options = chart->mutable_options();
  double xmin(std::numeric_limits<double>::max());
  double xmax(std::numeric_limits<double>::lowest());
  double ymin(std::numeric_limits<double>::max());
  double ymax(std::numeric_limits<double>::lowest());
  for (int index = 0; index < chart->line_size(); ++index) {
    auto* line = chart->mutable_line(index);
    for (auto& point : line->point()) {
      xmin = std::min(xmin, point.x());
      ymin = std::min(ymin, point.y());
      xmax = std::max(xmax, point.x());
      ymax = std::max(ymax, point.y());
    }
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
  }
  options->mutable_x()->set_min(xmin);
  options->mutable_x()->set_max(xmax);
  options->mutable_x()->set_label_string(label_name_x);
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string(label_name_y);
}

void PopulateChartOptions(double x_min, double x_max,
                          const std::string& x_label, double y_min,
                          double y_max, const std::string& y_label,
                          bool display, Chart* chart) {
  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

void AddSTGraph(const STGraphDebug& st_graph, Chart* chart) {
  if (st_graph.name() == "DP_ST_SPEED_OPTIMIZER") {
    chart->set_title("Speed Heuristic");
  } else {
    chart->set_title("Planning S-T Graph");
  }
  PopulateChartOptions(-2.0, 10.0, "t (second)", -10.0, 220.0, "s (meter)",
                       false, chart);

  for (const auto& boundary : st_graph.boundary()) {
    std::string type =
        StGraphBoundaryDebug_StBoundaryType_Name(boundary.type()).substr(17);

    auto* boundary_chart = chart->add_polygon();
    auto* properties = boundary_chart->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["cubicInterpolationMode"] = "monotone";
    (*properties)["showLine"] = "true";
    (*properties)["showText"] = "true";
    (*properties)["fill"] = "false";

    if (type == "DRIVABLE_REGION") {
      (*properties)["color"] = "\"rgba(0, 255, 0, 0.5)\"";
    } else {
      (*properties)["color"] = "\"rgba(255, 0, 0, 0.8)\"";
    }

    boundary_chart->set_label(boundary.name() + "_" + type);
    for (const auto& point : boundary.point()) {
      auto* point_debug = boundary_chart->add_point();
      point_debug->set_x(point.t());
      point_debug->set_y(point.s());
    }
  }

  auto* speed_profile = chart->add_line();
  auto* properties = speed_profile->mutable_properties();
  (*properties)["color"] = "\"rgba(255, 255, 255, 0.5)\"";
  for (const auto& point : st_graph.speed_profile()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.t());
    point_debug->set_y(point.s());
  }
}

void AddSLFrame(const SLFrameDebug& sl_frame, Chart* chart) {
  chart->set_title(sl_frame.name());
  PopulateChartOptions(0.0, 80.0, "s (meter)", -8.0, 8.0, "l (meter)", false,
                       chart);
  auto* sl_line = chart->add_line();
  sl_line->set_label("SL Path");
  for (const auto& sl_point : sl_frame.sl_path()) {
    auto* point_debug = sl_line->add_point();
    point_debug->set_x(sl_point.s());
    point_debug->set_y(sl_point.l());
  }
}

void AddSpeedPlan(
    const ::google::protobuf::RepeatedPtrField<SpeedPlan>& speed_plans,
    Chart* chart) {
  chart->set_title("Speed Plan");
  PopulateChartOptions(0.0, 80.0, "s (meter)", 0.0, 50.0, "v (m/s)", false,
                       chart);

  for (const auto& speed_plan : speed_plans) {
    auto* line = chart->add_line();
    line->set_label(speed_plan.name());
    for (const auto& point : speed_plan.speed_point()) {
      auto* point_debug = line->add_point();
      point_debug->set_x(point.s());
      point_debug->set_y(point.v());
    }

    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
    if (speed_plan.name() == "DpStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(27, 249, 105, 0.5)\"";
    } else if (speed_plan.name() == "QpSplineStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(54, 162, 235, 1)\"";
    }
  }
}

}  // namespace

OnLaneDebugExporter::OnLaneDebugExporter(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

void OnLaneDebugExporter::ExportReferenceLineDebug(
    Frame* frame, planning_internal::Debug* debug) const {
  if (!FLAGS_enable_record_debug || frame == nullptr || debug == nullptr) {
    return;
  }
  for (const auto& reference_line_info : frame->reference_line_info()) {
    auto* rl_debug = debug->mutable_planning_data()->add_reference_line();
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);

    const auto& reference_points =
        reference_line_info.reference_line().reference_points();
    double kappa_rms = 0.0;
    double dkappa_rms = 0.0;
    double kappa_max_abs = std::numeric_limits<double>::lowest();
    double dkappa_max_abs = std::numeric_limits<double>::lowest();
    for (const auto& reference_point : reference_points) {
      const double kappa_sq = reference_point.kappa() * reference_point.kappa();
      const double dkappa_sq =
          reference_point.dkappa() * reference_point.dkappa();
      kappa_rms += kappa_sq;
      dkappa_rms += dkappa_sq;
      kappa_max_abs = std::max(kappa_max_abs, kappa_sq);
      dkappa_max_abs = std::max(dkappa_max_abs, dkappa_sq);
    }
    const double reference_points_size =
        static_cast<double>(reference_points.size());
    kappa_rms = std::sqrt(kappa_rms / reference_points_size);
    dkappa_rms = std::sqrt(dkappa_rms / reference_points_size);
    rl_debug->set_kappa_rms(kappa_rms);
    rl_debug->set_dkappa_rms(dkappa_rms);
    rl_debug->set_kappa_max_abs(kappa_max_abs);
    rl_debug->set_dkappa_max_abs(dkappa_max_abs);

    bool is_off_road = false;
    double minimum_boundary = std::numeric_limits<double>::infinity();
    const double adc_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    const auto& reference_line_path =
        reference_line_info.reference_line().GetMapPath();
    const auto sample_s = 0.1;
    const auto reference_line_length =
        reference_line_info.reference_line().Length();
    double average_offset = 0.0;
    double sample_count = 0.0;
    for (double s = 0.0; s < reference_line_length; s += sample_s) {
      const double left_width = reference_line_path.GetLaneLeftWidth(s);
      const double right_width = reference_line_path.GetLaneRightWidth(s);
      average_offset += 0.5 * std::abs(left_width - right_width);
      if (left_width < adc_half_width || right_width < adc_half_width) {
        is_off_road = true;
      }
      minimum_boundary = std::min(minimum_boundary, left_width);
      minimum_boundary = std::min(minimum_boundary, right_width);
      ++sample_count;
    }
    rl_debug->set_is_offroad(is_off_road);
    rl_debug->set_minimum_boundary(minimum_boundary);
    rl_debug->set_average_offset(average_offset / sample_count);
  }
}

void OnLaneDebugExporter::ExportFailedLaneChangeSTChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) const {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
}

void OnLaneDebugExporter::ExportOnLaneChart(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) const {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, dst_data->add_chart());
  }
  for (const auto& sl_frame : src_data.sl_frame()) {
    AddSLFrame(sl_frame, dst_data->add_chart());
  }
  AddSpeedPlan(src_data.speed_plan(), dst_data->add_chart());
}

void OnLaneDebugExporter::ExportOpenSpaceChart(
    Frame* frame, const planning_internal::Debug& debug_info,
    const ADCTrajectory& trajectory_pb,
    planning_internal::Debug* debug_chart) const {
  if (!FLAGS_enable_record_debug || frame == nullptr ||
      debug_chart == nullptr) {
    return;
  }
  AddOpenSpaceOptimizerResult(*frame, debug_info, debug_chart);
  AddPartitionedTrajectory(*frame, debug_info, debug_chart);
  AddStitchSpeedProfile(*frame, debug_chart);
  AddPublishedSpeed(*frame, trajectory_pb, debug_chart);
  AddPublishedAcceleration(*frame, trajectory_pb, debug_chart);
}

void OnLaneDebugExporter::ExportPlanningReferenceLinePath(
    const ReferenceLineInfo& best_ref_info,
    planning_internal::Debug* ptr_debug) const {
  if (!FLAGS_enable_record_debug || ptr_debug == nullptr) {
    return;
  }
  auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
  reference_line->set_name("planning_reference_line");
  const auto& reference_points =
      best_ref_info.reference_line().reference_points();
  double s = 0.0;
  double prev_x = 0.0;
  double prev_y = 0.0;
  bool empty_path = true;
  for (const auto& reference_point : reference_points) {
    auto* path_point = reference_line->add_path_point();
    path_point->set_x(reference_point.x());
    path_point->set_y(reference_point.y());
    path_point->set_theta(reference_point.heading());
    path_point->set_kappa(reference_point.kappa());
    path_point->set_dkappa(reference_point.dkappa());
    if (empty_path) {
      path_point->set_s(0.0);
      empty_path = false;
    } else {
      const double dx = reference_point.x() - prev_x;
      const double dy = reference_point.y() - prev_y;
      s += std::hypot(dx, dy);
      path_point->set_s(s);
    }
    prev_x = reference_point.x();
    prev_y = reference_point.y();
  }
}

void OnLaneDebugExporter::AddOpenSpaceOptimizerResult(
    const Frame& frame, const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) const {
  if (!frame.open_space_info().open_space_provider_success()) {
    return;
  }

  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  const auto& open_space_debug = debug_info.planning_data().open_space();
  chart->set_title("Open Space Trajectory Optimizer Visualization");
  PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                       open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                       open_space_debug.xy_boundary(2) - 1.0,
                       open_space_debug.xy_boundary(3) + 1.0, "y (meter)", true,
                       chart);

  chart->mutable_options()->set_sync_xy_window_size(true);
  chart->mutable_options()->set_aspect_ratio(0.9);
  int obstacle_index = 1;
  for (const auto& obstacle : open_space_debug.obstacles()) {
    auto* obstacle_outline = chart->add_line();
    obstacle_outline->set_label(absl::StrCat("Bdr", obstacle_index));
    ++obstacle_index;
    for (int vertice_index = 0;
         vertice_index < obstacle.vertices_x_coords_size(); ++vertice_index) {
      auto* point_debug = obstacle_outline->add_point();
      point_debug->set_x(obstacle.vertices_x_coords(vertice_index));
      point_debug->set_y(obstacle.vertices_y_coords(vertice_index));
    }
    auto* obstacle_properties = obstacle_outline->mutable_properties();
    (*obstacle_properties)["borderWidth"] = "2";
    (*obstacle_properties)["pointRadius"] = "0";
    (*obstacle_properties)["lineTension"] = "0";
    (*obstacle_properties)["fill"] = "false";
    (*obstacle_properties)["showLine"] = "true";
  }

  const auto smoothed_trajectory = open_space_debug.smoothed_trajectory();
  auto* smoothed_line = chart->add_line();
  smoothed_line->set_label("Smooth");
  for (int index = 0;
       index < smoothed_trajectory.vehicle_motion_point_size() / 2; ++index) {
    const auto& point = smoothed_trajectory.vehicle_motion_point(index);
    auto* point_debug = smoothed_line->add_point();
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }
  auto* smoothed_properties = smoothed_line->mutable_properties();
  (*smoothed_properties)["borderWidth"] = "2";
  (*smoothed_properties)["pointRadius"] = "0";
  (*smoothed_properties)["lineTension"] = "0";
  (*smoothed_properties)["fill"] = "false";
  (*smoothed_properties)["showLine"] = "true";

  const auto warm_start_trajectory = open_space_debug.warm_start_trajectory();
  auto* warm_start_line = chart->add_line();
  warm_start_line->set_label("WarmStart");
  for (int index = 0;
       index < warm_start_trajectory.vehicle_motion_point_size() / 2; ++index) {
    auto* point_debug = warm_start_line->add_point();
    const auto& point = warm_start_trajectory.vehicle_motion_point(index);
    point_debug->set_x(point.trajectory_point().path_point().x());
    point_debug->set_y(point.trajectory_point().path_point().y());
  }
  auto* warm_start_properties = warm_start_line->mutable_properties();
  (*warm_start_properties)["borderWidth"] = "2";
  (*warm_start_properties)["pointRadius"] = "0";
  (*warm_start_properties)["lineTension"] = "0";
  (*warm_start_properties)["fill"] = "false";
  (*warm_start_properties)["showLine"] = "true";
}

void OnLaneDebugExporter::AddPartitionedTrajectory(
    const Frame& frame, const planning_internal::Debug& debug_info,
    planning_internal::Debug* debug_chart) const {
  if (!frame.open_space_info().open_space_provider_success()) {
    return;
  }

  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_trajectories =
      open_space_debug.chosen_trajectory().trajectory();
  if (chosen_trajectories.empty() ||
      chosen_trajectories[0].trajectory_point().empty()) {
    return;
  }

  const auto& vehicle_state = frame.vehicle_state();
  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  auto* chart_kappa = debug_chart->mutable_planning_data()->add_chart();
  auto* chart_theta = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Partitioned Trajectory");
  chart_kappa->set_title("total kappa");
  chart_theta->set_title("total theta");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(0.9);

  auto* adc_shape = chart->add_car();
  adc_shape->set_x(vehicle_state.x());
  adc_shape->set_y(vehicle_state.y());
  adc_shape->set_heading(vehicle_state.heading());
  adc_shape->set_label("ADV");
  adc_shape->set_color("rgba(54, 162, 235, 1)");

  const auto& chosen_trajectory = chosen_trajectories[0];
  auto* chosen_line = chart->add_line();
  chosen_line->set_label("Chosen");
  for (const auto& point : chosen_trajectory.trajectory_point()) {
    auto* point_debug = chosen_line->add_point();
    point_debug->set_x(point.path_point().x());
    point_debug->set_y(point.path_point().y());
  }
  auto* chosen_properties = chosen_line->mutable_properties();
  (*chosen_properties)["borderWidth"] = "2";
  (*chosen_properties)["pointRadius"] = "0";
  (*chosen_properties)["lineTension"] = "0";
  (*chosen_properties)["fill"] = "false";
  (*chosen_properties)["showLine"] = "true";
  auto* theta_line = chart_theta->add_line();
  auto* kappa_line = chart_kappa->add_line();
  size_t partitioned_trajectory_label = 0;
  for (const auto& partitioned_trajectory :
       open_space_debug.partitioned_trajectories().trajectory()) {
    auto* partition_line = chart->add_line();
    partition_line->set_label(
        absl::StrCat("Partitioned ", partitioned_trajectory_label));
    ++partitioned_trajectory_label;
    for (const auto& point : partitioned_trajectory.trajectory_point()) {
      auto* point_debug = partition_line->add_point();
      auto* point_theta = theta_line->add_point();
      auto* point_kappa = kappa_line->add_point();
      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
      point_theta->set_x(point.relative_time());
      point_kappa->set_x(point.relative_time());
      point_theta->set_y(point.path_point().theta());
      point_kappa->set_y(point.path_point().kappa());
    }

    auto* partition_properties = partition_line->mutable_properties();
    (*partition_properties)["borderWidth"] = "2";
    (*partition_properties)["pointRadius"] = "0";
    (*partition_properties)["lineTension"] = "0";
    (*partition_properties)["fill"] = "false";
    (*partition_properties)["showLine"] = "true";
    SetChartMinMax(chart_kappa, "time", "total kappa");
    SetChartMinMax(chart_theta, "time", "total theta");
  }

  auto* stitching_line = chart->add_line();
  stitching_line->set_label("TrajectoryStitchingPoint");
  auto* trajectory_stitching_point = stitching_line->add_point();
  trajectory_stitching_point->set_x(
      open_space_debug.trajectory_stitching_point().path_point().x());
  trajectory_stitching_point->set_y(
      open_space_debug.trajectory_stitching_point().path_point().y());
  auto* stitching_properties = stitching_line->mutable_properties();
  (*stitching_properties)["borderWidth"] = "3";
  (*stitching_properties)["pointRadius"] = "5";
  (*stitching_properties)["lineTension"] = "0";
  (*stitching_properties)["fill"] = "true";
  (*stitching_properties)["showLine"] = "true";

  if (open_space_debug.is_fallback_trajectory()) {
    auto* collision_line = chart->add_line();
    collision_line->set_label("FutureCollisionPoint");
    auto* future_collision_point = collision_line->add_point();
    future_collision_point->set_x(
        open_space_debug.future_collision_point().path_point().x());
    future_collision_point->set_y(
        open_space_debug.future_collision_point().path_point().y());
    auto* collision_properties = collision_line->mutable_properties();
    (*collision_properties)["borderWidth"] = "3";
    (*collision_properties)["pointRadius"] = "8";
    (*collision_properties)["lineTension"] = "0";
    (*collision_properties)["fill"] = "true";
    (*collision_properties)["showLine"] = "true";
    (*collision_properties)["pointStyle"] = "cross";

    const auto& fallback_trajectories =
        open_space_debug.fallback_trajectory().trajectory();
    if (fallback_trajectories.empty() ||
        fallback_trajectories[0].trajectory_point().empty()) {
      return;
    }
    const auto& fallback_trajectory = fallback_trajectories[0];
    auto* fallback_line = chart->add_line();
    fallback_line->set_label("Fallback");
    for (const auto& point : fallback_trajectory.trajectory_point()) {
      auto* point_debug = fallback_line->add_point();
      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
    }
    auto* fallback_properties = fallback_line->mutable_properties();
    (*fallback_properties)["borderWidth"] = "3";
    (*fallback_properties)["pointRadius"] = "2";
    (*fallback_properties)["lineTension"] = "0";
    (*fallback_properties)["fill"] = "false";
    (*fallback_properties)["showLine"] = "true";
  }
}

void OnLaneDebugExporter::AddStitchSpeedProfile(
    const Frame& frame, planning_internal::Debug* debug_chart) const {
  if (!frame.open_space_info().open_space_provider_success() ||
      injector_ == nullptr || injector_->frame_history()->Latest() == nullptr) {
    return;
  }

  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Speed Plan Visualization");
  auto* options = chart->mutable_options();
  double xmin(std::numeric_limits<double>::max());
  double xmax(std::numeric_limits<double>::lowest());
  double ymin(std::numeric_limits<double>::max());
  double ymax(std::numeric_limits<double>::lowest());
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  const auto& last_trajectory =
      injector_->frame_history()->Latest()->current_frame_planned_trajectory();
  for (const auto& point : last_trajectory.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       last_trajectory.header().timestamp_sec());
    point_debug->set_y(point.v());
    xmin = std::min(xmin, point_debug->x());
    xmax = std::max(xmax, point_debug->x());
    ymin = std::min(ymin, point_debug->y());
    ymax = std::max(ymax, point_debug->y());
  }
  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("speed (m/s)");
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";
}

void OnLaneDebugExporter::AddPublishedSpeed(
    const Frame& frame, const ADCTrajectory& trajectory_pb,
    planning_internal::Debug* debug_chart) const {
  if (!frame.open_space_info().open_space_provider_success()) {
    return;
  }

  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();
  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  double xmin(std::numeric_limits<double>::max());
  double xmax(std::numeric_limits<double>::lowest());
  double ymin(std::numeric_limits<double>::max());
  double ymax(std::numeric_limits<double>::lowest());
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());
    point_debug->set_y(trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE
                           ? -point.v()
                           : point.v());
    xmin = std::min(xmin, point_debug->x());
    xmax = std::max(xmax, point_debug->x());
    ymin = std::min(ymin, point_debug->y());
    ymax = std::max(ymax, point_debug->y());
  }
  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("speed (m/s)");
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");
  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

void OnLaneDebugExporter::AddPublishedAcceleration(
    const Frame& frame, const ADCTrajectory& trajectory_pb,
    planning_internal::Debug* debug_chart) const {
  if (!frame.open_space_info().open_space_provider_success()) {
    return;
  }
  double xmin(std::numeric_limits<double>::max());
  double xmax(std::numeric_limits<double>::lowest());
  double ymin(std::numeric_limits<double>::max());
  double ymax(std::numeric_limits<double>::lowest());
  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time() +
                       trajectory_pb.header().timestamp_sec());
    point_debug->set_y(trajectory_pb.gear() == canbus::Chassis::GEAR_REVERSE
                           ? -point.a()
                           : point.a());
    xmin = std::min(xmin, point_debug->x());
    xmax = std::max(xmax, point_debug->x());
    ymin = std::min(ymin, point_debug->y());
    ymax = std::max(ymax, point_debug->y());
  }
  options->mutable_x()->set_window_size(xmax - xmin);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_min(ymin);
  options->mutable_y()->set_max(ymax);
  options->mutable_y()->set_label_string("acceleration (m/s)");
  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";

  auto* sliding_line = chart->add_line();
  sliding_line->set_label("Time");
  auto* point_debug_up = sliding_line->add_point();
  point_debug_up->set_x(Clock::NowInSeconds());
  point_debug_up->set_y(2.1);
  auto* point_debug_down = sliding_line->add_point();
  point_debug_down->set_x(Clock::NowInSeconds());
  point_debug_down->set_y(-1.1);
  auto* sliding_line_properties = sliding_line->mutable_properties();
  (*sliding_line_properties)["borderWidth"] = "2";
  (*sliding_line_properties)["pointRadius"] = "0";
  (*sliding_line_properties)["lineTension"] = "0";
  (*sliding_line_properties)["fill"] = "false";
  (*sliding_line_properties)["showLine"] = "true";
}

}  // namespace planning
}  // namespace apollo
