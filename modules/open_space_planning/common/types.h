#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace apollo {
namespace open_space_planning {

using CandidateId = std::uint64_t;
using TopologyId = std::uint64_t;
using Revision = std::uint64_t;

enum class Gear {
  kDrive = 0,
  kReverse = 1,
};

enum class CellState : std::uint8_t {
  kFree = 0,
  kOccupied = 1,
  kUnknown = 2,
  kNoDrive = 3,
};

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
};

struct VehicleState {
  Pose2d pose;
  double longitudinal_velocity = 0.0;
  double longitudinal_acceleration = 0.0;
  double steering_angle = 0.0;
  Gear gear = Gear::kDrive;
  double timestamp_sec = 0.0;
};

struct VehicleModel {
  double wheel_base = 0.0;
  double front_edge_to_center = 0.0;
  double back_edge_to_center = 0.0;
  double left_edge_to_center = 0.0;
  double right_edge_to_center = 0.0;
  double maximum_steering_angle = 0.0;
  double maximum_curvature = 0.0;
};

struct GridMap {
  std::string frame_id;
  double timestamp_sec = 0.0;
  Pose2d origin;
  double resolution = 0.0;
  std::size_t width = 0;
  std::size_t height = 0;
  Revision revision = 0;
  std::vector<CellState> cell_state;
  std::vector<std::uint8_t> traversal_cost;
  std::vector<std::uint32_t> semantic_flags;

  std::size_t cell_count() const { return width * height; }
};

struct GoalState {
  Pose2d pose;
  double position_tolerance = 0.0;
  double heading_tolerance = 0.0;
  bool allow_reverse = false;
  Revision revision = 0;
};

struct PredictedObstacleState {
  double relative_time = 0.0;
  Pose2d pose;
};

struct DynamicObstacle {
  std::string id;
  std::vector<Pose2d> footprint;
  std::vector<PredictedObstacleState> prediction;
};

struct PlanningProblem {
  std::shared_ptr<const GridMap> grid_map;
  VehicleState start;
  VehicleModel vehicle;
  GoalState goal;
  std::vector<DynamicObstacle> dynamic_obstacles;
  double planning_timestamp_sec = 0.0;
};

struct GeometricPathPoint {
  Pose2d pose;
  double s = 0.0;
  double curvature = 0.0;
  double curvature_derivative = 0.0;
  Gear gear = Gear::kDrive;
};

struct CorridorSample {
  double s = 0.0;
  double minimum_lateral_offset = 0.0;
  double maximum_lateral_offset = 0.0;
};

struct GearSegment {
  std::size_t begin_index = 0;
  std::size_t end_index = 0;
  Gear gear = Gear::kDrive;
};

struct RouteCandidate {
  CandidateId id = 0;
  TopologyId topology_id = 0;
  std::vector<GeometricPathPoint> skeleton;
  std::vector<CorridorSample> corridor;
  std::vector<GearSegment> gear_segments;
  double search_cost = std::numeric_limits<double>::infinity();
  double minimum_clearance = 0.0;
  Revision map_revision = 0;
  Revision goal_revision = 0;
};

struct PhysicalTrajectoryPoint {
  Pose2d pose;
  double s = 0.0;
  double curvature = 0.0;
  double curvature_derivative = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double relative_time = 0.0;
  Gear gear = Gear::kDrive;
};

struct PhysicalTrajectory {
  CandidateId source_candidate_id = 0;
  std::vector<PhysicalTrajectoryPoint> points;
  double cost = std::numeric_limits<double>::infinity();
  Revision map_revision = 0;
  Revision goal_revision = 0;
};

struct ValidationIssue {
  std::string rule;
  std::string detail;
  std::size_t trajectory_point_index = 0;
};

struct ValidationReport {
  bool safe = false;
  std::vector<ValidationIssue> issues;
};

enum class PlanningOutcome {
  kTrajectory = 0,
  kFallbackTrajectory,
};

struct PlanningResult {
  PlanningOutcome outcome = PlanningOutcome::kTrajectory;
  RouteCandidate active_route;
  PhysicalTrajectory trajectory;
  ValidationReport validation;
};

}  // namespace open_space_planning
}  // namespace apollo

