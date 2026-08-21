#include "modules/open_space_planning/runtime/open_space_planner.h"

#include <utility>
#include <vector>

#include "modules/open_space_planning/common/problem_validator.h"

namespace apollo {
namespace open_space_planning {

OpenSpacePlanner::OpenSpacePlanner(
    OpenSpacePlannerConfig config,
    std::unique_ptr<RoutePlanner> route_planner,
    std::unique_ptr<TrajectoryPlanner> trajectory_planner,
    std::unique_ptr<TrajectoryValidator> trajectory_validator,
    std::unique_ptr<FallbackPlanner> fallback_planner)
    : config_(config),
      route_planner_(std::move(route_planner)),
      trajectory_planner_(std::move(trajectory_planner)),
      trajectory_validator_(std::move(trajectory_validator)),
      fallback_planner_(std::move(fallback_planner)) {}

Status OpenSpacePlanner::Plan(const PlanningProblem& problem,
                              PlanningResult* result) {
  if (result == nullptr) {
    return {StatusCode::kInvalidInput, "planning result is null"};
  }
  if (route_planner_ == nullptr || trajectory_planner_ == nullptr ||
      trajectory_validator_ == nullptr || fallback_planner_ == nullptr) {
    return {StatusCode::kInternalError,
            "open-space planning pipeline is incomplete"};
  }

  const Status input_status = ProblemValidator::Validate(problem);
  if (!input_status.ok()) {
    return input_status;
  }
  if (config_.maximum_route_candidates == 0) {
    return {StatusCode::kInvalidInput,
            "maximum route candidate count must be positive"};
  }

  std::vector<RouteCandidate> candidates;
  const RoutePlanningRequest route_request{
      problem, config_.maximum_route_candidates};
  const Status route_status = route_planner_->Plan(route_request, &candidates);
  if (!route_status.ok() || candidates.empty()) {
    return PlanFallback(problem, result);
  }

  for (const auto& candidate : candidates) {
    if (!RevisionsMatch(problem, candidate) || candidate.skeleton.empty()) {
      continue;
    }

    PhysicalTrajectory trajectory;
    const TrajectoryPlanningRequest trajectory_request{problem, candidate};
    const Status trajectory_status =
        trajectory_planner_->Plan(trajectory_request, &trajectory);
    if (!trajectory_status.ok() || trajectory.points.empty() ||
        !RevisionsMatch(problem, trajectory)) {
      continue;
    }

    const TrajectoryValidationRequest validation_request{problem, trajectory};
    ValidationReport validation =
        trajectory_validator_->Validate(validation_request);
    if (!validation.safe) {
      continue;
    }

    result->outcome = PlanningOutcome::kTrajectory;
    result->active_route = candidate;
    result->trajectory = std::move(trajectory);
    result->validation = std::move(validation);
    return Status::Ok();
  }

  return PlanFallback(problem, result);
}

bool OpenSpacePlanner::RevisionsMatch(
    const PlanningProblem& problem, const RouteCandidate& route) const {
  return route.map_revision == problem.grid_map->revision &&
         route.goal_revision == problem.goal.revision;
}

bool OpenSpacePlanner::RevisionsMatch(
    const PlanningProblem& problem,
    const PhysicalTrajectory& trajectory) const {
  return trajectory.map_revision == problem.grid_map->revision &&
         trajectory.goal_revision == problem.goal.revision;
}

Status OpenSpacePlanner::PlanFallback(const PlanningProblem& problem,
                                      PlanningResult* result) {
  PhysicalTrajectory fallback;
  const FallbackPlanningRequest fallback_request{problem};
  const Status fallback_status =
      fallback_planner_->Plan(fallback_request, &fallback);
  if (!fallback_status.ok() || fallback.points.empty() ||
      !RevisionsMatch(problem, fallback)) {
    return {StatusCode::kFallbackFailed,
            fallback_status.ok() ? "fallback trajectory is invalid"
                                 : fallback_status.message()};
  }

  const TrajectoryValidationRequest validation_request{problem, fallback};
  ValidationReport validation =
      trajectory_validator_->Validate(validation_request);
  if (!validation.safe) {
    return {StatusCode::kSafetyValidationFailed,
            "fallback trajectory failed safety validation"};
  }

  result->outcome = PlanningOutcome::kFallbackTrajectory;
  result->active_route = RouteCandidate{};
  result->trajectory = std::move(fallback);
  result->validation = std::move(validation);
  return Status::Ok();
}

}  // namespace open_space_planning
}  // namespace apollo

