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

#include "modules/localization_health/directional_constraint_tracker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

#include "modules/localization_health/reason_aggregator.h"

namespace apollo {
namespace localization {

namespace {

bool HasRevokingFault(uint64_t active_reasons) {
  const HealthReason reasons[] = {
      REASON_ASSESSMENT_TIMEOUT,    REASON_POSE_TIMEOUT,
      REASON_TIMESTAMP_REGRESSION,  REASON_DATA_TOO_OLD,
      REASON_SEQUENCE_ERROR,        REASON_SESSION_CHANGED,
      REASON_TIME_SYNC_ERROR,       REASON_NON_FINITE_OUTPUT,
      REASON_INVALID_QUATERNION,    REASON_INVALID_COVARIANCE,
      REASON_KINEMATIC_VIOLATION,   REASON_UNDECLARED_POSE_JUMP,
      REASON_ESTIMATOR_NOT_RUNNING, REASON_LOCAL_POSE_INVALID,
      REASON_LIO_DIVERGED,
  };
  for (const auto reason : reasons) {
    if (ReasonAggregator::HasReason(active_reasons, reason)) {
      return true;
    }
  }
  return false;
}

bool IsFinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

std::array<double, 6> NormalizedProjection(
    const DirectionalConstraint& constraint) {
  std::array<double, 6> result{};
  int dominant = 0;
  for (int axis = 1; axis < 6; ++axis) {
    if (std::abs(constraint.projection(axis)) >
        std::abs(constraint.projection(dominant))) {
      dominant = axis;
    }
  }
  for (int axis = 0; axis < 6; ++axis) {
    result[axis] =
        constraint.projection(axis) / std::abs(constraint.projection(dominant));
  }
  return result;
}

bool SameProjection(const std::array<double, 6>& first,
                    const std::array<double, 6>& second) {
  // Numerical equivalence only, not permission to rotate a physical constraint.
  double difference = 0.0;
  double reversed = 0.0;
  for (int axis = 0; axis < 6; ++axis) {
    difference = std::max(difference, std::abs(first[axis] - second[axis]));
    reversed = std::max(reversed, std::abs(first[axis] + second[axis]));
  }
  return std::min(difference, reversed) <= 1e-8;
}

}  // namespace

void DirectionalConstraintTracker::Reset() { recovery_states_.clear(); }

bool DirectionalConstraintTracker::IsStructurallyValid(
    const DirectionalConstraint& constraint) {
  if (constraint.id().empty() || constraint.reference_frame().empty() ||
      constraint.reference_id().empty() || constraint.source().empty() ||
      constraint.projection_size() != 6 ||
      !IsFinitePositive(constraint.standard_deviation()) ||
      !IsFinitePositive(constraint.error_budget()) ||
      constraint.standard_deviation() > constraint.error_budget() ||
      !IsFinitePositive(constraint.evaluation_time()) ||
      !IsFinitePositive(constraint.valid_until())) {
    return false;
  }

  bool nonzero = false;
  for (int i = 0; i < constraint.projection_size(); ++i) {
    if (!std::isfinite(constraint.projection(i))) {
      return false;
    }
    nonzero = nonzero || constraint.projection(i) != 0.0;
  }
  if (!nonzero) {
    return false;
  }

  if (constraint.unit() == "meters") {
    return true;
  }
  if (constraint.unit() == "radians") {
    return constraint.projection(0) == 0.0 &&
           constraint.projection(1) == 0.0 &&
           constraint.projection(2) == 0.0;
  }
  return false;
}

bool DirectionalConstraintTracker::IsProjection(
    const DirectionalConstraint& constraint, int coefficient_index,
    const std::string& unit) {
  if (constraint.mode() != DIRECTION_CONSTRAINED ||
      constraint.unit() != unit || constraint.projection_size() != 6) {
    return false;
  }
  for (int i = 0; i < constraint.projection_size(); ++i) {
    if (!std::isfinite(constraint.projection(i)) ||
        (i == coefficient_index && constraint.projection(i) == 0.0) ||
        (i != coefficient_index && constraint.projection(i) != 0.0)) {
      return false;
    }
  }
  return true;
}

std::string DirectionalConstraintTracker::MakeKey(
    const std::string& session_id, uint64_t odom_generation,
    const DirectionalConstraint& constraint) {
  std::ostringstream key;
  key << session_id << '\x1f' << odom_generation << '\x1f' << constraint.id()
      << '\x1f' << constraint.reference_frame() << '\x1f'
      << constraint.reference_id() << '\x1f' << constraint.source()
      << '\x1f' << constraint.unit();
  return key.str();
}

std::vector<DirectionalConstraint> DirectionalConstraintTracker::Evaluate(
    const LocalizationAssessment& assessment,
    const LocalizationEstimate* pose, double now_sec,
    uint64_t active_reasons, const LocalizationHealthConfig& config) {
  std::vector<DirectionalConstraint> output;
  if (pose == nullptr || HasRevokingFault(active_reasons) ||
      !assessment.estimator_running() || !assessment.pose_valid() ||
      !assessment.output_continuous() || assessment.session_id().empty() ||
      assessment.odom_generation() == 0 ||
      !std::isfinite(assessment.local_measurement_time()) ||
      std::abs(assessment.local_measurement_time() -
               pose->measurement_time()) > config.pose_timeout_threshold()) {
    Reset();
    return output;
  }

  std::set<std::string> active_keys;
  std::map<std::string, int> counts;
  for (const auto& input : assessment.directional_constraints()) {
    ++counts[MakeKey(assessment.session_id(), assessment.odom_generation(),
                     input)];
  }
  for (const auto& input : assessment.directional_constraints()) {
    DirectionalConstraint result = input;
    result.set_mode(DIRECTION_UNAVAILABLE);

    if (!IsStructurallyValid(input)) {
      output.push_back(result);
      continue;
    }

    const std::string key =
        MakeKey(assessment.session_id(), assessment.odom_generation(), input);
    if (counts[key] != 1) {
      recovery_states_.erase(key);
      output.push_back(result);
      continue;
    }
    active_keys.insert(key);

    bool evaluation_fresh =
        input.evaluation_time() <= now_sec &&
        now_sec - input.evaluation_time() <=
            config.assessment_timeout_threshold();
    bool valid_now = input.valid_until() >= now_sec;
    if (!evaluation_fresh || !valid_now) {
      recovery_states_.erase(key);
      output.push_back(result);
      continue;
    }

    if (input.mode() == DIRECTION_PROPAGATED) {
      recovery_states_.erase(key);
      result.set_mode(DIRECTION_PROPAGATED);
      output.push_back(result);
      continue;
    }

    bool observation_fresh =
        IsFinitePositive(input.last_observation_time()) &&
        input.last_observation_time() <= now_sec &&
        now_sec - input.last_observation_time() <=
            config.assessment_timeout_threshold();
    if (input.mode() != DIRECTION_CONSTRAINED || !input.independent() ||
        input.observation_sequence() == 0 || !observation_fresh) {
      recovery_states_.erase(key);
      output.push_back(result);
      continue;
    }

    auto& state = recovery_states_[key];
    const auto projection = NormalizedProjection(input);
    if (state.observation_count != 0 &&
        !SameProjection(state.projection, projection)) {
      state = RecoveryState();
    }
    if (state.observation_count == 0) {
      state.projection = projection;
      state.last_sequence = input.observation_sequence();
      state.first_observation_time = input.last_observation_time();
      state.last_observation_time = input.last_observation_time();
      state.observation_count = 1;
    } else if (input.observation_sequence() > state.last_sequence &&
               input.last_observation_time() >
                   state.last_observation_time) {
      state.last_sequence = input.observation_sequence();
      state.last_observation_time = input.last_observation_time();
      if (state.observation_count < std::numeric_limits<uint32_t>::max()) {
        ++state.observation_count;
      }
    } else if (input.observation_sequence() != state.last_sequence ||
               input.last_observation_time() !=
                   state.last_observation_time) {
      state.last_sequence = input.observation_sequence();
      state.first_observation_time = input.last_observation_time();
      state.last_observation_time = input.last_observation_time();
      state.observation_count = 1;
    }

    if (state.observation_count >= config.direction_recovery_frames() &&
        state.last_observation_time - state.first_observation_time >=
            config.direction_recovery_min_span()) {
      result.set_mode(DIRECTION_CONSTRAINED);
    }
    output.push_back(result);
  }

  for (auto it = recovery_states_.begin(); it != recovery_states_.end();) {
    if (active_keys.count(it->first) == 0) {
      it = recovery_states_.erase(it);
    } else {
      ++it;
    }
  }
  return output;
}

bool DirectionalConstraintTracker::HasLaneLateralAndHeading(
    const std::vector<DirectionalConstraint>& constraints) {
  std::set<std::string> lateral_references;
  std::set<std::string> heading_references;
  for (const auto& constraint : constraints) {
    std::string reference = constraint.reference_frame() + '\x1f' +
                            constraint.reference_id() + '\x1f' +
                            constraint.source();
    if (constraint.id() == "lane-lateral" &&
        IsProjection(constraint, 1, "meters")) {
      lateral_references.insert(reference);
    }
    if (constraint.id() == "lane-heading" &&
        IsProjection(constraint, 5, "radians")) {
      heading_references.insert(reference);
    }
  }
  for (const auto& reference : lateral_references) {
    if (heading_references.count(reference) != 0) {
      return true;
    }
  }
  return false;
}

}  // namespace localization
}  // namespace apollo
