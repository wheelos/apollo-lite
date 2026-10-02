// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "modules/localization/core/lane_relative.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>

namespace apollo {
namespace localization {
namespace unified {
namespace {

bool ValidBoundary(const std::vector<Eigen::Vector3d>& points) {
  if (points.size() < 2 || points.size() > 2000) {
    return false;
  }
  for (size_t i = 0; i < points.size(); ++i) {
    if (!points[i].allFinite() ||
        (i > 0 && points[i].x() - points[i - 1].x() <= 1e-6)) {
      return false;
    }
  }
  return true;
}

bool At(const std::vector<Eigen::Vector3d>& points, double x,
        double* y, double* slope) {
  if (x < points.front().x() || x > points.back().x()) {
    return false;
  }
  auto after = std::lower_bound(
      points.begin(), points.end(), x,
      [](const Eigen::Vector3d& point, double value) { return point.x() < value; });
  if (after == points.begin()) {
    ++after;
  }
  const auto& before = *std::prev(after);
  *slope = (after->y() - before.y()) / (after->x() - before.x());
  *y = before.y() + *slope * (x - before.x());
  return true;
}

}  // namespace

Result EvaluateLane(const LaneGeometry& geometry, const LanePolicy& policy,
                    double motion_position_std, double motion_heading_std,
                    LaneRelation* relation) {
  if (relation == nullptr) {
    return {Reason::INVALID_INPUT, "missing lane result"};
  }
  *relation = LaneRelation();
  for (double value : {policy.front, policy.rear, policy.half_width,
                       policy.boundary_std, policy.heading_std,
                       policy.sigma_multiplier, policy.minimum_width,
                       policy.maximum_width}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return {Reason::CONFIG_INVALID, "lane geometry requires qualified budgets"};
    }
  }
  if (!std::isfinite(policy.clearance) || policy.clearance < 0.0 ||
      !std::isfinite(motion_position_std) || motion_position_std < 0.0 ||
      !std::isfinite(motion_heading_std) || motion_heading_std < 0.0 ||
      !std::isfinite(policy.reference_x) || policy.reference_x < 0.0 ||
      policy.maximum_width < policy.minimum_width ||
      !ValidBoundary(geometry.left) || !ValidBoundary(geometry.right)) {
    return {Reason::INVALID_INPUT, "invalid lane boundary or uncertainty"};
  }
  double left = 0.0;
  double right = 0.0;
  double left_slope = 0.0;
  double right_slope = 0.0;
  if (!At(geometry.left, policy.reference_x, &left, &left_slope) ||
      !At(geometry.right, policy.reference_x, &right, &right_slope)) {
    return {Reason::HISTORY_UNAVAILABLE, "lane evidence does not cover declared reference station"};
  }
  relation->width = left - right;
  if (left <= right || relation->width < policy.minimum_width ||
      relation->width > policy.maximum_width ||
      std::abs(std::atan(left_slope) - std::atan(right_slope)) > 0.3) {
    return {Reason::INVALID_INPUT, "lane pair inconsistent or ambiguous"};
  }
  relation->lateral = -(left + right) / 2.0;
  relation->heading = -std::atan((left_slope + right_slope) / 2.0);
  // Conservative sum: local motion and transformed boundary errors correlate.
  relation->lateral_std = policy.boundary_std + motion_position_std;
  relation->heading_std = policy.heading_std + motion_heading_std;
  const double radius = std::hypot(
      std::max(policy.front, policy.rear), policy.half_width);
  const double expansion =
      policy.clearance + policy.sigma_multiplier * relation->lateral_std +
      radius * std::min(2.0, policy.sigma_multiplier * relation->heading_std);
  const double front = policy.front + expansion;
  const double rear = -policy.rear - expansion;
  if (geometry.left.front().x() > rear || geometry.right.front().x() > rear ||
      geometry.left.back().x() < front || geometry.right.back().x() < front) {
    relation->containment_reason = Reason::HISTORY_UNAVAILABLE;
    return {};
  }
  std::vector<double> stations{rear, front, 0.0};
  for (const auto* boundary : {&geometry.left, &geometry.right}) {
    for (const auto& point : *boundary) {
      if (point.x() > rear && point.x() < front) {
        stations.push_back(point.x());
      }
    }
  }
  relation->left_clearance = std::numeric_limits<double>::infinity();
  relation->right_clearance = std::numeric_limits<double>::infinity();
  for (double x : stations) {
    if (!At(geometry.left, x, &left, &left_slope) ||
        !At(geometry.right, x, &right, &right_slope) ||
        left - right < policy.minimum_width ||
        left - right > policy.maximum_width) {
      return {Reason::INVALID_INPUT, "crossing or unsupported lane boundaries"};
    }
    relation->left_clearance =
        std::min(relation->left_clearance, left - policy.half_width - expansion);
    relation->right_clearance =
        std::min(relation->right_clearance, -right - policy.half_width - expansion);
  }
  relation->contained =
      relation->left_clearance > 0.0 && relation->right_clearance > 0.0;
  relation->containment_reason =
      relation->contained ? Reason::NONE : Reason::COVARIANCE_EXCEEDED;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace apollo
