// Copyright 2026 WheelOS. All Rights Reserved.

#include "modules/lidar_fusion/core/terrain/surface.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {

const char* ToString(CellStatus status) {
  switch (status) {
    case CellStatus::kUnknownEmpty:
      return "unknown_empty";
    case CellStatus::kUnknownNoAnchor:
      return "unknown_no_anchor";
    case CellStatus::kUnknownGap:
      return "unknown_gap";
    case CellStatus::kSupported:
      return "supported";
  }
  return "invalid";
}

const char* ToString(StepStatus status) {
  switch (status) {
    case StepStatus::kUnknownUnsupported:
      return "unknown_unsupported";
    case StepStatus::kNoDiscontinuityObserved:
      return "no_discontinuity_observed";
    case StepStatus::kObservedBoundaryDiscontinuity:
      return "observed_boundary_discontinuity";
    case StepStatus::kObservedWithinCellLayerSpan:
      return "observed_within_cell_layer_span";
    case StepStatus::kLayerSpanAndBoundaryDiscontinuity:
      return "layer_span_and_boundary_discontinuity";
  }
  return "invalid";
}

const char* ToString(PitStatus status) {
  switch (status) {
    case PitStatus::kUnknownNoFloor:
      return "unknown_no_floor";
    case PitStatus::kUnknownNoRim:
      return "unknown_no_rim";
    case PitStatus::kUnknownInsufficientFloor:
      return "unknown_insufficient_floor";
    case PitStatus::kUnknownLayerAmbiguity:
      return "unknown_layer_ambiguity";
    case PitStatus::kNoDepressionObserved:
      return "no_depression_observed";
    case PitStatus::kMeasured:
      return "measured";
  }
  return "invalid";
}

const char* ToString(TerrainEstimate::Status status) {
  switch (status) {
    case TerrainEstimate::Status::kUnknownNoAnchor:
      return "unknown_no_anchor";
    case TerrainEstimate::Status::kSupported:
      return "supported";
  }
  return "invalid";
}

}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
