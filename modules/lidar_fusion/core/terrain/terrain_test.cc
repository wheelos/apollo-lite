// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "modules/lidar_fusion/core/terrain/traversability.h"

namespace apollo {
namespace lidar_fusion {
namespace terrain {
namespace {

SurfaceConfig SmallConfig() {
  SurfaceConfig config;
  config.x_min = 0.0;
  config.x_max = 2.0;
  config.y_min = 0.0;
  config.y_max = 2.0;
  config.cell_size = 0.5;
  config.expected_ground_z.reset();
  config.min_anchor_points = 1;
  return config;
}

std::vector<Point> Grid(const std::function<double(std::size_t, std::size_t,
                                                   double, double)>& height) {
  std::vector<Point> points;
  uint32_t id = 100;
  for (std::size_t ix = 0; ix < 4; ++ix) {
    for (std::size_t iy = 0; iy < 4; ++iy) {
      const double x = 0.25 + ix * 0.5;
      const double y = 0.25 + iy * 0.5;
      points.push_back(Point{id++, x, y, height(ix, iy, x, y), 0});
    }
  }
  return points;
}

TerrainEstimate Estimate(const std::vector<Point>& points,
                         std::optional<uint32_t> root = 100,
                         SurfaceConfig config = SmallConfig()) {
  TerrainEstimate result;
  std::string error;
  EXPECT_TRUE(EstimateSurface(points, config, root, &result, &error)) << error;
  return result;
}

TraversabilityLimits DefaultLimits() {
  return TraversabilityLimits{0.3, 0.05, 0.2, 0.2};
}

RiskGrid FullCoverage(const TerrainEstimate& surface) {
  RiskGrid risk;
  risk.nx = surface.nx;
  risk.ny = surface.ny;
  risk.layers.assign(surface.cells.size(), 0);
  risk.observed.assign(surface.cells.size(), 1);
  return risk;
}

TEST(TerrainTest, FlatAndAnalyticRampPreserveSourceIdsAndFitSlope) {
  auto points =
      Grid([](std::size_t, std::size_t, double, double) { return 0.0; });
  auto flat = Estimate(points, 100);
  ASSERT_EQ(flat.status, TerrainEstimate::Status::kSupported);
  ASSERT_TRUE(flat.root_anchor_id.has_value());
  EXPECT_EQ(*flat.root_anchor_id, 100U);
  EXPECT_EQ(flat.points.front().id, 100U);
  EXPECT_TRUE(flat.points.front().ground_candidate);
  EXPECT_EQ(flat.cells.size(), 16U);
  for (const auto& cell : flat.cells) {
    EXPECT_EQ(cell.status, CellStatus::kSupported);
    EXPECT_TRUE(cell.has_slope);
    EXPECT_TRUE(cell.has_roughness);
    EXPECT_NEAR(cell.slope_gradient, 0.0, 1e-12);
    EXPECT_NEAR(cell.roughness_m, 0.0, 1e-12);
  }
  TraversabilityEstimate flat_decision;
  std::string error;
  ASSERT_TRUE(ClassifyTraversability(flat, DefaultLimits(), FullCoverage(flat),
                                     &flat_decision, &error))
      << error;
  EXPECT_TRUE(flat_decision.cells.front().terrain_connected);
  EXPECT_EQ(flat_decision.cells.front().decision_state,
            DecisionState::kCandidate);
  EXPECT_EQ(flat_decision.overall_status,
            TraversabilityStatus::kObservedRootConnected);

  auto ramp_points = Grid([](std::size_t, std::size_t, double x, double y) {
    return 0.2 * x + 0.1 * y;
  });
  auto ramp = Estimate(ramp_points, 100, [] {
    auto config = SmallConfig();
    config.expected_ground_z = 0.075;
    return config;
  }());
  EXPECT_NEAR(ramp.cells[ramp.CellId(1, 1)].slope_gradient,
              std::hypot(0.2, 0.1), 1e-10);
  EXPECT_TRUE(ramp.points.front().ground);
}

TEST(TerrainTest, StepBoundaryIsMeasuredAndDisconnectedBeyondTheRoot) {
  auto points = Grid([](std::size_t ix, std::size_t, double, double) {
    return ix >= 2 ? 0.5 : 0.0;
  });
  auto surface = Estimate(points);
  EXPECT_EQ(surface.cells[surface.CellId(0, 0)].status, CellStatus::kSupported);
  EXPECT_EQ(surface.cells[surface.CellId(3, 0)].status,
            CellStatus::kUnknownGap);
  const auto& edge = surface.cells[surface.CellId(1, 0)];
  EXPECT_EQ(edge.step_status, StepStatus::kObservedBoundaryDiscontinuity);
  EXPECT_NEAR(edge.step_m, 0.5, 1e-12);

  TraversabilityEstimate result;
  std::string error;
  ASSERT_TRUE(ClassifyTraversability(surface, DefaultLimits(),
                                     FullCoverage(surface), &result, &error))
      << error;
  EXPECT_TRUE(result.cells[surface.CellId(1, 0)].reason_bits &
              kReasonStepExceeds);
  EXPECT_FALSE(result.cells[surface.CellId(2, 0)].terrain_connected);
  EXPECT_EQ(result.overall_status,
            TraversabilityStatus::kObservedRootConnected);
}

TEST(TerrainTest, EnclosedObservedFloorReportsRelativeDepthAndExtent) {
  auto points = Grid([](std::size_t ix, std::size_t iy, double, double) {
    return ix == 1 && iy == 1 ? -0.5 : 0.0;
  });
  points.push_back(Point{1000, 0.75, 0.75, -0.5, 0});
  points.push_back(Point{1001, 0.75, 0.75, -0.5, 0});
  auto surface = Estimate(points);
  const auto& floor = surface.cells[surface.CellId(1, 1)];
  EXPECT_EQ(floor.status, CellStatus::kUnknownGap);
  EXPECT_EQ(floor.pit_status, PitStatus::kMeasured);
  ASSERT_TRUE(floor.has_pit_depth);
  EXPECT_NEAR(floor.pit_depth_m, 0.5, 1e-10);
  EXPECT_NEAR(floor.pit_width_x_m, 0.5, 1e-12);
  EXPECT_NEAR(floor.pit_width_y_m, 0.5, 1e-12);

  TraversabilityEstimate result;
  std::string error;
  ASSERT_TRUE(ClassifyTraversability(surface, DefaultLimits(),
                                     FullCoverage(surface), &result, &error))
      << error;
  EXPECT_EQ(result.cells[surface.CellId(1, 1)].terrain_state,
            TerrainState::kBlocked);
  EXPECT_TRUE(result.cells[surface.CellId(1, 1)].reason_bits &
              kReasonPitExceeds);
}

TEST(TerrainTest, BridgeLayersRemainSeparateAndClassifierMarksAmbiguity) {
  auto points =
      Grid([](std::size_t, std::size_t, double, double) { return 0.0; });
  const auto base = points;
  for (const auto& point : base) {
    points.push_back(Point{point.id + 1000, point.x, point.y, 4.0, 0});
  }
  auto surface = Estimate(points, 100);
  const auto& cell = surface.cells[surface.CellId(1, 1)];
  EXPECT_EQ(cell.observed_layers.size(), 2U);
  EXPECT_TRUE(cell.observed_layers.front().selected);
  EXPECT_FALSE(cell.observed_layers.back().selected);
  EXPECT_EQ(cell.step_status, StepStatus::kObservedWithinCellLayerSpan);
  EXPECT_NEAR(cell.observed_layer_span_m, 4.0, 1e-12);
  EXPECT_FALSE(surface.points[16].ground_candidate);

  TraversabilityEstimate result;
  std::string error;
  ASSERT_TRUE(ClassifyTraversability(surface, DefaultLimits(),
                                     FullCoverage(surface), &result, &error))
      << error;
  EXPECT_EQ(result.cells[surface.CellId(1, 1)].terrain_state,
            TerrainState::kUnknown);
  EXPECT_TRUE(result.cells[surface.CellId(1, 1)].reason_bits &
              kReasonLayerAmbiguity);
}

TEST(TerrainTest, SelectedUpperSurfaceKeepsLowerLayerPitStatusUnknown) {
  auto points =
      Grid([](std::size_t, std::size_t, double, double) { return 0.0; });
  uint32_t id = 2000;
  for (const CellIndex cell :
       {CellIndex{1, 1}, CellIndex{1, 2}, CellIndex{2, 1}, CellIndex{2, 2}}) {
    const double x = 0.25 + cell.x * 0.5;
    const double y = 0.25 + cell.y * 0.5;
    for (int repeat = 0; repeat < 3; ++repeat) {
      points.push_back(Point{id++, x, y, -0.5, 0});
    }
  }
  auto surface = Estimate(points, 100);
  for (const CellIndex cell :
       {CellIndex{1, 1}, CellIndex{1, 2}, CellIndex{2, 1}, CellIndex{2, 2}}) {
    const auto& observed = surface.cells[surface.CellId(cell.x, cell.y)];
    EXPECT_EQ(observed.status, CellStatus::kSupported);
    EXPECT_EQ(observed.observed_layers.size(), 2U);
    EXPECT_EQ(observed.pit_status, PitStatus::kUnknownLayerAmbiguity);
    EXPECT_FALSE(observed.has_pit_depth);
  }
  EXPECT_TRUE(surface.points.front().ground_candidate);
  EXPECT_FALSE(surface.points[16].ground_candidate);

  TraversabilityEstimate decision;
  std::string error;
  ASSERT_TRUE(ClassifyTraversability(surface, DefaultLimits(),
                                     FullCoverage(surface), &decision, &error))
      << error;
  const auto& ambiguous = decision.cells[surface.CellId(1, 1)];
  EXPECT_EQ(ambiguous.terrain_state, TerrainState::kUnknown);
  EXPECT_TRUE(ambiguous.reason_bits & kReasonLayerAmbiguity);
}

TEST(TerrainTest, SparseAnchorAndInvalidInputsFailClosed) {
  std::vector<Point> sparse = {Point{10, 0.25, 0.25, 0.0, 0},
                               Point{11, 0.25, 0.75, 0.0, 0},
                               Point{12, 0.25, 1.25, 0.0, 0}};
  TerrainEstimate surface;
  std::string error;
  ASSERT_TRUE(EstimateSurface(sparse, SmallConfig(), 10, &surface, &error))
      << error;
  EXPECT_EQ(surface.status, TerrainEstimate::Status::kUnknownNoAnchor);
  EXPECT_FALSE(surface.root_cell.has_value());

  auto duplicate = sparse;
  duplicate[1].id = duplicate[0].id;
  EXPECT_FALSE(EstimateSurface(duplicate, SmallConfig(), 10, &surface, &error));
  EXPECT_NE(error.find("unique"), std::string::npos);
  EXPECT_FALSE(EstimateSurface(sparse, SmallConfig(), 0, &surface, &error));
  EXPECT_NE(error.find("root_anchor_id"), std::string::npos);

  auto invalid_config = SmallConfig();
  invalid_config.x_max = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(EstimateSurface(sparse, invalid_config, 10, &surface, &error));
  invalid_config = SmallConfig();
  invalid_config.x_max = 1000000.0;
  EXPECT_FALSE(EstimateSurface(sparse, invalid_config, 10, &surface, &error));

  auto non_finite =
      Grid([](std::size_t, std::size_t, double, double) { return 0.0; });
  non_finite.push_back(
      Point{3000, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0});
  surface = Estimate(non_finite);
  EXPECT_EQ(surface.points.back().cell_id, -1);
  EXPECT_FALSE(surface.points.back().ground_candidate);
}

TEST(TerrainTest, RiskCoverageAndIndependentPointRiskAreFailClosed) {
  auto points =
      Grid([](std::size_t, std::size_t, double, double) { return 0.0; });
  points.front().risk_bits = kMeasuredRisk;
  auto surface = Estimate(points);
  EXPECT_TRUE(surface.points.front().ground_candidate);
  EXPECT_FALSE(surface.points.front().ground);
  RiskGrid risk;
  risk.nx = surface.nx;
  risk.ny = surface.ny;
  risk.layers.assign(surface.cells.size(), 0);
  risk.observed.assign(surface.cells.size(), 1);
  risk.layers[surface.CellId(2, 1)] = kObjectRisk;
  risk.layers[surface.CellId(3, 1)] = kUnknownRisk;
  TraversabilityEstimate result;
  std::string error;
  ASSERT_TRUE(
      ClassifyTraversability(surface, DefaultLimits(), risk, &result, &error))
      << error;
  EXPECT_EQ(result.cells[surface.CellId(0, 0)].risk_layers, kUnknownRisk);
  EXPECT_EQ(result.cells[surface.CellId(0, 0)].decision_state,
            DecisionState::kUnknown);
  EXPECT_EQ(result.cells[surface.CellId(2, 1)].decision_state,
            DecisionState::kBlocked);
  EXPECT_EQ(result.cells[surface.CellId(3, 1)].decision_state,
            DecisionState::kUnknown);
  EXPECT_FALSE(result.cells[surface.CellId(1, 0)].risk_constrained_connected);

  risk.observed[surface.CellId(1, 0)] = 0;
  EXPECT_TRUE(
      ClassifyTraversability(surface, DefaultLimits(), risk, &result, &error));
  EXPECT_FALSE(result.cells[surface.CellId(1, 0)].risk_constrained_candidate);
}

TEST(TerrainTest, RootConnectivityDoesNotReseedAcrossDiagonalOrRiskWall) {
  auto points =
      Grid([](std::size_t, std::size_t, double, double) { return 0.0; });
  auto surface = Estimate(points);
  TraversabilityLimits limits = DefaultLimits();
  for (std::size_t y = 0; y < surface.ny; ++y) {
    auto& cell = surface.cells[surface.CellId(1, y)];
    cell.roughness_m = 0.1;
  }
  RiskGrid risk = FullCoverage(surface);
  TraversabilityEstimate result;
  std::string error;
  ASSERT_TRUE(ClassifyTraversability(surface, limits, risk, &result, &error))
      << error;
  EXPECT_FALSE(result.cells[surface.CellId(2, 0)].terrain_connected);
  EXPECT_TRUE(result.cells[surface.CellId(0, 0)].terrain_connected);

  surface = Estimate(points);
  risk = FullCoverage(surface);
  for (std::size_t y = 0; y < surface.ny; ++y) {
    risk.layers[surface.CellId(1, y)] = kObjectRisk;
  }
  ASSERT_TRUE(ClassifyTraversability(surface, limits, risk, &result, &error))
      << error;
  EXPECT_TRUE(result.cells[surface.CellId(3, 0)].terrain_connected);
  EXPECT_FALSE(result.cells[surface.CellId(3, 0)].risk_constrained_connected);

  surface = Estimate(points);
  surface.cells[surface.CellId(0, 0)].roughness_m = 0.1;
  ASSERT_TRUE(ClassifyTraversability(surface, limits, FullCoverage(surface),
                                     &result, &error))
      << error;
  EXPECT_EQ(result.overall_status, TraversabilityStatus::kObservedRootVetoed);
  EXPECT_FALSE(result.cells[surface.CellId(0, 0)].terrain_connected);
}

}  // namespace
}  // namespace terrain
}  // namespace lidar_fusion
}  // namespace apollo
