# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Ground reference contracts independent of the recorded evaluation labels."""

import unittest
from types import SimpleNamespace

import numpy as np
from ground_continuity import (
    ContinuityOptions,
    connected_ground,
    instance_retention,
    sparse_obstacles,
)
from ground_experiment import (
    RadialOptions,
    measured_cells,
    promotion_checks,
    radial_ground,
)
from ground_validation_closure import common_comparison, components


def config():
    return SimpleNamespace(
        min_x=-60,
        max_x=60,
        min_y=-40,
        max_y=40,
        expected_ground_z=-1.73,
        ground_seed_band=0.3,
        ground_max_slope=0.3,
        ground_max_residual=0.08,
        ground_tolerance=0.1,
        obstacle_min_height=0.15,
        clearance_height=2.5,
        resolution=0.25,
    )


def ray(start, stop, height):
    radius = np.arange(start, stop, 0.5)
    z = height(radius) if callable(height) else np.full(len(radius), height)
    return np.column_stack((radius, np.zeros(len(radius)), z))


class RadialGroundTest(unittest.TestCase):
    def test_diagnostic_trace_preserves_estimator_and_reports_gap_boundary(self):
        points = np.vstack((ray(1, 5, -1.73), ray(15, 20, -1.73)))
        trace = {}
        before = radial_ground(points, config())
        after = radial_ground(points, config(), trace=trace)
        np.testing.assert_array_equal(before, after)
        events = trace[180]["events"]
        self.assertIn("observed_gap_reset", [event["reason"] for event in events])
        gap = next(event for event in events if event["reason"] == "observed_gap_reset")
        self.assertEqual(gap["sample"][1], 15)

    def test_slope_continues_beyond_global_seed_band_and_original_order(self):
        points = ray(1, 25, lambda r: -1.73 + 0.08 * (r - 1))
        order = np.random.default_rng(42).permutation(len(points))
        original = points.copy()
        estimate = radial_ground(points[order], config())
        np.testing.assert_allclose(estimate, points[order, 2], atol=1e-12)
        np.testing.assert_array_equal(points, original)
        self.assertGreater(points[-1, 2] + 1.73, config().ground_seed_band)

    def test_roof_is_not_an_initial_ground_anchor(self):
        points = ray(1, 15, -0.5)
        self.assertTrue(np.isnan(radial_ground(points, config())).all())

    def test_ground_below_obstacle_preserves_obstacle_height(self):
        floor = ray(1, 10, -1.73)
        obstacle = floor[4:8].copy()
        obstacle[:, 2] += 0.5
        points = np.vstack((floor, obstacle))
        estimate = radial_ground(points, config())
        np.testing.assert_allclose(estimate, -1.73, atol=1e-12)
        ground = np.abs(points[:, 2] - estimate) <= config().ground_tolerance
        self.assertTrue(ground[: len(floor)].all())
        self.assertFalse(ground[len(floor) :].any())
        self.assertEqual(
            len(
                measured_cells(
                    points, estimate, ground, np.zeros(len(points), bool), config()
                )
            ),
            len(obstacle),
        )

    def test_step_and_long_gap_cannot_start_remote_ground(self):
        floor = ray(1, 5, -1.73)
        step = ray(5, 10, -0.93)
        remote = ray(15, 20, -1.73)
        estimate = radial_ground(np.vstack((floor, step, remote)), config())
        np.testing.assert_allclose(estimate[: len(floor)], -1.73, atol=1e-12)
        self.assertTrue(np.isnan(estimate[len(floor) :]).all())

    def test_one_sector_cannot_support_an_unobserved_neighbor(self):
        floor = ray(1, 5, -1.73)
        other = ray(15, 20, -1.73)
        angle = np.deg2rad(2)
        other[:, 1] = other[:, 0] * np.sin(angle)
        other[:, 0] *= np.cos(angle)
        result = radial_ground(np.vstack((floor, other)), config())
        self.assertTrue(np.isnan(result[len(floor) :]).all())

    def test_sparse_invalid_and_outside_roi_remain_unsupported(self):
        points = np.array(
            [
                [1, 0, -1.73],
                [1.5, 0, -1.73],
                [np.nan, 0, 0],
                [np.inf, 0, 0],
                [60, 0, -1.73],
            ]
        )
        self.assertTrue(np.isnan(radial_ground(points, config())).all())
        self.assertEqual(len(radial_ground(np.empty((0, 3)), config())), 0)
        with self.assertRaisesRegex(ValueError, "Nx3"):
            radial_ground(np.zeros((3, 2)), config())

    def test_unsupported_points_occupy_but_ground_alone_does_not(self):
        points = np.array([[1, 0, -1.73], [2, 0, -1.73], [3, 0, -1.23]])
        z = np.array([-1.73, np.nan, -1.73])
        cells = measured_cells(
            points,
            z,
            np.array([True, False, False]),
            np.array([False, False, False]),
            config(),
        )
        self.assertEqual(cells, {(248, 160), (252, 160)})

    def test_invalid_options_and_negative_slope_rejected(self):
        for options in (
            RadialOptions(sectors=0),
            RadialOptions(bin_size=float("nan")),
            RadialOptions(max_gap=0.1),
            RadialOptions(min_bins=2),
        ):
            with self.assertRaises(ValueError):
                radial_ground(np.empty((0, 3)), config(), options)
        bad = config()
        bad.ground_max_slope = -1
        with self.assertRaisesRegex(ValueError, "ground contract"):
            radial_ground(np.empty((0, 3)), bad)


class ContinuityTest(unittest.TestCase):
    def test_common_seed_comparison_does_not_expand_or_select_remote_island(self):
        old = [{(6, 0), (7, 0)}, {(0, 0)}]
        new = [{(6, 0), (7, 0), (8, 0)}, {(0, 0)}]
        report = common_comparison(old, new, 4, 3)
        self.assertEqual(report["common_seed_xy"], [[0, 0]])
        self.assertEqual(report["baseline"]["same_seed_selected_cells"], 1)
        self.assertEqual(report["candidate"]["same_seed_selected_cells"], 1)
        self.assertEqual(report["candidate"]["largest_component_cells"], 3)
        empty = common_comparison([{(6, 0)}], [{(7, 0)}], 4, 4)
        self.assertEqual(empty["status"], "BLOCKED")
        self.assertIsNone(empty["candidate"]["same_seed_selected_cells"])

    def test_component_inventory_preserves_obstacle_and_height_boundaries(self):
        points = np.array([[0.25, 0.25, 0], [0.75, 0.25, 0], [1.25, 0.25, 0.4]])
        inventory = components(
            points, points[:, 2], np.ones(3, bool), np.zeros(3, bool)
        )
        self.assertEqual(inventory, [{(0, 0), (1, 0)}, {(2, 0)}])
        blocked = components(
            points, points[:, 2], np.ones(3, bool), np.array([False, True, False])
        )
        self.assertEqual(blocked, [{(0, 0)}, {(2, 0)}])

    def test_remote_island_and_unobserved_hole_are_not_filled(self):
        points = np.array([[x, 0.25, 0] for x in (0.25, 0.75, 1.25, 2.25, 10.25)])
        report, cells = connected_ground(
            points,
            points[:, 2],
            np.ones(5, bool),
            np.zeros(5, bool),
            ContinuityOptions(seed_radius=1),
        )
        self.assertEqual(cells, {(0, 0), (1, 0), (2, 0)})
        self.assertEqual(report["components"], 3)
        self.assertNotIn((3, 0), cells)

    def test_step_and_supported_obstacle_block_connectivity(self):
        points = np.array([[0.25, 0.25, 0], [0.75, 0.25, 0], [1.25, 0.25, 0.4]])
        _, cells = connected_ground(
            points,
            points[:, 2],
            np.ones(3, bool),
            np.zeros(3, bool),
            ContinuityOptions(seed_radius=0.5),
        )
        self.assertEqual(cells, {(0, 0), (1, 0)})
        _, cells = connected_ground(
            points,
            np.zeros(3),
            np.ones(3, bool),
            np.array([False, True, False]),
            ContinuityOptions(seed_radius=0.5),
        )
        self.assertEqual(cells, {(0, 0)})

    def test_no_near_ego_support_is_not_a_valid_connected_region(self):
        points = np.array([[10, 0, 0]])
        report, cells = connected_ground(
            points, np.zeros(1), np.ones(1, bool), np.zeros(1, bool)
        )
        self.assertFalse(report["seed_observed"])
        self.assertFalse(cells)

    def test_blind_zone_uses_nearest_observed_ring_without_filling_it(self):
        points = np.array([[x, 0.25, 0] for x in (4.25, 4.75, 5.25)])
        report, cells = connected_ground(
            points, np.zeros(3), np.ones(3, bool), np.zeros(3, bool)
        )
        self.assertTrue(report["seed_expanded_outside_blind_zone"])
        self.assertEqual(cells, {(8, 0), (9, 0), (10, 0)})
        report, cells = connected_ground(
            points,
            np.zeros(3),
            np.ones(3, bool),
            np.zeros(3, bool),
            ContinuityOptions(max_seed_radius=3),
        )
        self.assertFalse(report["seed_observed"])
        self.assertFalse(cells)

    def test_diagonal_cells_are_not_connected_and_slope_is(self):
        points = np.array([[0.25, 0.25, 0], [0.75, 0.75, 0.1]])
        _, cells = connected_ground(
            points,
            points[:, 2],
            np.ones(2, bool),
            np.zeros(2, bool),
            ContinuityOptions(seed_radius=0.5),
        )
        self.assertEqual(cells, {(0, 0)})
        points[1, 1] = 0.25
        _, cells = connected_ground(
            points,
            points[:, 2],
            np.ones(2, bool),
            np.zeros(2, bool),
            ContinuityOptions(seed_radius=0.5),
        )
        self.assertEqual(cells, {(0, 0), (1, 0)})

    def test_isolated_points_removed_but_dense_thin_support_kept(self):
        points = np.array(
            [
                [0, 0, 0],
                [0.01, 0, 0],
                [0.02, 0, 0],
                [0.03, 0, 0],
                [5, 0, 0],
                [5.1, 0, 0],
                [10, 0, 0],
            ]
        )
        keep, stats = sparse_obstacles(points, np.ones(7, bool))
        np.testing.assert_array_equal(keep, [True] * 4 + [False] * 3)
        self.assertEqual(stats["removed_points"], 3)
        self.assertLess(stats["input_voxels"], len(points))
        no_filter, _ = sparse_obstacles(
            points, np.ones(7, bool), ContinuityOptions(max_isolated_points=0)
        )
        self.assertTrue(no_filter.all())

    def test_connected_four_point_obstacle_is_not_deleted_as_sparse(self):
        points = np.array([[x, 0, 0] for x in (0.01, 0.11, 0.21, 0.31)])
        keep, _ = sparse_obstacles(points, np.ones(4, bool))
        self.assertTrue(keep.all())

    def test_instance_retention_distinguishes_few_points_from_entire_target(self):
        points = np.array([[x, 0, 0] for x in (0.01, 0.21, 0.41, 5.01, 5.21, 5.41)])
        identity = np.array([1, 1, 1, 2, 2, 2])
        retained = np.array([True, False, False, False, False, False])
        report = instance_retention(
            points, identity, np.ones(6, bool), np.ones(6, bool), retained, 0.1
        )
        self.assertEqual(report["baseline_supported_instances"], 2)
        self.assertEqual(report["lost_instance_ids"], [2])

    def test_retention_keeps_sparse_and_partial_loss_in_denominator(self):
        points = np.array([[x, 0, 0] for x in (0.01, 1.01, 1.21, 1.41)])
        report = instance_retention(
            points,
            np.array([1, 2, 2, 2]),
            np.ones(4, bool),
            np.ones(4, bool),
            np.array([False, True, False, False]),
            0.1,
        )
        self.assertEqual(report["all_baseline_instances"], 2)
        self.assertEqual(report["all_completely_lost_instance_ids"], [1])
        self.assertEqual(report["partially_lost_instance_ids"], [2])
        self.assertEqual(report["baseline_obstacle_voxels"], 4)
        self.assertEqual(report["retained_baseline_obstacle_voxels"], 1)
        self.assertEqual(report["instances"][0]["baseline_voxels"], 1)


class PromotionTest(unittest.TestCase):
    def test_sparse_instance_loss_cannot_hide_behind_legacy_cohort(self):
        totals = {"roi": {"baseline": {"tp": 100}, "candidate": {"tp": 300}}}
        row = {
            "labeled": True,
            "obstacle_instances_raw": {
                "newly_completely_lost_instances": 0,
                "all_completely_lost_instances": 1,
            },
            "obstacle_instances": {
                "newly_completely_lost_instances": 0,
                "all_completely_lost_instances": 1,
            },
            "continuity": {
                "baseline": {"ego_connected_cells": 10, "seed_observed": True},
                "candidate": {"ego_connected_cells": 30, "seed_observed": True},
            },
        }
        self.assertFalse(
            promotion_checks([row], totals)[
                "eligible_for_native_integration_experiment"
            ]
        )

    def test_point_errors_do_not_block_but_erased_instance_does(self):
        totals = {"roi": {"baseline": {"tp": 100}, "candidate": {"tp": 300}}}
        clean = {
            "labeled": True,
            "new_thing_ground_errors": 2,
            "obstacle_instances_raw": {
                "newly_completely_lost_instances": 0,
                "all_completely_lost_instances": 0,
            },
            "obstacle_instances": {
                "newly_completely_lost_instances": 0,
                "all_completely_lost_instances": 0,
            },
            "continuity": {
                "baseline": {"ego_connected_cells": 10, "seed_observed": True},
                "candidate": {"ego_connected_cells": 30, "seed_observed": True},
            },
        }
        lost = {
            **clean,
            "obstacle_instances_raw": {
                "newly_completely_lost_instances": 1,
                "all_completely_lost_instances": 1,
            },
        }
        result = promotion_checks([clean, lost], totals)
        self.assertTrue(result["ego_connected_ground_increased"])
        self.assertFalse(result["eligible_for_native_integration_experiment"])
        self.assertTrue(
            promotion_checks([clean], totals)[
                "eligible_for_native_integration_experiment"
            ]
        )
        self.assertFalse(promotion_checks([clean], totals)["production_qualified"])
        regressed = {
            **clean,
            "continuity": {
                "baseline": {"ego_connected_cells": 10, "seed_observed": True},
                "candidate": {"ego_connected_cells": 1, "seed_observed": True},
            },
        }
        result = promotion_checks([clean, regressed], totals)
        self.assertTrue(result["ego_connected_ground_increased"])
        self.assertEqual(result["frames_with_connected_area_regression"], 1)
        self.assertFalse(result["eligible_for_native_integration_experiment"])

    def test_unlabeled_or_no_connectivity_gain_is_not_acceptance(self):
        result = promotion_checks([{"labeled": False}], {})
        self.assertFalse(result["eligible_for_native_integration_experiment"])
        row = {
            "labeled": True,
            "obstacle_instances_raw": {"newly_completely_lost_instances": 0},
            "obstacle_instances": {"newly_completely_lost_instances": 0},
            "continuity": {
                "baseline": {"ego_connected_cells": 10, "seed_observed": True},
                "candidate": {"ego_connected_cells": 10, "seed_observed": True},
            },
        }
        totals = {"roi": {"baseline": {"tp": 100}, "candidate": {"tp": 100}}}
        self.assertFalse(
            promotion_checks([row], totals)[
                "eligible_for_native_integration_experiment"
            ]
        )


if __name__ == "__main__":
    unittest.main()
