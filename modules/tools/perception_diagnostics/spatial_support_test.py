# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Analytic terrain and footprint regressions independent of recorded outputs."""

import unittest

import numpy as np
from spatial_support.clearance import distances
from spatial_support.cost import layers as cost_layers
from spatial_support.passage import query as passage_query
from spatial_support.passage import touched
from spatial_support.terrain import INFERRED, MEASURED, UNKNOWN_RISK, build_scene
from spatial_support.trajectory import Vehicle, query, sample_path
from spatial_support_experiment import blocker_statistics, layer_blockers, path_blockers


def flat_scene(size=12, resolution=1.0):
    points = np.array(
        [
            [(x + 0.5) * resolution, (y + 0.5) * resolution, 0]
            for x in range(size)
            for y in range(size)
        ]
    )
    return build_scene(
        points,
        points[:, 2],
        np.ones(len(points), bool),
        resolution,
        np.array([0.0, 0.0]),
        np.zeros((size, size), np.uint8),
        np.ones((size, size), bool),
        0.3,
        0.3,
        0.08,
        np.ones((size, size), bool),
    )


class TerrainTest(unittest.TestCase):
    def test_analytic_two_dimensional_ramp(self):
        points = np.array(
            [[x + 0.5, y + 0.5, 0.1 * x + 0.2 * y] for x in range(5) for y in range(5)]
        )
        layers = np.zeros((5, 5), np.uint8)
        before = points.copy()
        scene = build_scene(
            points,
            points[:, 2],
            np.ones(25, bool),
            1,
            np.zeros(2),
            layers,
            np.zeros((5, 5), bool),
            0.3,
            0.3,
            0.08,
        )
        np.testing.assert_allclose(scene.slope, np.hypot(0.1, 0.2), atol=1e-12)
        np.testing.assert_allclose(scene.roughness, 0, atol=1e-12)
        self.assertAlmostEqual(scene.step[2, 2], 0.2)
        self.assertFalse(scene.terrain_limit.any())
        np.testing.assert_array_equal(points, before)
        np.testing.assert_array_equal(layers, 0)

    def test_step_rejects_local_terrain(self):
        scene = flat_scene()
        points = np.array(
            [[x + 0.5, y + 0.5, float(x >= 6)] for x in range(12) for y in range(12)]
        )
        stepped = build_scene(
            points,
            points[:, 2],
            np.ones(144, bool),
            1,
            np.zeros(2),
            scene.layers,
            scene.verified_free,
            0.3,
            0.3,
            0.08,
        )
        self.assertEqual(stepped.step[5, 5], 1.0)
        self.assertTrue(stepped.terrain_limit[5, 5])
        self.assertFalse(stepped.terrain_limit[2, 2])

    def test_no_plane_from_collinear_support_or_missing_ground(self):
        points = np.array([[x + 0.5, 0.5, 0] for x in range(4)])
        scene = build_scene(
            points,
            points[:, 2],
            np.ones(4, bool),
            1,
            np.zeros(2),
            np.zeros((5, 5), np.uint8),
            np.zeros((5, 5), bool),
            0.3,
            0.3,
            0.08,
        )
        self.assertTrue(np.isnan(scene.slope).all())
        self.assertTrue(np.isnan(scene.ground_z[:, 1:]).all())

    def test_invalid_grid_and_thresholds_rejected(self):
        for resolution, step in ((0, 0.2), (1, -1), (1, np.nan)):
            with self.assertRaises(ValueError):
                build_scene(
                    np.empty((0, 3)),
                    np.empty(0),
                    np.empty(0, bool),
                    resolution,
                    np.zeros(2),
                    np.zeros((2, 2), np.uint8),
                    np.zeros((2, 2), bool),
                    0.3,
                    step,
                    0.08,
                )


class ClearanceTest(unittest.TestCase):
    def test_cell_rectangle_not_center_distance_and_independent_layers(self):
        scene = flat_scene()
        scene.layers[5, 5] = MEASURED
        scene.layers[8, 5] = INFERRED | UNKNOWN_RISK
        result = distances(scene, np.array([[2.5, 5.5], [5.0, 5.5]]))
        np.testing.assert_allclose(result["measured"], [2.5, 0])
        np.testing.assert_allclose(result["inferred"], [5.5, 3])
        np.testing.assert_allclose(result["unknown_risk"], [5.5, 3])

    def test_empty_layer_has_no_barrier_not_free_evidence(self):
        scene = flat_scene()
        scene.verified_free[:] = False
        result = distances(scene, np.array([[6.0, 6.0]]))
        self.assertTrue(np.isinf(result["measured"][0]))
        risk = query(scene, [[6, 6]], Vehicle(1, 0.5, 0))
        self.assertEqual(risk["samples"][0]["state"], "UNKNOWN")
        self.assertIn("unverified_free", risk["samples"][0]["reasons"])
        self.assertFalse(risk["collision_free_authorized"])

    def test_invalid_query_rejected(self):
        with self.assertRaises(ValueError):
            distances(flat_scene(), np.array([[np.nan, 0]]))


class TrajectoryTest(unittest.TestCase):
    def test_supported_synthetic_path_is_never_production_authorization(self):
        scene = flat_scene()
        before = scene.layers.copy()
        result = query(scene, [[3, 6], [8, 6]], Vehicle(1, 0.5, 0.1))
        self.assertEqual(set(result["state_counts"]), {"SUPPORTED_ONLY"})
        self.assertIsNone(result["first_risk_arc_m"])
        self.assertFalse(result["production_qualified"])
        np.testing.assert_array_equal(scene.layers, before)

    def test_unknown_terrain_hole_and_boundary_are_not_safe(self):
        scene = flat_scene()
        scene.ground_z[5, 5] = np.nan
        result = query(scene, [[5.5, 5.5]], Vehicle(1, 0.5, 0))
        self.assertEqual(result["samples"][0]["state"], "UNKNOWN")
        self.assertIn("missing_ground", result["samples"][0]["reasons"])
        result = query(scene, [[0.2, 6]], Vehicle(1, 0.5, 0))
        self.assertIn("roi_boundary", result["samples"][0]["reasons"])
        self.assertEqual(result["samples"][0]["state"], "UNKNOWN")

    def test_observed_but_disconnected_ground_is_unknown(self):
        scene = flat_scene()
        scene.ego_connected[5, 5] = False
        result = query(scene, [[5.5, 5.5]], Vehicle(1, 0.5, 0))
        self.assertEqual(result["samples"][0]["state"], "UNKNOWN")
        self.assertIn("disconnected_ground", result["samples"][0]["reasons"])

    def test_terrain_limit_and_unknown_object_risk_are_separate(self):
        scene = flat_scene()
        scene.terrain_limit[5, 5] = True
        result = query(scene, [[5.5, 5.5]], Vehicle(1, 0.5, 0))
        self.assertEqual(result["samples"][0]["state"], "TERRAIN_LIMIT")
        scene.layers[5, 5] = UNKNOWN_RISK
        result = query(scene, [[5.5, 5.5]], Vehicle(1, 0.5, 0))
        self.assertEqual(result["samples"][0]["state"], "OBSTACLE_RISK")
        self.assertIn("unknown_risk", result["samples"][0]["reasons"])

    def test_obstacle_between_waypoints_not_skipped(self):
        scene = flat_scene()
        scene.layers[5, 5] = MEASURED
        result = query(scene, [[2, 5.5], [9, 5.5]], Vehicle(0.1, 0.1, 0))
        self.assertGreater(result["state_counts"]["OBSTACLE_RISK"], 0)
        self.assertLessEqual(result["first_risk_arc_m"], 3)
        self.assertGreater(result["first_risk_arc_m"], 0)
        self.assertGreater(
            result["swept_query_radius_m"], result["physical_footprint_radius_m"]
        )

    def test_larger_vehicle_cannot_remove_risk(self):
        scene = flat_scene()
        scene.layers[5, 5] = MEASURED
        small = query(scene, [[2, 3], [9, 3]], Vehicle(0.5, 0.5, 0))
        large = query(scene, [[2, 3], [9, 3]], Vehicle(4, 2, 0.2))
        self.assertEqual(set(small["state_counts"]), {"SUPPORTED_ONLY"})
        self.assertIn("OBSTACLE_RISK", large["state_counts"])

    def test_polyline_arc_duplicate_points_and_invalid_inputs(self):
        xy, arc, gap = sample_path([[1, 1], [1, 1], [4, 1]], 0.4)
        self.assertAlmostEqual(arc[-1], 3)
        self.assertLessEqual(gap, 0.4)
        np.testing.assert_array_equal(xy[-1], [4, 1])
        for path in ([], [[np.nan, 0]], [[1, 2, 3]]):
            with self.assertRaises(ValueError):
                sample_path(path, 0.1)
        with self.assertRaisesRegex(ValueError, "budget"):
            sample_path([[0, 0], [100, 0]], 0.001)
        for vehicle in (Vehicle(0, 1, 0), Vehicle(1, 1, -1)):
            with self.assertRaises(ValueError):
                query(flat_scene(), [[6, 6]], vehicle)


class P2Test(unittest.TestCase):
    def test_blocker_diagnostics_count_exclusive_and_overlapping_reasons(self):
        counts = blocker_statistics(
            {
                "measured": [True, True, False, False],
                "visibility": [False, True, True, False],
            }
        )
        self.assertEqual(counts["blocked_union"], 3)
        self.assertEqual(counts["multiple_blockers"], 1)
        self.assertEqual(counts["no_listed_blocker"], 1)
        self.assertEqual(counts["exclusive"], {"measured": 1, "visibility": 1})
        self.assertEqual(counts["pairwise_intersections"]["measured|visibility"], 1)
        scene = flat_scene()
        scene.layers[5, 5] = MEASURED | INFERRED | UNKNOWN_RISK
        scene.ground_z[5, 5] = np.nan
        scene.ego_connected[5, 5] = False
        scene.verified_free[5, 5] = False
        scene.terrain_limit[5, 5] = True
        before = scene.layers.copy()
        cells = layer_blockers(scene)
        self.assertEqual(cells["blocked_union"], 1)
        self.assertEqual(
            cells["exact_intersections"],
            {"measured|detection|unknown|terrain|missing|disconnected|visibility": 1},
        )
        result = passage_query(scene, [[5.5, 5.5]], Vehicle(0.1, 0.1, 0), 0)
        samples = path_blockers(result)
        self.assertEqual(samples["blocked_union"], 1)
        self.assertEqual(samples["multiple_blockers"], 1)
        self.assertEqual(samples["exclusive"]["measured"], 0)
        self.assertFalse(result["collision_free_authorized"])
        np.testing.assert_array_equal(scene.layers, before)

    def test_cost_limits_recompute_admissibility_without_mutating_scene(self):
        scene = flat_scene()
        scene.slope[2, 2] = 0.2
        scene.step[3, 3] = 0.2
        scene.roughness[4, 4] = 0.06
        scene.slope[5, 5] = np.nan
        scene.verified_free[6, 6] = False
        costs = cost_layers(scene, 0.1, 0.1, 0.04)
        for cell, ratio in (
            ((2, 2), "slope_ratio"),
            ((3, 3), "step_ratio"),
            ((4, 4), "roughness_ratio"),
        ):
            self.assertGreater(costs[ratio][cell], 1)
            self.assertTrue(costs["terrain_limit"][cell])
            self.assertFalse(costs["admissible_diagnostic"][cell])
        self.assertTrue(costs["unsupported"][5, 5])
        self.assertTrue(np.isnan(costs["slope_ratio"][5, 5]))
        self.assertFalse(costs["admissible_diagnostic"][6, 6])
        self.assertFalse(scene.terrain_limit.any())
        scene.terrain_limit[2, 2] = True
        relaxed = cost_layers(scene, 0.3, 0.3, 0.08)
        self.assertFalse(relaxed["terrain_limit"][2, 2])
        self.assertTrue(relaxed["admissible_diagnostic"][2, 2])

    def test_invalid_cost_normalizations_rejected(self):
        for value in (0, -1, np.nan, np.inf):
            for index in range(3):
                limits = [0.3, 0.2, 0.08]
                limits[index] = value
                with self.subTest(index=index, value=value):
                    with self.assertRaises(ValueError):
                        cost_layers(flat_scene(), *limits)

    def test_equivalent_directional_paths_do_not_introduce_corner_risk(self):
        scene = flat_scene(size=30, resolution=0.25)
        scene.layers[:, 8] = MEASURED
        scene.layers[:, 20] = MEASURED
        vehicle = Vehicle(4, 1, 0.1)
        paths = [
            [[2.5, 3.5], [4.5, 3.5]],
            [[2.5, 3.5], [3.5, 3.5], [4.5, 3.5]],
            [[2.5, 3.5], [2.5, 3.5], [3.37, 3.5], [4.5, 3.5], [4.5, 3.5]],
        ]
        for yaw in (None, 0, 2 * np.pi):
            for path in paths:
                with self.subTest(yaw=yaw, path=path):
                    result = passage_query(scene, path, vehicle, yaw)
                    self.assertEqual(set(result["state_counts"]), {"SUPPORTED_ONLY"})
                    self.assertFalse(
                        any(s["corner_rotation_envelope"] for s in result["samples"])
                    )
        for yaw in (None, 0):
            for path in paths:
                scene.layers[12, 13] = UNKNOWN_RISK
                result = passage_query(scene, path, vehicle, yaw)
                self.assertIn("OBSTACLE_RISK", result["state_counts"])
                self.assertTrue(
                    any("unknown_risk" in s["reasons"] for s in result["samples"])
                )
                self.assertFalse(result["collision_free_authorized"])

    def test_duplicate_poses_and_heading_changes_keep_directional_contract(self):
        scene = flat_scene()
        vehicle = Vehicle(2, 1, 0)
        for path in ([[6, 6]], [[6, 6], [6, 6], [6, 6]]):
            with self.assertRaises(ValueError):
                passage_query(scene, path, vehicle)
            result = passage_query(scene, path, vehicle, np.pi / 2)
            self.assertEqual(set(result["state_counts"]), {"SUPPORTED_ONLY"})
        for path in (
            [[4, 4], [6, 4], [6, 4], [6, 6]],
            [[4, 4], [6, 4], [4, 4]],
            [[4, 4], [6, 6], [6, 6], [8, 8]],
        ):
            result = passage_query(scene, path, vehicle)
            corners = any(s["corner_rotation_envelope"] for s in result["samples"])
            self.assertEqual(corners, path[-1] != [8, 8])
            fixed = passage_query(scene, path, vehicle, 0)
            self.assertFalse(
                any(s["corner_rotation_envelope"] for s in fixed["samples"])
            )

    def test_cost_layers_keep_unknown_and_sources_separate(self):
        scene = flat_scene()
        scene.layers[5, 5] = MEASURED | UNKNOWN_RISK
        scene.verified_free[4, 4] = False
        scene.slope[3, 3] = np.nan
        costs = cost_layers(scene, 0.3, 0.2, 0.08)
        self.assertTrue(costs["measured_obstacle"][5, 5])
        self.assertTrue(costs["unknown_obstacle"][5, 5])
        self.assertFalse(costs["inferred_obstacle"][5, 5])
        self.assertFalse(costs["admissible_diagnostic"][5, 5])
        self.assertFalse(costs["admissible_diagnostic"][4, 4])
        self.assertTrue(costs["unsupported"][3, 3])
        self.assertTrue(np.isnan(costs["slope_ratio"][3, 3]))

    def test_directional_footprint_fits_passage_disk_rejects(self):
        scene = flat_scene(size=30, resolution=0.25)
        scene.layers[:, 8] = MEASURED
        scene.layers[:, 20] = MEASURED
        vehicle = Vehicle(4, 1, 0.1)
        path = [[2.5, 3.5], [4.5, 3.5]]
        disk = query(scene, path, vehicle)
        rectangle = passage_query(scene, path, vehicle)
        self.assertIn("OBSTACLE_RISK", disk["state_counts"])
        self.assertEqual(set(rectangle["state_counts"]), {"SUPPORTED_ONLY"})
        rotated = passage_query(scene, [[3.5, 3.5]], vehicle, np.pi / 2)
        self.assertIn("OBSTACLE_RISK", rotated["state_counts"])
        self.assertFalse(rectangle["collision_free_authorized"])

    def test_cell_contact_and_corner_envelope_are_conservative(self):
        scene = flat_scene()
        keys, _ = touched(scene, np.array([6.0, 6.0]), 0, 1, 1)
        self.assertIn((4, 5), set(map(tuple, keys)))
        result = passage_query(scene, [[4, 4], [6, 4], [6, 6]], Vehicle(2, 1, 0))
        self.assertTrue(any(s["corner_rotation_envelope"] for s in result["samples"]))
        scene.ego_connected[:] = False
        result = passage_query(scene, [[6, 6]], Vehicle(1, 1, 0), 0)
        self.assertIn("disconnected_ground", result["samples"][0]["reasons"])
        self.assertEqual(result["samples"][0]["state"], "UNKNOWN")
        with self.assertRaises(ValueError):
            passage_query(scene, [[6, 6]], Vehicle(1, 1, 0))


if __name__ == "__main__":
    unittest.main()
