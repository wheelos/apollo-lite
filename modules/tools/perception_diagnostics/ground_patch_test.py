# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Analytic regressions for the independent local-patch ground comparator."""

import unittest

import numpy as np
from ground_patch import GroundPatchConfig, estimate_ground_patch
from spatial_support.observed_terrain import observed_terrain_metrics


def small_config(**overrides):
    values = {
        "x_min": 0.0,
        "x_max": 2.0,
        "y_min": 0.0,
        "y_max": 2.0,
        "patch_size": 1.0,
        "expected_ground_z": 0.4,
    }
    values.update(overrides)
    return GroundPatchConfig(**values)


class GroundPatchTest(unittest.TestCase):
    def test_tilted_plane_obstacle_and_original_order_are_preserved(self):
        rows = []
        for x, y in (
            (0.2, 0.2),
            (0.8, 0.2),
            (0.2, 0.8),
            (0.8, 0.8),
        ):
            for ix in range(2):
                for iy in range(2):
                    px, py = ix + x, iy + y
                    rows.append([px, py, 0.1 * px + 0.2 * py + 0.4, 17.0])
        rows.append([0.5, 0.5, 1.8, 99.0])
        points = np.asarray(rows)
        estimate = estimate_ground_patch(
            points, config=small_config(expected_ground_z=0.65)
        )

        self.assertTrue(estimate.ground_mask[:-1].all())
        self.assertFalse(estimate.ground_mask[-1])
        self.assertTrue(estimate.support_mask.all())
        self.assertFalse(estimate.seed_mask[-1])
        self.assertEqual(estimate.ground_z.shape, (len(points),))
        np.testing.assert_allclose(
            estimate.plane_coefficients[:, :, 0], 0.1, atol=1e-12
        )
        np.testing.assert_allclose(
            estimate.plane_coefficients[:, :, 1], 0.2, atol=1e-12
        )
        np.testing.assert_allclose(
            estimate.uprightness, 1 / np.sqrt(1 + 0.1**2 + 0.2**2), atol=1e-12
        )
        self.assertEqual(estimate.cell_id[-1], 0)
        self.assertEqual(estimate.seed_count.shape, (2, 2))
        self.assertEqual(set(estimate.patch_status.flat), {"supported"})
        terrain = observed_terrain_metrics(points, estimate)
        self.assertEqual(terrain.raw_return_count[0, 0], 5)
        self.assertEqual(terrain.ground_candidate_count[0, 0], 4)
        self.assertGreater(terrain.all_return_plane_distance_std[0, 0], 0)
        self.assertAlmostEqual(terrain.ground_candidate_plane_distance_std[0, 0], 0)
        self.assertEqual(terrain.status.dtype.kind, "U")
        self.assertEqual(estimate.patch_status.dtype.kind, "U")

    def test_rank_deficient_and_missing_patches_stay_unsupported_nan(self):
        config = small_config()
        points = np.array(
            [
                [0.2, 0.5, 0.4],
                [0.4, 0.5, 0.4],
                [0.6, 0.5, 0.4],
                [0.8, 0.5, 0.4],
                [np.nan, 0.2, 0.4],
                [2.0, 0.5, 0.4],
            ]
        )
        estimate = estimate_ground_patch(points, config=config)

        self.assertFalse(estimate.patch_supported.any())
        self.assertEqual(estimate.patch_status[0, 0], "unsupported_rank")
        self.assertEqual(estimate.patch_status[1, 1], "unsupported_empty")
        self.assertTrue(np.isnan(estimate.ground_z).all())
        self.assertFalse(estimate.ground_mask.any())
        self.assertEqual(estimate.cell_id[-2], -1)
        self.assertEqual(estimate.cell_id[-1], -1)

        terrain = observed_terrain_metrics(points, estimate)
        self.assertTrue(np.isnan(terrain.slope_gradient).all())
        self.assertTrue(np.isnan(terrain.all_return_plane_distance_std).all())
        self.assertTrue(np.isnan(terrain.ground_candidate_plane_distance_std).all())
        self.assertFalse(terrain.plane_supported.any())
        self.assertFalse(terrain.availability[1, 1])

    def test_sparse_no_height_and_missing_neighbor_remain_unknown(self):
        points = np.array(
            [
                [0.2, 0.2, 0.4],
                [0.8, 0.2, 0.4],
                [0.2, 0.8, 0.4],
                [0.8, 0.8, 0.4],
                [1.2, 0.3, 0.4],
                [1.8, 0.7, 0.4],
                [2.5, 0.5, 1.0],
            ]
        )
        config = GroundPatchConfig(
            x_min=0,
            x_max=3,
            y_min=0,
            y_max=1,
            patch_size=1,
            expected_ground_z=0.4,
        )
        estimate = estimate_ground_patch(points, config=config)
        metrics = observed_terrain_metrics(points, estimate)

        self.assertTrue(estimate.patch_supported[0, 0])
        self.assertEqual(estimate.patch_status[1, 0], "unsupported_insufficient_seeds")
        self.assertEqual(estimate.patch_status[2, 0], "unsupported_no_height_seeds")
        self.assertTrue(np.isnan(metrics.step[0, 0]))
        self.assertTrue(np.isnan(metrics.step[1, 0]))
        self.assertEqual(metrics.raw_return_count[2, 0], 1)
        self.assertTrue(np.isnan(metrics.all_return_height_std[2, 0]))
        self.assertTrue(np.isnan(metrics.all_return_plane_distance_std[2, 0]))

    def test_over_slope_patch_is_rejected_by_gradient_contract(self):
        points = np.array(
            [
                [0.2, 0.2, 0.16],
                [0.8, 0.2, 0.40],
                [0.2, 0.8, 0.40],
                [0.8, 0.8, 0.64],
            ]
        )
        config = GroundPatchConfig(
            x_min=0,
            x_max=1,
            y_min=0,
            y_max=1,
            patch_size=1,
            expected_ground_z=0.6,
            ground_seed_band=0.5,
        )
        estimate = estimate_ground_patch(points, config=config)

        self.assertFalse(estimate.patch_supported[0, 0])
        self.assertEqual(estimate.patch_status[0, 0], "unsupported_uprightness")
        self.assertTrue(np.isnan(estimate.ground_z).all())

    def test_raw_height_spread_survives_flat_fitted_plane(self):
        points = np.array(
            [
                [0.2, 0.2, -0.09],
                [0.8, 0.2, -0.09],
                [0.2, 0.8, -0.09],
                [0.8, 0.8, 0.09],
            ]
        )
        config = GroundPatchConfig(
            x_min=0,
            x_max=1,
            y_min=0,
            y_max=1,
            patch_size=1,
            expected_ground_z=-0.09,
        )
        estimate = estimate_ground_patch(points, config=config)
        terrain = observed_terrain_metrics(points, estimate)

        self.assertTrue(estimate.patch_supported[0, 0])
        np.testing.assert_allclose(estimate.plane_coefficients[0, 0, :2], 0, atol=1e-12)
        self.assertAlmostEqual(terrain.slope_gradient[0, 0], 0)
        self.assertGreater(terrain.all_return_height_std[0, 0], 0)
        self.assertGreater(terrain.all_return_plane_distance_std[0, 0], 0)
        self.assertAlmostEqual(terrain.all_return_height_std[0, 0], 0.09, places=12)
        self.assertEqual(terrain.raw_return_count[0, 0], 4)
        self.assertEqual(terrain.ground_candidate_count[0, 0], 3)

    def test_step_is_plane_disagreement_at_shared_boundary(self):
        rows = []
        for ix in range(2):
            step_offset = 0.0
            for x, y in ((0.2, 0.2), (0.8, 0.2), (0.2, 0.8), (0.8, 0.8)):
                px = ix + x
                rows.append([px, y, 0.1 * px + 0.4 + step_offset])
        points = np.asarray(rows)
        config = GroundPatchConfig(
            x_min=0,
            x_max=2,
            y_min=0,
            y_max=1,
            patch_size=1,
            expected_ground_z=0.5,
            ground_seed_band=0.5,
        )
        estimate = estimate_ground_patch(points, config=config)
        metrics = observed_terrain_metrics(points, estimate)

        self.assertTrue(estimate.patch_supported.all())
        self.assertAlmostEqual(metrics.step[0, 0], 0.0, places=12)
        self.assertAlmostEqual(metrics.step[1, 0], 0.0, places=12)
        points[4:, 2] += 0.2
        stepped = estimate_ground_patch(points, config=config)
        metrics = observed_terrain_metrics(points, stepped)
        self.assertAlmostEqual(metrics.step[0, 0], 0.2, places=12)
        self.assertAlmostEqual(metrics.step[1, 0], 0.2, places=12)

    def test_ground_labels_use_only_original_raw_residual_and_fit_is_seed_only(self):
        points = np.array(
            [
                [0.2, 0.2, 0.4],
                [0.8, 0.2, 0.4],
                [0.2, 0.8, 0.4],
                [0.8, 0.8, 0.4],
                [0.5, 0.5, 0.8],
            ]
        )
        estimate = estimate_ground_patch(points, config=small_config())
        self.assertEqual(estimate.seed_mask[:4].sum(), 3)
        self.assertFalse(estimate.seed_mask[4])
        self.assertTrue(estimate.ground_mask[:4].all())
        self.assertFalse(estimate.ground_mask[4])
        self.assertAlmostEqual(estimate.ground_z[4], 0.4)


if __name__ == "__main__":
    unittest.main()
