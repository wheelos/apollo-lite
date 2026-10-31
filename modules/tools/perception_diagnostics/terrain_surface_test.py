# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Focused geometry regressions for observed local terrain surfaces."""

import unittest

import numpy as np
from terrain_surface import TerrainSurfaceConfig, estimate_terrain_surface


def small_config(**overrides):
    values = {
        "x_min": 0.0,
        "x_max": 2.0,
        "y_min": 0.0,
        "y_max": 2.0,
        "cell_size": 0.5,
        "expected_ground_z": 0.0,
        "seed_height_band": 0.2,
        "min_anchor_points": 1,
    }
    values.update(overrides)
    return TerrainSurfaceConfig(**values)


def grid_points(height_fn, *, x_count=4, y_count=4):
    rows = []
    for ix in range(x_count):
        for iy in range(y_count):
            x = 0.25 + ix * 0.5
            y = 0.25 + iy * 0.5
            rows.append([x, y, height_fn(ix, iy, x, y)])
    return np.asarray(rows, dtype=float)


class TerrainSurfaceTest(unittest.TestCase):
    def test_ramp_is_continuous_and_step_is_reported_without_layer_jump(self):
        ramp = grid_points(lambda _ix, _iy, x, y: 0.2 * x + 0.1 * y)
        ramp_estimate = estimate_terrain_surface(
            ramp,
            root_anchor_id=0,
            config=small_config(expected_ground_z=0.075),
        )
        self.assertTrue((ramp_estimate.cell_status == "supported").all())
        self.assertTrue(ramp_estimate.point_ground_mask.all())
        self.assertAlmostEqual(
            ramp_estimate.cell_slope_gradient[1, 1],
            np.hypot(0.2, 0.1),
            places=10,
        )

        stepped = ramp.copy()
        stepped[8:, 2] += 0.5
        step_estimate = estimate_terrain_surface(
            stepped,
            root_anchor_id=0,
            config=small_config(expected_ground_z=0.075),
        )
        self.assertTrue(step_estimate.cell_status[0, :].tolist() == ["supported"] * 4)
        self.assertTrue(step_estimate.cell_status[3, :].tolist() == ["unknown_gap"] * 4)
        self.assertAlmostEqual(step_estimate.cell_step_m[1, 0], 0.5, places=12)
        self.assertFalse(step_estimate.point_ground_mask[8:].any())

    def test_raw_depression_depth_and_observed_extent_survive_surface_selection(self):
        points = grid_points(lambda ix, iy, _x, _y: -0.5 if (ix, iy) == (1, 1) else 0.0)
        points = np.vstack((points, points[5], points[5]))
        estimate = estimate_terrain_surface(
            points, root_anchor_id=0, config=small_config()
        )
        self.assertEqual(estimate.cell_status[1, 1], "unknown_gap")
        self.assertEqual(estimate.pit_status[1, 1], "measured")
        self.assertAlmostEqual(estimate.pit_depth_m[1, 1], 0.5, places=12)
        self.assertEqual(estimate.pit_width_x_m[1, 1], 0.5)
        self.assertEqual(estimate.pit_width_y_m[1, 1], 0.5)
        self.assertEqual(estimate.cell_observed_layers[1, 1][0].elevation_z, -0.5)
        self.assertTrue(np.isnan(estimate.cell_elevation_z[1, 1]))

        missing_floor = np.delete(points, [5, 16, 17], axis=0)
        missing = estimate_terrain_surface(
            missing_floor,
            root_anchor_id=0,
            point_ids=np.delete(np.arange(len(points)), [5, 16, 17]),
            config=small_config(),
        )
        self.assertEqual(missing.cell_status[1, 1], "unknown_empty")
        self.assertEqual(missing.pit_status[1, 1], "unknown_no_floor")
        self.assertTrue(np.isnan(missing.pit_depth_m[1, 1]))

    def test_same_cell_vertical_layer_span_is_reported_without_layer_merging(self):
        points = grid_points(lambda _ix, _iy, _x, _y: 0.0)
        points = np.vstack((points, [0.75, 0.75, 0.3]))
        estimate = estimate_terrain_surface(
            points, root_anchor_id=0, config=small_config()
        )
        self.assertAlmostEqual(estimate.cell_observed_layer_span_m[1, 1], 0.3)
        self.assertAlmostEqual(estimate.cell_step_m[1, 1], 0.3)
        self.assertEqual(
            estimate.cell_step_status[1, 1], "observed_within_cell_layer_span"
        )
        self.assertEqual(len(estimate.cell_observed_layers[1, 1]), 2)
        self.assertFalse(estimate.point_ground_mask[-1])

    def test_selected_upper_surface_with_enclosed_lower_layer_is_pit_unknown(self):
        points = grid_points(lambda _ix, _iy, _x, _y: 0.0)
        lower_cells = {(1, 1), (1, 2), (2, 1), (2, 2)}
        lower_rows = [
            [0.25 + ix * 0.5, 0.25 + iy * 0.5, -0.5]
            for ix, iy in sorted(lower_cells)
            for _ in range(3)
        ]
        points = np.vstack((points, lower_rows))
        estimate = estimate_terrain_surface(
            points, root_anchor_id=0, config=small_config()
        )

        self.assertEqual(estimate.root_anchor_id, 0)
        self.assertEqual(estimate.overall_status, "supported")
        self.assertTrue(estimate.point_ground_mask[:16].all())
        self.assertFalse(estimate.point_ground_mask[16:].any())
        for cell in lower_cells:
            self.assertEqual(estimate.cell_status[cell], "supported")
            self.assertEqual(len(estimate.cell_observed_layers[cell]), 2)
            self.assertEqual(estimate.pit_status[cell], "unknown_layer_ambiguity")
            self.assertAlmostEqual(estimate.cell_elevation_z[cell], 0.0)
            self.assertTrue(np.isnan(estimate.pit_depth_m[cell]))

    def test_pit_width_uses_connected_observed_extent_and_sparse_floor_stays_unknown(
        self,
    ):
        points = []
        for ix in range(16):
            for iy in range(16):
                if 2 <= ix < 4 and 2 <= iy < 4:
                    z = -0.3
                elif 8 <= ix < 14 and 8 <= iy < 14:
                    z = -0.5
                else:
                    z = 0.0
                x, y = 0.25 + ix * 0.5, 0.25 + iy * 0.5
                points.extend([[x, y, z]] * 3)
        points = np.asarray(points, dtype=float)
        config = TerrainSurfaceConfig(
            x_min=0,
            x_max=8,
            y_min=0,
            y_max=8,
            expected_ground_z=None,
        )
        estimate = estimate_terrain_surface(points, root_anchor_id=0, config=config)
        narrow_pit = np.s_[2:4, 2:4]
        wide_pit = np.s_[8:14, 8:14]
        self.assertTrue((estimate.pit_status[narrow_pit] == "measured").all())
        self.assertTrue((estimate.pit_status[wide_pit] == "measured").all())
        np.testing.assert_allclose(estimate.pit_width_x_m[narrow_pit], 1.0)
        np.testing.assert_allclose(estimate.pit_width_y_m[narrow_pit], 1.0)
        np.testing.assert_allclose(estimate.pit_depth_m[narrow_pit], 0.3)
        np.testing.assert_allclose(estimate.pit_width_x_m[wide_pit], 3.0)
        np.testing.assert_allclose(estimate.pit_width_y_m[wide_pit], 3.0)
        np.testing.assert_allclose(estimate.pit_depth_m[wide_pit], 0.5)

        one_return_floor = grid_points(
            lambda ix, iy, _x, _y: -0.5 if (ix, iy) == (1, 1) else 0.0
        )
        sparse = estimate_terrain_surface(
            one_return_floor, root_anchor_id=0, config=small_config()
        )
        self.assertEqual(sparse.pit_status[1, 1], "unknown_insufficient_floor")
        self.assertTrue(np.isnan(sparse.pit_depth_m[1, 1]))

    def test_partial_rim_does_not_claim_a_pit_measurement(self):
        rows = []
        for ix in range(2):
            for iy in range(4):
                rows.append([0.25 + ix * 0.5, 0.25 + iy * 0.5, 0.0])
        rows.append([1.25, 0.75, -0.5])
        config = small_config()
        estimate = estimate_terrain_surface(
            np.asarray(rows), root_anchor_id=0, config=config
        )
        self.assertEqual(estimate.pit_status[2, 1], "unknown_no_rim")
        self.assertTrue(np.isnan(estimate.pit_depth_m[2, 1]))

        three_quadrant_rows = []
        supported_cells = {
            (0, 0),
            (0, 1),
            (0, 2),
            (1, 2),
            (2, 2),
        }
        for ix, iy in sorted(supported_cells):
            three_quadrant_rows.append([0.25 + ix * 0.5, 0.25 + iy * 0.5, 0.0])
        three_quadrant_rows.append([0.75, 0.75, -0.5])
        three_quadrant = estimate_terrain_surface(
            np.asarray(three_quadrant_rows),
            root_anchor_id=0,
            config=small_config(),
        )
        self.assertEqual(three_quadrant.pit_status[1, 1], "unknown_no_rim")
        self.assertTrue(np.isnan(three_quadrant.pit_depth_m[1, 1]))

    def test_bridge_layers_do_not_merge_and_explicit_anchor_is_z_translation_invariant(
        self,
    ):
        base = grid_points(lambda _ix, _iy, _x, _y: 0.0)
        xy = base[:, :2]
        stacked = np.vstack(
            (
                np.column_stack((xy, np.zeros(len(xy)))),
                np.column_stack((xy, np.full(len(xy), 4.0))),
            )
        )
        ids = np.arange(len(stacked), dtype=np.int64) + 100
        estimate = estimate_terrain_surface(
            stacked, ids, root_anchor_id=100, config=small_config()
        )
        self.assertTrue(estimate.point_ground_mask[:16].all())
        self.assertFalse(estimate.point_ground_mask[16:].any())
        self.assertEqual(len(estimate.cell_observed_layers[1, 1]), 2)
        self.assertEqual(
            estimate.cell_step_status[1, 1], "observed_within_cell_layer_span"
        )
        self.assertAlmostEqual(estimate.cell_observed_layer_span_m[1, 1], 4.0)

        shifted = stacked.copy()
        shifted[:, 2] += 7.0
        shifted_estimate = estimate_terrain_surface(
            shifted, ids, root_anchor_id=100, config=small_config()
        )
        np.testing.assert_array_equal(
            estimate.point_ground_mask, shifted_estimate.point_ground_mask
        )
        np.testing.assert_allclose(
            shifted_estimate.cell_elevation_z[estimate.cell_status == "supported"],
            estimate.cell_elevation_z[estimate.cell_status == "supported"] + 7.0,
        )

        auto_config = small_config(expected_ground_z=None)
        automatic = estimate_terrain_surface(stacked, ids, config=auto_config)
        shifted_automatic = estimate_terrain_surface(shifted, ids, config=auto_config)
        self.assertEqual(automatic.overall_status, "supported")
        self.assertEqual(shifted_automatic.overall_status, "supported")
        self.assertTrue(automatic.point_ground_mask.any())
        self.assertEqual(automatic.root_anchor_id, shifted_automatic.root_anchor_id)
        np.testing.assert_array_equal(
            automatic.point_ground_mask, shifted_automatic.point_ground_mask
        )

    def test_risk_mask_preserves_raw_layers_but_suppresses_ground_label(self):
        points = grid_points(lambda _ix, _iy, _x, _y: 0.0)
        points = np.vstack((points, [0.25, 0.25, 0.0]))
        risk = np.zeros(len(points), dtype=bool)
        risk[0] = True
        risk[-1] = True
        estimate = estimate_terrain_surface(
            points,
            point_ids=np.arange(len(points)) + 20,
            risk_mask=risk,
            root_anchor_id=20,
            config=small_config(),
        )
        self.assertEqual(estimate.root_anchor_id, 20)
        self.assertTrue(estimate.point_ground_candidate_mask[-1])
        self.assertFalse(estimate.point_ground_mask[-1])
        self.assertTrue(estimate.point_ground_candidate_mask[0])
        self.assertFalse(estimate.point_ground_mask[0])
        self.assertEqual(estimate.cell_raw_return_count[0, 0], 2)
        self.assertEqual(
            sum(layer.point_count for layer in estimate.cell_observed_layers[0, 0]),
            2,
        )

    def test_missing_anchor_and_rank_blind_slope_remain_explicitly_unknown(self):
        no_seed = np.array([[0.25, 0.25, 2.0]])
        unknown = estimate_terrain_surface(no_seed, config=small_config())
        self.assertEqual(unknown.overall_status, "unknown_no_anchor")
        self.assertEqual(unknown.cell_status[0, 0], "unknown_no_anchor")
        self.assertTrue(np.isnan(unknown.point_surface_z).all())
        self.assertFalse(unknown.point_ground_mask.any())

        line = np.array([[0.25 + i * 0.5, 0.25, 0.1 * i] for i in range(4)])
        rank_blind = estimate_terrain_surface(
            line, root_anchor_id=0, config=small_config()
        )
        self.assertEqual(rank_blind.overall_status, "unknown_no_anchor")
        self.assertFalse(rank_blind.point_ground_mask.any())
        self.assertTrue(np.isnan(rank_blind.cell_slope_gradient).all())

    def test_isolated_near_origin_return_cannot_seed_supported_surface(self):
        config = TerrainSurfaceConfig(
            x_min=-2.0,
            x_max=2.0,
            y_min=-2.0,
            y_max=2.0,
            expected_ground_z=None,
        )
        rows = []
        isolated_id = 0
        for ix in range(8):
            for iy in range(8):
                if (ix, iy) == (4, 4):
                    continue
                x, y = -1.75 + ix * 0.5, -1.75 + iy * 0.5
                rows.extend([[x, y, 0.0]] * 3)
        isolated_id = len(rows)
        rows.append([0.25, 0.25, 1.0])
        ids = np.arange(len(rows), dtype=np.int64)
        estimate = estimate_terrain_surface(
            np.asarray(rows), point_ids=ids, config=config
        )
        self.assertNotEqual(estimate.root_anchor_id, isolated_id)
        self.assertFalse(estimate.point_ground_mask[isolated_id])
        self.assertEqual(estimate.overall_status, "supported")

        one_return = np.array([[0.25, 0.25, 0.0]])
        insufficient = estimate_terrain_surface(
            one_return,
            root_anchor_id=0,
            config=TerrainSurfaceConfig(
                x_min=0, x_max=1, y_min=0, y_max=1, expected_ground_z=None
            ),
        )
        self.assertEqual(insufficient.overall_status, "unknown_no_anchor")
        self.assertEqual(insufficient.cell_status[0, 0], "unknown_no_anchor")

    def test_unobserved_column_does_not_get_filled_or_connect_far_cells(self):
        points = []
        for ix in (0, 1, 3):
            for iy in range(4):
                points.append([0.25 + ix * 0.5, 0.25 + iy * 0.5, 0.0])
        estimate = estimate_terrain_surface(
            np.asarray(points),
            root_anchor_id=0,
            config=small_config(),
        )
        self.assertTrue(
            estimate.cell_status[0:2, :].tolist() == [["supported"] * 4] * 2
        )
        self.assertTrue(estimate.cell_status[2, :].tolist() == ["unknown_empty"] * 4)
        self.assertTrue(estimate.cell_status[3, :].tolist() == ["unknown_gap"] * 4)
        self.assertTrue(np.isnan(estimate.cell_elevation_z[2, :]).all())
        self.assertTrue(np.isnan(estimate.cell_elevation_z[3, :]).all())


if __name__ == "__main__":
    unittest.main()
