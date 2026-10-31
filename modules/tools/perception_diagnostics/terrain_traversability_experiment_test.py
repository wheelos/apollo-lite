# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Synthetic tests for native risk-grid projection and limits contracts."""

import unittest
from types import SimpleNamespace

import numpy as np
from terrain_surface import TerrainSurfaceConfig
from terrain_traversability import MEASURED, OBJECT, UNKNOWN_RISK
from terrain_traversability_experiment import (
    _limits_from_payload,
    _summarize_frames,
    rasterize_native_risk,
)


class ProtoLike(SimpleNamespace):
    def HasField(self, name):
        return hasattr(self, name)


def native_cell(x, y, *, measured=False, object_risk=False, unknown=False):
    return ProtoLike(
        x_index=x,
        y_index=y,
        occupancy_obs=measured,
        occupancy_det=object_risk,
        occupancy_unknown=unknown,
    )


def native_map(origin_x, origin_y, resolution, width, height, cells):
    return ProtoLike(
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
        width=width,
        height=height,
        cell=cells,
    )


class TerrainTraversabilityExperimentTest(unittest.TestCase):
    def test_native_rectangles_use_positive_overlap_and_negative_origins(self):
        config = TerrainSurfaceConfig(
            x_min=-1.0,
            x_max=1.0,
            y_min=-1.0,
            y_max=1.0,
            cell_size=0.5,
        )
        grid = native_map(
            -0.5,
            -0.5,
            1.0,
            1,
            1,
            [native_cell(0, 0, measured=True)],
        )
        layers, observed, info = rasterize_native_risk(grid, config, audited=True)
        expected = np.zeros((4, 4), dtype=np.uint8)
        expected[1:3, 1:3] = MEASURED
        np.testing.assert_array_equal(layers, expected)
        np.testing.assert_array_equal(observed, expected != 0)
        self.assertEqual(info["fully_covered_terrain_cell_count"], 4)
        self.assertEqual(info["native_omitted_cell_count"], 0)
        self.assertEqual(layers[0, 0], 0)
        self.assertFalse(observed[0, 0])

    def test_sparse_omissions_are_unobserved_but_explicit_empty_cells_count(self):
        config = TerrainSurfaceConfig(
            x_min=-0.5,
            x_max=0.5,
            y_min=-0.25,
            y_max=0.25,
            cell_size=0.5,
        )
        grid = native_map(
            -0.5,
            -0.25,
            0.5,
            2,
            1,
            [native_cell(0, 0)],
        )
        layers, observed, info = rasterize_native_risk(grid, config, audited=True)
        np.testing.assert_array_equal(layers, np.zeros((2, 1), dtype=np.uint8))
        np.testing.assert_array_equal(observed[:, 0], [True, False])
        self.assertEqual(info["native_serialized_cell_count"], 1)
        self.assertEqual(info["native_omitted_cell_count"], 1)

    def test_partial_coverage_and_point_unknown_risk_are_preserved(self):
        config = TerrainSurfaceConfig(
            x_min=-0.75,
            x_max=0.75,
            y_min=-0.75,
            y_max=0.75,
            cell_size=0.5,
        )
        grid = native_map(
            -0.25,
            -0.25,
            0.75,
            1,
            1,
            [native_cell(0, 0, object_risk=True)],
        )
        layers, observed, info = rasterize_native_risk(
            grid,
            config,
            point_xyz=np.array([[0.5, 0.5, 0.0]]),
            point_risk_layers=np.array([UNKNOWN_RISK], dtype=np.uint8),
            audited=True,
        )
        expected = np.zeros((3, 3), dtype=np.uint8)
        expected[1, 1] = OBJECT
        expected[1, 2] = OBJECT
        expected[2, 1] = OBJECT
        expected[2, 2] = OBJECT | UNKNOWN_RISK
        np.testing.assert_array_equal(layers, expected)
        expected_observed = np.zeros((3, 3), dtype=bool)
        expected_observed[1, 1] = True
        np.testing.assert_array_equal(observed, expected_observed)
        self.assertEqual(info["fully_covered_terrain_cell_count"], 1)
        self.assertGreater(info["partially_covered_terrain_cell_count"], 0)

    def test_risk_projection_requires_full_native_audit(self):
        config = TerrainSurfaceConfig(
            x_min=0.0,
            x_max=0.5,
            y_min=0.0,
            y_max=0.5,
            cell_size=0.5,
        )
        grid = native_map(
            0.0,
            0.0,
            0.5,
            1,
            1,
            [native_cell(0, 0)],
        )
        with self.assertRaisesRegex(ValueError, "requires audited output"):
            rasterize_native_risk(grid, config)

    def test_limits_require_explicit_diagnostic_only_label_and_all_values(self):
        payload = {
            "diagnostic_only": True,
            "max_slope_gradient": 0.30,
            "max_roughness_m": 0.10,
            "max_step_m": 0.20,
            "max_depression_depth_m": 0.20,
        }
        limits = _limits_from_payload(payload)
        self.assertEqual(limits.max_slope_gradient, 0.30)
        payload["diagnostic_only"] = False
        with self.assertRaisesRegex(ValueError, "diagnostic_only"):
            _limits_from_payload(payload)
        payload["diagnostic_only"] = True
        del payload["max_step_m"]
        with self.assertRaisesRegex(ValueError, "exactly four"):
            _limits_from_payload(payload)

    def test_group_summary_retains_total_risk_rejection_memberships(self):
        metrics = {
            "overall_status": "OBSERVED_ROOT_CONNECTED",
            "terrain_candidate_cells": 4,
            "terrain_candidate_area_m2": 1.0,
            "risk_constrained_candidate_cells": 1,
            "risk_constrained_candidate_area_m2": 0.25,
            "terrain_connected_cells": 3,
            "terrain_connected_area_m2": 0.75,
            "risk_constrained_connected_cells": 1,
            "risk_constrained_connected_area_m2": 0.25,
            "risk_observed_cells": 4,
            "risk_rejection": {
                "risk_rejected_candidate_cells": 3,
                "risk_rejected_candidate_area_m2": 0.75,
                "source_membership_cells_within_terrain_candidate": {
                    "measured": 1,
                    "object": 1,
                    "unknown": 2,
                    "risk_unobserved": 0,
                },
                "risk_reason_bit_counts_within_terrain_candidate": {
                    "measured": 1,
                    "object": 1,
                    "unknown": 2,
                    "risk_unobserved": 0,
                },
                "risk_membership_combinations": {
                    "measured": 1,
                    "object": 1,
                    "unknown": 1,
                },
                "sole_risk_rejection_counts": {"measured": 1},
                "overlapping_risk_rejection_counts": {"object+unknown": 1},
            },
        }
        summary = _summarize_frames([{"metrics": metrics}])
        membership = summary["risk_rejection"][
            "source_membership_cells_within_terrain_candidate"
        ]
        self.assertEqual(membership["measured"], 1)
        self.assertEqual(membership["object"], 1)
        self.assertEqual(membership["unknown"], 2)
        self.assertEqual(membership.get("risk_unobserved", 0), 0)
        self.assertEqual(
            summary["risk_rejection"]["sole_counts"],
            {"measured": 1},
        )
        self.assertEqual(
            summary["risk_rejection"]["overlap_counts"],
            {"object+unknown": 1},
        )


if __name__ == "__main__":
    unittest.main()
