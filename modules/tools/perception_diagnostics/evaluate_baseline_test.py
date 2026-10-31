# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Reject incomplete or contradictory replay evidence in the diagnostic module."""

import unittest
from types import SimpleNamespace

from evaluate_baseline import audit_frame, coverage


class CoverageTest(unittest.TestCase):
    def setUp(self):
        self.manifest = {"samples": [{"file": "frame.cloud.pb"}]}
        self.rows = [
            {"frame": "frame.cloud.pb", "mode": str(i), "free_cells": "0"}
            for i in range(4)
        ]

    def test_complete_comparison_is_accepted(self):
        coverage(self.manifest, self.rows)

    def test_partial_or_duplicate_modes_cannot_be_a_baseline(self):
        for rows in (self.rows[:-1], self.rows + [self.rows[0]]):
            with self.assertRaisesRegex(ValueError, "coverage"):
                coverage(self.manifest, rows)

    def test_unverified_free_cannot_pass(self):
        self.rows[0]["free_cells"] = "1"
        with self.assertRaisesRegex(ValueError, "FREE"):
            coverage(self.manifest, self.rows)


class OccupancyAuditTest(unittest.TestCase):
    def setUp(self):
        self.config = SimpleNamespace(
            min_x=0,
            min_y=0,
            max_x=1,
            max_y=1,
            resolution=0.25,
            obstacle_min_height=0.15,
            clearance_height=2.5,
        )
        point = SimpleNamespace(
            point_id=0,
            finite=True,
            ground_valid=False,
            is_ground_geo=False,
            position=SimpleNamespace(x=0.1, y=0.1, z=0.8),
            group=0,
        )
        decision = SimpleNamespace(
            point_id=0,
            explanation=0,
            HasField=lambda name: False,
        )
        self.cell = SimpleNamespace(
            x_index=0,
            y_index=0,
            occupancy_obs=True,
            occupancy_det=False,
            occupancy_unknown=False,
            traversability=1,
            object_ref=[],
            ground_supported=False,
            endpoint_observed=True,
            visibility_mask=0,
        )
        self.out = SimpleNamespace(
            point_evidence=[point],
            point_decision=[decision],
            object=[],
            unknown=[],
            point_view=[],
            space_map=SimpleNamespace(cell=[self.cell], width=4, height=4),
        )

    def test_actual_point_requires_measured_occupancy(self):
        self.assertEqual(audit_frame(self.out, self.config)["measured_cells"], 1)
        self.cell.occupancy_obs = False
        with self.assertRaisesRegex(ValueError, "measured occupancy"):
            audit_frame(self.out, self.config)

    def test_inference_cannot_replace_actual_measurement(self):
        self.cell.occupancy_obs = False
        self.cell.occupancy_det = True
        with self.assertRaisesRegex(ValueError, "measured occupancy"):
            audit_frame(self.out, self.config)

    def test_dangling_cell_reference_is_not_a_valid_baseline(self):
        self.cell.object_ref = [1]
        with self.assertRaisesRegex(ValueError, "dangling cell"):
            audit_frame(self.out, self.config)

    def test_functional_projection_cannot_masquerade_as_a_full_trace(self):
        self.out.publication = SimpleNamespace(detail=1)
        self.out.point_evidence = []
        self.out.point_decision = []
        with self.assertRaisesRegex(ValueError, "FULL_EVIDENCE diagnostic"):
            audit_frame(self.out, self.config)


if __name__ == "__main__":
    unittest.main()
