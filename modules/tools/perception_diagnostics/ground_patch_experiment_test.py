# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Regression guards for the patch experiment's auditable ground boundary."""

import unittest

import numpy as np
from ground_patch_experiment import summary, validate_prediction


class PatchArtifactTest(unittest.TestCase):
    def test_rejects_filtered_or_reordered_ground_membership(self):
        points = np.array([[0.0, 0.0, -1.7], [1.0, 0.0, -1.5]])
        estimate = np.array([-1.7, -1.7])
        ground = np.array([True, False])
        validate_prediction(points, estimate, ground, 0.1)
        for corrupt in (ground[::-1], np.array([False, False])):
            with self.assertRaisesRegex(ValueError, "identity/classification"):
                validate_prediction(points, estimate, corrupt, 0.1)
        with self.assertRaisesRegex(ValueError, "identity/classification"):
            validate_prediction(points, estimate[:1], ground, 0.1)

    def test_rejects_supported_nonfinite_input(self):
        points = np.array([[np.nan, 0.0, -1.7]])
        with self.assertRaisesRegex(ValueError, "nonfinite points"):
            validate_prediction(points, np.array([-1.7]), np.array([True]), 0.1)
        validate_prediction(points, np.array([np.nan]), np.array([False]), 0.1)

    def test_roughness_unknown_is_not_zero_or_pass(self):
        result = summary(np.array([np.nan, np.nan]))
        self.assertEqual(result["available_cells"], 0)
        self.assertEqual(result["unavailable_cells"], 2)
        self.assertIsNone(result["median"])
        self.assertIsNone(result["max"])

    def test_numeric_provenance_needs_no_pickle(self):
        from ground_patch import estimate_ground_patch
        from spatial_support.observed_terrain import observed_terrain_metrics

        points = np.empty((0, 3))
        estimate = estimate_ground_patch(points)
        observed = observed_terrain_metrics(points, estimate)
        for result in (estimate, observed):
            for name, values in vars(result).items():
                if isinstance(values, np.ndarray):
                    self.assertNotEqual(
                        values.dtype,
                        object,
                        msg=f"{name} cannot be audited with allow_pickle=False",
                    )


if __name__ == "__main__":
    unittest.main()
