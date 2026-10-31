# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Contract tests for the frozen terrain surface experiment and artifacts."""

import unittest

import numpy as np
from terrain_surface_experiment import (
    CASE_CONTRACTS,
    _layer_arrays,
    _metrics_from_arrays,
    run_analytic_cases,
)


class TerrainSurfaceExperimentTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matrix, cls.results = run_analytic_cases()
        cls.cases = {case["id"]: case for case in cls.matrix["cases"]}

    def test_frozen_case_matrix_has_objective_expectation_and_verdict(self):
        self.assertEqual(self.matrix["case_count"], len(CASE_CONTRACTS))
        self.assertEqual(set(self.cases), {case["id"] for case in CASE_CONTRACTS})
        for case in self.matrix["cases"]:
            self.assertTrue(case["objective"])
            self.assertTrue(case["expected"])
            self.assertIn(
                case["functional_verdict"], ("PASS", "FAIL", "DIAGNOSTIC", "BLOCKED")
            )
            self.assertIn("first_wrong_boundary", case)

    def test_analytic_flat_ramp_and_step_are_separate_observables(self):
        for case_id in ("flat-ground", "continuous-ramp", "abrupt-step"):
            self.assertEqual(self.cases[case_id]["functional_verdict"], "PASS")
        _, ramp = self.results["ramp"]
        supported = ramp.cell_status == "supported"
        self.assertGreater(
            float(
                np.nanmax(ramp.cell_elevation_z[supported])
                - np.nanmin(ramp.cell_elevation_z[supported])
            ),
            0.7,
        )

    def test_pit_metrics_distinguish_measured_and_unknown_support(self):
        self.assertEqual(self.cases["wide-shallow-pit"]["functional_verdict"], "PASS")
        self.assertEqual(self.cases["narrow-deep-pit"]["functional_verdict"], "PASS")
        self.assertEqual(
            self.cases["undersupported-pit-floor"]["functional_verdict"], "PASS"
        )
        sparse_points, sparse = self.results["sparse_pit_floor"]
        self.assertEqual(
            sparse.pit_status[tuple(self._sparse_floor_cell(sparse_points, sparse))],
            "unknown_insufficient_floor",
        )

    @staticmethod
    def _sparse_floor_cell(points, result):
        row = int(np.argmin(np.sum((points[:, :2] - [2.25, 0.0]) ** 2, axis=1)))
        return divmod(int(result.point_cell_id[row]), result.cell_status.shape[1])

    def test_bridge_layers_preserve_source_ids_and_support_both_root_choices(self):
        self.assertEqual(self.cases["bridge-two-level"]["functional_verdict"], "PASS")
        points, estimate = self.results["bridge"]
        cell = tuple(
            divmod(
                int(estimate.point_cell_id[0]),
                estimate.cell_status.shape[1],
            )
        )
        self.assertEqual(len(estimate.cell_observed_layers[cell]), 2)
        persisted = _layer_arrays(estimate)
        self.assertEqual(
            int(persisted["observed_layer_source_point_ids"].size), len(points)
        )
        self.assertEqual(persisted["observed_layer_source_offsets"].dtype.kind, "i")

    def test_risk_never_moves_root_or_changes_surface_fit(self):
        self.assertEqual(self.cases["small-risk-object"]["functional_verdict"], "PASS")
        points, with_risk = self.results["obstacle"]
        self.assertEqual(int(with_risk.point_risk_mask.sum()), 3)
        self.assertFalse(
            np.any(with_risk.point_ground_mask & with_risk.point_risk_mask)
        )
        self.assertEqual(
            self.cases["isolated-near-root-return"]["functional_verdict"], "PASS"
        )

    def test_unknown_gaps_and_no_anchor_are_not_zero_filled(self):
        self.assertEqual(self.cases["unknown-gap"]["functional_verdict"], "PASS")
        self.assertEqual(self.cases["no-anchor-unknown"]["functional_verdict"], "PASS")
        _, gap = self.results["gap"]
        self.assertTrue(np.isnan(gap.cell_elevation_z).any())
        _, no_anchor = self.results["no_anchor"]
        self.assertTrue(np.isnan(no_anchor.cell_elevation_z).all())

    def test_z_translation_preserves_auto_root_and_local_metrics(self):
        self.assertEqual(
            self.cases["z-translation-invariance"]["functional_verdict"], "PASS"
        )
        self.assertEqual(
            self.cases["coplanar-thing-ambiguity"]["functional_verdict"], "DIAGNOSTIC"
        )

    def test_subcell_layer_span_is_not_claimed_as_a_connected_step(self):
        case = self.cases["subcell-two-layer-boundary"]
        self.assertEqual(case["functional_verdict"], "DIAGNOSTIC")
        self.assertAlmostEqual(case["actual"]["cell_observed_layer_span_m"], 0.30)
        self.assertEqual(case["actual"]["target_cell_status"], "unknown_no_anchor")
        self.assertIsNone(case["actual"]["step_metric_m"])
        self.assertEqual(case["actual"]["step_metric_status"], "unknown_unsupported")

    def test_metrics_count_layer_ambiguity_as_unknown(self):
        arrays = {
            "point_id": np.array([0]),
            "point_ground_mask": np.array([True]),
            "point_ground_candidate_mask": np.array([True]),
            "point_risk_mask": np.array([False]),
            "cell_status": np.array(["supported"]),
            "cell_roughness_m": np.array([0.0]),
            "cell_slope_gradient": np.array([0.0]),
            "cell_step_m": np.array([1.0]),
            "pit_status": np.array(["unknown_layer_ambiguity"]),
        }
        metrics = _metrics_from_arrays(arrays, None)
        self.assertEqual(metrics["pit_unknown_cells"], 1)
        self.assertEqual(metrics["pit_measured_cells"], 0)


if __name__ == "__main__":
    unittest.main()
