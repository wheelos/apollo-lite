# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Analytic observed-terrain, independent-risk and exact-root regressions."""

import copy
import unittest
from dataclasses import replace

import numpy as np
import terrain_traversability as traversability
from terrain_surface import TerrainSurfaceConfig, estimate_terrain_surface

LIMITS = traversability.TraversabilityLimits(0.3, 0.05, 0.2, 0.2)


def surface(
    height=lambda _x, _y: 0.0,
    *,
    extra=(),
    risk_mask=None,
    cells=None,
    root_anchor_id=0,
):
    cells = sorted(cells if cells is not None else np.ndindex((4, 4)))
    points = [[0.25 + x * 0.5, 0.25 + y * 0.5, height(x, y)] for x, y in cells]
    points.extend(extra)
    config = TerrainSurfaceConfig(
        x_min=0,
        x_max=2,
        y_min=0,
        y_max=2,
        cell_size=0.5,
        min_anchor_points=1,
        expected_ground_z=None,
    )
    return estimate_terrain_surface(
        np.asarray(points),
        root_anchor_id=root_anchor_id,
        risk_mask=risk_mask,
        config=config,
    )


def classify(estimate, limits=LIMITS, **kwargs):
    return traversability.classify_traversability(estimate, limits, **kwargs)


class TerrainTraversabilityTest(unittest.TestCase):
    def test_flat_and_analytic_ramp_are_observed_candidates_not_safe(self):
        for height in (lambda _x, _y: 0.0, lambda x, y: 0.1 * x + 0.05 * y):
            estimate = surface(height)
            result = classify(estimate, risk_observed=np.ones((4, 4), dtype=bool))
            self.assertTrue(result.terrain_candidate.all())
            self.assertTrue(result.risk_constrained_connected.all())
            self.assertTrue((result.terrain_state == "TERRAIN_PASSABLE").all())
            self.assertTrue((result.decision_state == "CANDIDATE").all())
            self.assertFalse(result.reason_bits.any())
            self.assertTrue(np.isnan(estimate.cell_step_m).all())
            self.assertEqual(result.root_cell, estimate.root_cell)

    def test_explicit_tighter_slope_and_roughness_limits_block(self):
        estimate = surface(lambda x, _y: 0.1 * (0.25 + 0.5 * x))
        self.assertTrue(
            classify(
                estimate, replace(LIMITS, max_slope_gradient=0.15)
            ).terrain_candidate.all()
        )
        result = classify(estimate, replace(LIMITS, max_slope_gradient=0.05))
        self.assertTrue((result.terrain_state == "BLOCKED").all())
        self.assertTrue(
            (result.reason_bits & traversability.REASON_SLOPE_EXCEEDS).all()
        )
        estimate = surface()
        estimate.cell_roughness_m[1, 1] = 0.06
        result = classify(estimate)
        self.assertEqual(result.terrain_state[1, 1], "BLOCKED")
        self.assertTrue(
            result.reason_bits[1, 1] & traversability.REASON_ROUGHNESS_EXCEEDS
        )

    def test_measured_boundary_and_pit_block_even_unsupported_floor(self):
        estimate = surface(lambda x, _y: 0.3 if x >= 2 else 0.0)
        result = classify(estimate)
        self.assertTrue(
            (result.reason_bits[1, :] & traversability.REASON_STEP_EXCEEDS).all()
        )
        below_step = classify(estimate, replace(LIMITS, max_step_m=0.6))
        self.assertTrue(below_step.terrain_candidate[1, :].all())
        self.assertFalse(result.terrain_connected[2:, :].any())
        estimate = surface(
            lambda x, y: -0.3 if (x, y) == (1, 1) else 0.0,
            extra=([0.75, 0.75, -0.3],) * 2,
            root_anchor_id=3,
        )
        self.assertEqual(estimate.cell_status[1, 1], "unknown_gap")
        self.assertAlmostEqual(estimate.pit_depth_m[1, 1], 0.3)
        result = classify(estimate)
        self.assertEqual(result.terrain_state[1, 1], "BLOCKED")
        self.assertTrue(result.reason_bits[1, 1] & traversability.REASON_PIT_EXCEEDS)
        self.assertTrue(result.reason_bits[1, 1] & traversability.REASON_UNSUPPORTED)
        below = classify(estimate, replace(LIMITS, max_depression_depth_m=0.6))
        self.assertEqual(below.terrain_state[1, 1], "UNKNOWN")
        self.assertFalse(below.terrain_candidate[1, 1])

    def test_missing_floor_and_sparse_local_rank_do_not_qualify(self):
        cells = set(np.ndindex((4, 4))) - {(1, 1)}
        result = classify(surface(cells=cells))
        self.assertEqual(result.terrain_state[1, 1], "UNKNOWN")
        self.assertFalse(result.terrain_connected[1, 1])
        self.assertTrue(result.reason_bits[1, 1] & traversability.REASON_PIT_UNKNOWN)
        estimate = surface(cells={(0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (3, 0)})
        result = classify(estimate)
        self.assertFalse(result.terrain_candidate[3, 0])
        self.assertEqual(result.terrain_state[3, 0], "GROUND_SUPPORTED")
        self.assertTrue(
            result.reason_bits[3, 0] & traversability.REASON_SLOPE_UNOBSERVED
        )
        self.assertTrue(
            result.reason_bits[3, 0] & traversability.REASON_ROUGHNESS_UNOBSERVED
        )
        rankless = classify(surface(cells={(0, y) for y in range(4)}))
        self.assertFalse(rankless.terrain_candidate.any())
        self.assertEqual(rankless.overall_status, "UNKNOWN_NO_ROOT")

    def test_bridge_and_small_raw_multilayer_are_unknown_not_physical_step(self):
        for separation in (0.21, 4.0):
            estimate = surface(extra=([0.75, 0.75, separation],))
            result = classify(estimate)
            self.assertEqual(result.terrain_state[1, 1], "UNKNOWN")
            self.assertFalse(result.terrain_candidate[1, 1])
            self.assertTrue(
                result.reason_bits[1, 1] & traversability.REASON_LAYER_AMBIGUITY
            )
            self.assertFalse(
                result.reason_bits[1, 1] & traversability.REASON_STEP_EXCEEDS
            )
            estimate.cell_step_status[1, 1] = "no_discontinuity_observed"
            estimate.pit_status[1, 1] = "unknown_no_rim"
            self.assertEqual(classify(estimate).terrain_state[1, 1], "UNKNOWN")

    def test_combined_boundary_and_span_preserves_blocker_and_unknown_reasons(self):
        estimate = surface(
            lambda x, _y: 0.3 if x >= 2 else 0.0,
            extra=([0.75, 0.75, 4.0],),
        )
        self.assertEqual(
            estimate.cell_step_status[1, 1], "layer_span_and_boundary_discontinuity"
        )
        result = classify(estimate)
        self.assertEqual(result.terrain_state[1, 1], "BLOCKED")
        for reason in (
            traversability.REASON_LAYER_AMBIGUITY,
            traversability.REASON_STEP_EXCEEDS,
            traversability.REASON_PIT_UNKNOWN,
        ):
            self.assertTrue(result.reason_bits[1, 1] & reason)

    def test_missing_metrics_and_unknown_statuses_stay_ground_supported(self):
        for field, value, reason in (
            ("cell_slope_gradient", np.nan, traversability.REASON_SLOPE_UNOBSERVED),
            ("cell_roughness_m", np.nan, traversability.REASON_ROUGHNESS_UNOBSERVED),
            (
                "cell_step_status",
                "unknown_unsupported",
                traversability.REASON_STEP_UNKNOWN,
            ),
            ("pit_status", "unknown_no_rim", traversability.REASON_PIT_UNKNOWN),
            (
                "cell_step_status",
                "observed_boundary_discontinuity",
                traversability.REASON_STEP_UNKNOWN,
            ),
            ("pit_status", "measured", traversability.REASON_PIT_UNKNOWN),
        ):
            estimate = surface()
            getattr(estimate, field)[1, 1] = value
            result = classify(estimate, risk_observed=np.ones((4, 4), dtype=bool))
            self.assertEqual(result.terrain_state[1, 1], "GROUND_SUPPORTED")
            self.assertEqual(result.decision_state[1, 1], "UNKNOWN")
            self.assertTrue(result.reason_bits[1, 1] & reason)

    def test_risk_sources_veto_independently_and_keep_all_reason_memberships(self):
        estimate = surface()
        layers = np.zeros((4, 4), dtype=np.uint8)
        layers[0, 1] = traversability.MEASURED
        layers[1, 1] = traversability.OBJECT
        layers[2, 1] = traversability.UNKNOWN_RISK
        layers[3, 1] = 7
        result = classify(estimate, risk_layers=layers)
        self.assertTrue(result.terrain_candidate.all())
        self.assertEqual(result.decision_state[0, 1], "BLOCKED")
        self.assertEqual(result.decision_state[1, 1], "BLOCKED")
        self.assertEqual(result.decision_state[2, 1], "UNKNOWN")
        self.assertEqual(result.decision_state[3, 1], "BLOCKED")
        for reason in (
            traversability.REASON_MEASURED_RISK,
            traversability.REASON_OBJECT_RISK,
            traversability.REASON_UNKNOWN_RISK,
            traversability.REASON_RISK_UNOBSERVED,
        ):
            self.assertTrue(result.reason_bits[3, 1] & reason)

    def test_original_risk_on_other_height_layer_cannot_be_erased(self):
        risk = np.zeros(18, dtype=bool)
        risk[5] = True
        risk[-2] = True
        risk[-1] = True
        estimate = surface(extra=([1.25, 1.25, 4.0], [10.0, 10.0, 0.0]), risk_mask=risk)
        for supplied in (
            None,
            np.zeros((4, 4), dtype=np.uint8),
            np.zeros((4, 4), dtype=np.uint8, order="F"),
        ):
            result = classify(
                estimate,
                risk_layers=supplied,
                risk_observed=np.ones((4, 4), dtype=bool),
            )
            self.assertEqual(result.risk_layers[1, 1], traversability.UNKNOWN_RISK)
            self.assertEqual(result.risk_layers[2, 2], traversability.UNKNOWN_RISK)
            self.assertEqual(result.risk_layers[3, 3], 0)
            self.assertEqual(result.decision_state[1, 1], "UNKNOWN")
            self.assertTrue(estimate.point_ground_candidate_mask[5])
        layers = np.zeros((4, 4), dtype=np.uint8)
        layers[2, 2] = traversability.OBJECT
        result = classify(estimate, risk_layers=layers)
        self.assertEqual(result.risk_layers[2, 2], traversability.OBJECT)
        self.assertEqual(result.risk_layers[1, 1], traversability.UNKNOWN_RISK)
        self.assertEqual(result.decision_state[2, 2], "BLOCKED")

    def test_no_risk_coverage_is_not_zero_risk(self):
        estimate = surface()
        for kwargs in ({}, {"risk_layers": np.zeros((4, 4), dtype=np.uint8)}):
            result = classify(estimate, **kwargs)
            self.assertTrue(result.terrain_connected.all())
            self.assertFalse(result.risk_constrained_candidate.any())
            self.assertTrue((result.decision_state == "UNKNOWN").all())
            self.assertTrue(
                (result.reason_bits & traversability.REASON_RISK_UNOBSERVED).all()
            )
        observed = np.ones((4, 4), dtype=bool)
        observed[1, 1] = False
        result = classify(estimate, risk_observed=observed)
        self.assertTrue(result.risk_constrained_candidate[0, 0])
        self.assertFalse(result.risk_constrained_candidate[1, 1])

    def test_exact_root_no_reseeding_diagonals_or_hole_filling(self):
        estimate = surface()
        estimate.cell_roughness_m[1, :] = 0.1
        result = classify(estimate, risk_observed=np.ones((4, 4), dtype=bool))
        self.assertTrue(result.terrain_candidate[2:, :].all())
        self.assertFalse(result.terrain_connected[2:, :].any())
        self.assertTrue(
            (result.reason_bits[2:, :] & traversability.REASON_DISCONNECTED).all()
        )
        estimate = surface()
        estimate.cell_roughness_m[0, 1] = 0.1
        estimate.cell_roughness_m[1, 0] = 0.1
        result = classify(estimate)
        self.assertEqual(np.count_nonzero(result.terrain_connected), 1)
        estimate.cell_roughness_m[0, 0] = 0.1
        result = classify(estimate)
        self.assertFalse(result.terrain_connected.any())
        self.assertEqual(result.overall_status, "OBSERVED_ROOT_VETOED")
        estimate.root_cell = None
        self.assertEqual(classify(estimate).overall_status, "UNKNOWN_NO_ROOT")
        self.assertFalse(classify(estimate).terrain_connected.any())

    def test_risk_wall_and_root_veto_do_not_reseed_terrain_component(self):
        estimate = surface()
        layers = np.zeros((4, 4), dtype=np.uint8)
        layers[1, :] = traversability.OBJECT
        kwargs = {"risk_layers": layers, "risk_observed": np.ones((4, 4), dtype=bool)}
        result = classify(estimate, **kwargs)
        self.assertTrue(result.terrain_connected.all())
        self.assertTrue(result.risk_constrained_candidate[2:, :].all())
        self.assertFalse(result.risk_constrained_connected[2:, :].any())
        layers[0, 0] = traversability.UNKNOWN_RISK
        result = classify(estimate, **kwargs)
        self.assertFalse(result.risk_constrained_connected.any())
        self.assertTrue(result.terrain_connected.all())

    def test_connectivity_checks_observed_neighbor_height_and_gradient(self):
        estimate = surface()
        estimate.cell_elevation_z[2:, :] = 1.0
        result = classify(estimate)
        self.assertTrue(result.terrain_candidate.all())
        self.assertFalse(result.terrain_connected[2:, :].any())
        estimate.cell_elevation_z[2:, :] = 0.1
        result = classify(estimate, replace(LIMITS, max_slope_gradient=0.1))
        self.assertFalse(result.terrain_connected[2:, :].any())
        self.assertTrue(classify(estimate).terrain_connected.all())

    def test_inputs_preserved_and_raw_risk_grids_copied(self):
        risk = np.zeros(16, dtype=bool)
        risk[0] = True
        estimate = surface(risk_mask=risk)
        original = copy.deepcopy(estimate)
        layers = np.zeros((4, 4), dtype=np.uint8)
        observed = np.ones((4, 4), dtype=bool)
        result = classify(estimate, risk_layers=layers, risk_observed=observed)
        result.risk_layers[0, 0] = 7
        result.risk_observed[0, 0] = False
        self.assertFalse(layers.any())
        self.assertTrue(observed.all())
        for name, value in vars(estimate).items():
            if isinstance(value, np.ndarray):
                np.testing.assert_array_equal(value, getattr(original, name))
            else:
                self.assertEqual(value, getattr(original, name))

    def test_invalid_thresholds_types_bits_shapes_and_coverage_rejected(self):
        estimate = surface()
        for name in vars(LIMITS):
            for bad in (0, -1, np.nan, np.inf, True, "0.2", 1j):
                with self.subTest(name=name, bad=bad), self.assertRaises(ValueError):
                    classify(estimate, replace(LIMITS, **{name: bad}))
        for layers in (
            np.zeros((3, 4), dtype=np.uint8),
            np.full((4, 4), -1),
            np.full((4, 4), 8),
            np.full((4, 4), 1.5),
            np.full((4, 4), np.nan),
            np.zeros((4, 4), dtype=bool),
            np.full((4, 4), "1"),
        ):
            with self.assertRaises(ValueError):
                classify(estimate, risk_layers=layers)
        for observed in (np.zeros((3, 4), dtype=bool), np.zeros((4, 4))):
            with self.assertRaises(ValueError):
                classify(estimate, risk_observed=observed)
        with self.assertRaises(ValueError):
            classify(estimate, None)
        with self.assertRaises(ValueError):
            classify(None)

    def test_measured_checkerboard_roughness_crosses_explicit_limits(self):
        estimate = surface(lambda x, y: 0.02 * (-1) ** (x + y))
        original = copy.deepcopy(estimate)
        roughness = 0.02 * np.sqrt(10 / 9)
        self.assertAlmostEqual(estimate.cell_roughness_m[1, 1], roughness, places=12)
        for limit, passes in ((0.015, False), (0.03, True)):
            result = classify(estimate, replace(LIMITS, max_roughness_m=limit))
            self.assertEqual(result.terrain_candidate[1, 1], passes)
            self.assertEqual(
                bool(
                    result.reason_bits[1, 1] & traversability.REASON_ROUGHNESS_EXCEEDS
                ),
                not passes,
            )
        np.testing.assert_array_equal(
            estimate.cell_roughness_m, original.cell_roughness_m
        )
        self.assertEqual(estimate.root_cell, original.root_cell)

    def test_exact_four_metric_boundaries_and_candidate_monotonicity(self):
        for field, limit_name, status_field, status, reason in (
            (
                "cell_slope_gradient",
                "max_slope_gradient",
                None,
                None,
                traversability.REASON_SLOPE_EXCEEDS,
            ),
            (
                "cell_roughness_m",
                "max_roughness_m",
                None,
                None,
                traversability.REASON_ROUGHNESS_EXCEEDS,
            ),
            (
                "cell_step_m",
                "max_step_m",
                "cell_step_status",
                "observed_boundary_discontinuity",
                traversability.REASON_STEP_EXCEEDS,
            ),
            (
                "pit_depth_m",
                "max_depression_depth_m",
                "pit_status",
                "measured",
                traversability.REASON_PIT_EXCEEDS,
            ),
        ):
            estimate = surface()
            if status_field:
                getattr(estimate, status_field)[1, 1] = status
            threshold = getattr(LIMITS, limit_name)
            for offset in (-1e-6, 0.0, 1e-6):
                getattr(estimate, field)[1, 1] = threshold + offset
                result = classify(estimate)
                self.assertEqual(result.terrain_candidate[1, 1], offset <= 0)
                self.assertEqual(bool(result.reason_bits[1, 1] & reason), offset > 0)
            original = getattr(estimate, field).copy()
            tight = classify(estimate)
            loose = classify(
                estimate, replace(LIMITS, **{limit_name: threshold + 2e-6})
            )
            self.assertTrue(loose.terrain_candidate[1, 1])
            for name in ("terrain_candidate", "terrain_connected"):
                self.assertFalse((getattr(tight, name) & ~getattr(loose, name)).any())
            np.testing.assert_array_equal(getattr(estimate, field), original)

        estimate = surface()
        estimate.pit_status[1, 1] = "measured"
        estimate.pit_depth_m[1, 1] = 0.15
        self.assertTrue(classify(estimate).terrain_candidate[1, 1])
        self.assertEqual(
            classify(
                estimate, replace(LIMITS, max_depression_depth_m=0.10)
            ).terrain_state[1, 1],
            "BLOCKED",
        )

    def test_z_translation_and_affine_tilt_preserve_relative_observables(self):
        risk = np.zeros(18, dtype=bool)
        risk[0] = True
        base = surface(
            lambda x, y: -0.3 if (x, y) == (1, 1) else 0.0,
            extra=([0.75, 0.75, -0.3],) * 2,
            risk_mask=risk,
            root_anchor_id=3,
        )
        self.assertEqual(base.overall_status, "supported")
        self.assertEqual(base.pit_status[1, 1], "measured")
        self.assertAlmostEqual(base.pit_depth_m[1, 1], 0.3)
        for a, b, c in ((0.0, 0.0, 7.0), (0.02, 0.01, 7.0)):
            transformed = surface(
                lambda x, y: (-0.3 if (x, y) == (1, 1) else 0.0)
                + a * (0.25 + x * 0.5)
                + b * (0.25 + y * 0.5)
                + c,
                extra=([0.75, 0.75, -0.3 + 0.75 * (a + b) + c],) * 2,
                risk_mask=risk,
                root_anchor_id=3,
            )
            self.assertEqual(transformed.root_anchor_id, base.root_anchor_id)
            self.assertEqual(transformed.root_cell, base.root_cell)
            known = np.isfinite(base.cell_slope_gradient)
            np.testing.assert_allclose(
                transformed.cell_slope_gradient[known], np.hypot(a, b), atol=1e-12
            )
            for name in ("cell_roughness_m", "cell_step_m", "pit_depth_m"):
                np.testing.assert_allclose(
                    getattr(transformed, name),
                    getattr(base, name),
                    atol=1e-12,
                    equal_nan=True,
                )
            first, second = classify(base), classify(transformed)
            for name in (
                "terrain_state",
                "decision_state",
                "reason_bits",
                "terrain_candidate",
                "terrain_connected",
                "risk_constrained_candidate",
                "risk_constrained_connected",
                "risk_layers",
                "risk_observed",
            ):
                np.testing.assert_array_equal(
                    getattr(first, name), getattr(second, name)
                )

    def test_upper_root_single_deck_and_overlapping_layers(self):
        for overlap in (False, True):
            cells = set(np.ndindex((4, 4)))
            estimate = surface(
                lambda x, _y: 4.0 if x < 2 else 0.0,
                extra=([0.25, 0.75, 0.0],) if overlap else (),
                cells=cells,
            )
            self.assertEqual(estimate.root_anchor_id, 0)
            self.assertEqual(estimate.cell_elevation_z[0, 1], 4.0)
            result = classify(estimate)
            self.assertFalse(result.terrain_connected[2:, :].any())
            if overlap:
                self.assertEqual(len(estimate.cell_observed_layers[0, 1]), 2)
                self.assertEqual(result.terrain_state[0, 1], "UNKNOWN")
                self.assertTrue(
                    result.reason_bits[0, 1] & traversability.REASON_LAYER_AMBIGUITY
                )
                self.assertFalse(
                    result.reason_bits[0, 1] & traversability.REASON_STEP_EXCEEDS
                )
            else:
                self.assertTrue(result.terrain_candidate[0, 1])

    def test_unobserved_band_and_insufficient_floor_or_rim_remain_unknown(self):
        band = surface(cells={(x, y) for x, y in np.ndindex((4, 4)) if x != 2})
        result = classify(band)
        self.assertTrue((result.terrain_state[2, :] == "UNKNOWN").all())
        self.assertTrue(
            (result.reason_bits[2, :] & traversability.REASON_UNSUPPORTED).all()
        )
        self.assertFalse(result.terrain_connected[2:, :].any())
        sparse = surface(
            lambda x, y: -0.3 if (x, y) == (1, 1) else 0.0,
            root_anchor_id=3,
        )
        partial_rim = surface(
            cells={(x, y) for x in range(2) for y in range(4)},
            extra=([1.25, 0.75, -0.3],) * 3,
        )
        for estimate, cell, status in (
            (sparse, (1, 1), "unknown_insufficient_floor"),
            (partial_rim, (2, 1), "unknown_no_rim"),
        ):
            self.assertEqual(estimate.pit_status[cell], status)
            self.assertTrue(np.isnan(estimate.pit_depth_m[cell]))
            result = classify(estimate)
            self.assertEqual(result.terrain_state[cell], "UNKNOWN")
            self.assertFalse(result.terrain_candidate[cell])
            self.assertTrue(
                result.reason_bits[cell] & traversability.REASON_PIT_UNKNOWN
            )

    def test_external_overhead_unknown_risk_preserves_single_layer_deck(self):
        estimate = surface(lambda _x, _y: 4.0)
        original = copy.deepcopy(estimate)
        layers = np.zeros((4, 4), dtype=np.uint8)
        layers[1, 1] = traversability.UNKNOWN_RISK
        observed = np.ones((4, 4), dtype=bool)
        result = classify(estimate, risk_layers=layers, risk_observed=observed)
        self.assertEqual(len(estimate.cell_observed_layers[1, 1]), 1)
        self.assertTrue(result.terrain_candidate[1, 1])
        self.assertEqual(result.decision_state[1, 1], "UNKNOWN")
        self.assertFalse(result.risk_constrained_candidate[1, 1])
        self.assertTrue(result.reason_bits[1, 1] & traversability.REASON_UNKNOWN_RISK)
        self.assertEqual(result.root_cell, original.root_cell)
        for name in ("cell_elevation_z", "cell_slope_gradient", "cell_roughness_m"):
            np.testing.assert_array_equal(
                getattr(estimate, name), getattr(original, name)
            )

    def test_isolated_return_cannot_bridge_unobserved_band_or_replace_root(self):
        cells = {(0, 0), (0, 1), (1, 0), (1, 1), (3, 3)}
        base = surface(cells=cells)
        for z in (0.0, 1.0):
            estimate = surface(cells=cells, extra=([1.25, 0.25, z],))
            self.assertEqual(estimate.root_anchor_id, base.root_anchor_id)
            self.assertEqual(estimate.cell_raw_return_count[2, 0], 1)
            self.assertEqual(len(estimate.cell_observed_layers[2, 0]), 1)
            result = classify(estimate)
            self.assertFalse(result.terrain_candidate[2, 0])
            self.assertFalse(result.terrain_connected[3, :].any())
            self.assertFalse(estimate.point_risk_mask.any())
            self.assertEqual(len(estimate.point_ids), len(base.point_ids) + 1)

    def test_invalid_explicit_source_roots_reject_without_fallback(self):
        with self.assertRaises(ValueError):
            surface(root_anchor_id=1000)
        with self.assertRaises(ValueError):
            surface(extra=([10.0, 10.0, 0.0],), root_anchor_id=16)
        undersupported = surface(cells={(0, y) for y in range(4)}, root_anchor_id=0)
        self.assertEqual(undersupported.overall_status, "unknown_no_anchor")
        self.assertIsNone(undersupported.root_cell)
        result = classify(undersupported)
        self.assertEqual(result.overall_status, "UNKNOWN_NO_ROOT")
        self.assertFalse(result.terrain_connected.any())
        self.assertFalse(result.risk_constrained_connected.any())

    def test_connected_bowl_exposes_selected_depression_identifiability_boundary(self):
        estimate = surface(lambda x, y: 0.02 * ((x - 1.5) ** 2 + (y - 1.5) ** 2))
        self.assertTrue((estimate.cell_status == "supported").all())
        self.assertLess(
            estimate.cell_elevation_z[1, 1], estimate.cell_elevation_z[0, 0]
        )
        self.assertTrue((estimate.pit_status == "no_depression_observed").all())
        self.assertTrue(np.isnan(estimate.pit_depth_m).all())
        result = classify(estimate)
        self.assertTrue(result.terrain_candidate.all())
        self.assertTrue(result.terrain_connected.all())
        self.assertTrue((result.decision_state == "UNKNOWN").all())
        alternative_interpretation = classify(copy.deepcopy(estimate))
        np.testing.assert_array_equal(
            result.terrain_state, alternative_interpretation.terrain_state
        )


if __name__ == "__main__":
    unittest.main()
