# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""SemanticKITTI audit regression tests with no model or dataset dependency."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np
from semantickitti import (
    binary_counts,
    confusion,
    inside_box,
    instance_counts,
    learning_labels,
    replay_caches,
    semantic_metrics,
    unknown_support_metrics,
    visible_instances,
)

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "lidar_fusion" / "tools")
)
from prepare_semantickitti import quantize_remission  # noqa: E402


class SemanticAuditTest(unittest.TestCase):
    def test_cpu_cached_replay_preserves_both_model_cache_paths(self):
        args = ["replay_cached", "clouds", "models", "output"]
        expected = (Path("models"), Path("models"))
        self.assertEqual(replay_caches(args), expected)
        self.assertEqual(replay_caches(args + ["3", "--profile"]), expected)
        for options in (
            ["--shadow-partitions"],
            ["3", "--shadow-partitions"],
            ["3", "--profile", "--shadow-partitions"],
            ["--shadow-partitions", "--profile"],
        ):
            self.assertEqual(replay_caches(args + options), expected)
        self.assertEqual(
            replay_caches(
                ["replay", "clouds", "out", "-", "-", "1", "1", "det", "seg"]
            ),
            (Path("det"), Path("seg")),
        )
        self.assertIsNone(
            replay_caches(["replay", "clouds", "out", "a", "b", "1", "1"])
        )
        for extra in (
            ["0"],
            ["101"],
            ["-1"],
            ["2", "--unknown"],
            ["--shadow-partitions", "--shadow-partitions"],
            ["3", "--profile", "--unknown"],
        ):
            with self.assertRaisesRegex(ValueError, "cached replay"):
                replay_caches(args + extra)

    def test_motion_labels_and_instance_bits_preserve_learning_classes(self):
        labels = np.array([10, 252, (12 << 16) | 254, 253, 60, 99], dtype=np.uint32)
        np.testing.assert_array_equal(learning_labels(labels), [1, 1, 6, 7, 9, 0])
        with self.assertRaisesRegex(ValueError, "unmapped"):
            learning_labels([999])

    def test_ignored_truth_not_ignored_predictions(self):
        matrix = confusion([0, 1, 1, 9], [1, 1, 0, 9])
        metrics = semantic_metrics(matrix)
        self.assertEqual(metrics["valid_points"], 3)
        self.assertAlmostEqual(metrics["accuracy"], 2 / 3)
        self.assertAlmostEqual(metrics["per_class"]["car"]["iou"], 0.5)
        self.assertAlmostEqual(metrics["miou_present_classes"], 0.75)
        with self.assertRaisesRegex(ValueError, "invalid prediction"):
            confusion([1], [20])

    def test_remission_conversion_has_explicit_quantization_bound(self):
        values = np.array([0, 0.5, 0.99, 1], dtype=np.float32)
        intensity, worst, mean = quantize_remission(values)
        np.testing.assert_array_equal(intensity, [0, 128, 252, 255])
        self.assertLessEqual(worst, 0.5 / 255 + 1e-8)
        self.assertGreater(mean, 0)
        for invalid in ([1.01], [math.nan], [-0.01]):
            with self.assertRaisesRegex(ValueError, "remission"):
                quantize_remission(np.array(invalid))

    def test_background_leak_and_target_loss_are_not_hidden(self):
        result = binary_counts(
            np.array([True, True, False, False]),
            np.array([True, False, True, True]),
            np.array([True, True, True, False]),
        )
        self.assertEqual(result, {"tp": 1, "fp": 1, "fn": 1, "tn": 0})

    def test_unknown_support_reports_mixed_ground_without_filling_extent(self):
        points = np.array([[-2, -2, 0], [2, 2, 0], [4, 4, 1], [6, 6, 1]])
        result = unknown_support_metrics(points, [9, 0, 1, 13], [[0, 0, 1, 2], [2, 3]])
        self.assertEqual(result["unique_finite_support_points"], 4)
        largest = result["largest_component"]
        self.assertEqual(largest["points"], 3)
        self.assertEqual(largest["valid_semantic_points"], 2)
        self.assertEqual(largest["semantic_ground_proxy_points"], 1)
        self.assertEqual(largest["semantic_thing_points"], 1)
        self.assertEqual(largest["support_aabb_area"], 36)
        self.assertTrue(largest["support_aabb_contains_origin"])
        self.assertEqual(result["components_spanning_origin_aabb"], 1)

    def test_unknown_support_handles_empty_nonfinite_and_invalid_indices(self):
        points = np.array([[math.nan, 0, 0]])
        self.assertIsNone(unknown_support_metrics(points, [9], [])["largest_component"])
        result = unknown_support_metrics(points, [9], [[0]])
        self.assertEqual(result["unique_finite_support_points"], 0)
        self.assertFalse(result["largest_component"]["support_aabb_contains_origin"])
        with self.assertRaisesRegex(ValueError, "index out of range"):
            unknown_support_metrics(points, [9], [[1]])


class InstanceAuditTest(unittest.TestCase):
    def test_native_box_orientation_is_applied(self):
        item = {"center": [0, 0, 0], "size": [4, 1, 2], "yaw": math.pi / 2}
        np.testing.assert_array_equal(
            inside_box(np.array([[0, 1.5, 0], [1.5, 0, 0]]), item), [True, False]
        )

    def test_instance_ids_not_semantic_components(self):
        points = np.column_stack((np.arange(12), np.zeros(12), np.zeros(12)))
        truth = np.ones(12, dtype=int)
        identity = np.array([3] * 6 + [4] * 6)
        instances, outlines = visible_instances(points, truth, identity, truth > 0)
        self.assertEqual([i["id"] for i in instances], [3, 4])
        self.assertEqual(len(outlines), 2)
        self.assertIn("NOT amodal", outlines[0]["source"])

    def test_matching_uses_one_to_one_point_iou_and_class(self):
        truth = [{"id": 2, "class_id": 1, "indices": np.arange(6)}]
        candidate = {
            "id": 10,
            "class_id": 1,
            "score": 0.9,
            "mask": np.array([True] * 6 + [False] * 6),
        }
        counts, _ = instance_counts(
            [candidate, {**candidate, "id": 11}], truth, np.ones(12, dtype=bool)
        )
        self.assertEqual(counts, {"tp": 1, "fp": 1, "fn": 0})
        counts, _ = instance_counts(
            [{**candidate, "class_id": 4}], truth, np.ones(12, dtype=bool)
        )
        self.assertEqual(counts, {"tp": 0, "fp": 1, "fn": 1})
        counts, _ = instance_counts(
            [{**candidate, "mask": np.array([True] * 2 + [False] * 10)}],
            truth,
            np.ones(12, dtype=bool),
        )
        self.assertEqual(counts, {"tp": 0, "fp": 1, "fn": 1})

    def test_unavailable_display_score_does_not_change_instance_ranking(self):
        truth = [{"id": 2, "class_id": 1, "indices": np.arange(6)}]
        first = {
            "id": 10,
            "class_id": 1,
            "score": 1.0,
            "mask": np.ones(6, dtype=bool),
        }
        second = {**first, "id": 11, "score": 0.9}
        valid = np.ones(6, dtype=bool)
        self.assertEqual(
            instance_counts(
                [{**first, "score": None, "ranking_score": 1.0}, second], truth, valid
            ),
            instance_counts([first, second], truth, valid),
        )


if __name__ == "__main__":
    unittest.main()
