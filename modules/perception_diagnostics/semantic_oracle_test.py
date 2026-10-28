# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Semantic oracle controls: GT is explicit, indexed, and never instance evidence."""

import unittest
from types import SimpleNamespace

import numpy as np
from semantic_oracle import substitute


def message(indices):
    return SimpleNamespace(
        point_label=[
            SimpleNamespace(point_index=i, semantic_label=14, confidence=0.25)
            for i in indices
        ]
    )


class SemanticOracleTest(unittest.TestCase):
    def test_remaps_motion_without_instance_bits_and_retains_ignored(self):
        labels = np.array([(44 << 16) | 252, 99, 40], dtype=np.uint32)
        result = message([2, 0, 1])
        self.assertEqual(substitute(result, labels), 2)
        self.assertEqual(
            [
                (p.point_index, p.semantic_label, p.confidence)
                for p in result.point_label
            ],
            [(2, 9, 1.0), (0, 1, 1.0), (1, 14, 0.25)],
        )

    def test_bad_mapping_or_count_is_rejected_before_substitution(self):
        for indices in ([0], [0, 0], [0, 2]):
            result = message(indices)
            with self.assertRaisesRegex(ValueError, "oracle point"):
                substitute(result, np.array([10, 40], dtype=np.uint32))
            self.assertTrue(all(p.semantic_label == 14 for p in result.point_label))
        with self.assertRaisesRegex(ValueError, "unmapped"):
            substitute(message([0]), np.array([999], dtype=np.uint32))


if __name__ == "__main__":
    unittest.main()
