# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Regression cases for labeled matching and official coordinate adaptation."""

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from nuscenes import box, in_roi, match

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lidar_fusion" / "tools"))
from prepare_nuscenes import category, records, rotation  # noqa: E402


def candidate(x=0.0, cls="car", score=0.8):
    return {"center": [x, 0, 0], "class": cls, "score": score}


class MatchingTest(unittest.TestCase):
    def test_distance_gate_and_class_are_explicit(self):
        counts, _ = match([candidate(2.0)], [candidate()])
        self.assertEqual(counts["car"]["tp"], 1)
        counts, _ = match([candidate(2.001)], [candidate()])
        self.assertEqual(dict(counts["car"]), {"tp": 0, "fp": 1, "fn": 1})
        counts, _ = match([candidate(cls="bus")], [candidate()])
        self.assertEqual(counts["bus"]["fp"], 1)
        self.assertEqual(counts["car"]["fn"], 1)

    def test_score_order_and_one_to_one(self):
        counts, matches = match(
            [candidate(0, score=0.2), candidate(1, score=0.9)], [candidate()]
        )
        self.assertEqual(dict(counts["car"]), {"tp": 1, "fp": 1, "fn": 0})
        self.assertEqual(matches[0]["prediction"], 1)
        self.assertIsNone(matches[1]["truth"])

    def test_no_truth_or_predictions(self):
        counts, _ = match([candidate()], [])
        self.assertEqual(counts["car"]["fp"], 1)
        counts, _ = match([], [candidate()])
        self.assertEqual(counts["car"]["fn"], 1)

    def test_roi_is_in_base_frame_and_half_open(self):
        config = SimpleNamespace(min_x=-50, max_x=50, min_y=-50, max_y=50)
        self.assertTrue(in_roi(candidate(-50), config))
        self.assertFalse(in_roi(candidate(50), config))


class CoordinateTest(unittest.TestCase):
    def test_quaternion_rotation_and_box_heading(self):
        r = rotation([math.sqrt(0.5), 0, 0, math.sqrt(0.5)])
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(
            r.T @ ([10, 4, 0] - np.array([10, 3, 0])), [1, 0, 0], atol=1e-12
        )
        obj = SimpleNamespace(
            box=SimpleNamespace(
                center=SimpleNamespace(x=1, y=2, z=3),
                size=SimpleNamespace(x=4, y=2, z=1.5),
                qx=0,
                qy=0,
                qz=math.sqrt(0.5),
                qw=math.sqrt(0.5),
            )
        )
        self.assertAlmostEqual(box(obj)["yaw"], math.pi / 2)
        self.assertEqual(box(obj)["size"], [4, 2, 1.5])
        with self.assertRaisesRegex(ValueError, "quaternion"):
            rotation([2, 0, 0, 0])

    def test_categories_do_not_invent_supported_ground_truth(self):
        self.assertEqual(category("human.pedestrian.adult"), "pedestrian")
        self.assertEqual(category("vehicle.bus.rigid"), "bus")
        self.assertIsNone(category("animal"))

    def test_large_json_table_can_cross_read_boundaries(self):
        values = [{"token": "x" * (1024 * 1024 + 17)}, {"token": "tail"}]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "table.json"
            path.write_text(json.dumps(values))
            self.assertEqual(list(records(path)), values)
            path.write_text('[{"token":')
            with self.assertRaisesRegex(ValueError, "malformed"):
                list(records(path))


if __name__ == "__main__":
    unittest.main()
