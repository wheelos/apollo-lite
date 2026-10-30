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
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
from nuscenes import box, in_roi, match, object_view

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "lidar_fusion" / "tools")
)
from prepare_nuscenes import category, records, rotation  # noqa: E402


def candidate(x=0.0, cls="car", score=0.8):
    return {"center": [x, 0, 0], "class": cls, "score": score}


def projection_object():
    schema = descriptor_pb2.FileDescriptorProto(
        name="projection.proto", syntax="proto2"
    )
    field = descriptor_pb2.FieldDescriptorProto
    definitions = {
        "Vec3": [(n, field.TYPE_DOUBLE, None, False) for n in ("x", "y", "z")],
        "Box": [
            ("center", field.TYPE_MESSAGE, ".Vec3", False),
            ("size", field.TYPE_MESSAGE, ".Vec3", False),
            *[(n, field.TYPE_DOUBLE, None, False) for n in ("qx", "qy", "qz", "qw")],
        ],
        "Object": [
            ("id", field.TYPE_UINT32, None, False),
            ("box", field.TYPE_MESSAGE, ".Box", False),
            ("unified_class", field.TYPE_UINT32, None, False),
            ("source", field.TYPE_UINT32, None, False),
            ("raw_score", field.TYPE_DOUBLE, None, False),
            ("class_conflict", field.TYPE_BOOL, None, False),
            ("classification_use", field.TYPE_UINT32, None, False),
            ("orientation_reliable", field.TYPE_BOOL, None, False),
            ("point_id", field.TYPE_UINT32, None, True),
            ("reason", field.TYPE_UINT32, None, True),
        ],
    }
    for name, fields in definitions.items():
        message = schema.message_type.add(name=name)
        for number, (name, kind, message_type, repeated) in enumerate(fields, 1):
            entry = message.field.add(
                name=name, number=number, type=kind, label=3 if repeated else 1
            )
            if message_type:
                entry.type_name = message_type
    pool = descriptor_pool.DescriptorPool()
    pool.Add(schema)
    obj = message_factory.GetMessageClass(pool.FindMessageTypeByName("Object"))()
    obj.box.size.x, obj.box.size.y, obj.box.size.z = 4, 2, 1.5
    obj.box.qw = 1
    obj.source = 2
    obj.classification_use = 1
    obj.point_id.extend([3, 5])
    return obj


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

    def test_absent_display_score_retains_historical_matching_order(self):
        legacy = [candidate(score=1.0), candidate(1, score=0.9)]
        displayed = [{**legacy[0], "score": None, "ranking_score": 1.0}, legacy[1]]
        self.assertEqual(match(displayed, [candidate()]), match(legacy, [candidate()]))


class ProjectionTest(unittest.TestCase):
    def test_unknown_candidate_preserves_absence_support_and_orientation(self):
        obj = projection_object()
        obj.unified_class = 0
        item = object_view(obj, {"Source": {2: "SEG_RECALL"}, "Reason": {}})
        self.assertEqual(item["class"], "unknown")
        self.assertIsNone(item["score"])
        self.assertFalse(item["score_present"])
        self.assertEqual(item["ranking_score"], 1.0)
        self.assertFalse(item["orientation_reliable"])
        self.assertEqual(item["evidence"]["point_id"], [3, 5])
        self.assertNotIn("raw_score", item["evidence"])
        self.assertNotIn("orientation_reliable", item["evidence"])
        self.assertEqual(item["quaternion"], [0, 0, 0, 1])

    def test_present_zero_score_and_other_vehicle_are_not_replaced(self):
        obj = projection_object()
        obj.unified_class = 7
        obj.raw_score = 0
        obj.orientation_reliable = True
        item = object_view(obj, {"Source": {2: "SEG_RECALL"}, "Reason": {}})
        self.assertEqual(item["class"], "other_vehicle")
        self.assertTrue(item["score_present"])
        self.assertEqual(item["score"], 0)
        self.assertEqual(item["ranking_score"], 0)
        self.assertTrue(item["orientation_reliable"])
        self.assertIn("raw_score", item["evidence"])


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
