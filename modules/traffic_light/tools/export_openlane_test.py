#!/usr/bin/env python3
"""Hermetic standard-library regressions for the diagnostic export contract."""

import argparse
import contextlib
import copy
import io
import json
import pathlib
import unittest
from unittest import mock

import export_openlane


class ExportOpenLaneTest(unittest.TestCase):
    def setUp(self):
        self.args = argparse.Namespace(
            root=pathlib.Path("openlane-test-fixture"),
            output=None,
            local_prefix_diagnostic=True,
            exclude_ambiguous_ids=False,
            clip_partial_boxes=False,
            verify_images=False,
        )
        self.element = {
            "id": "tl_00",
            "category": 1,
            "attribute": 1,
            "points": [[10, 20], [20, 40]],
        }
        self.annotation = {
            "lane_centerline": [{"id": "lane_0", "points": [[0, 0, 0], [1, 0, 0]]}],
            "traffic_element": [self.element],
            "topology_lclc": [[0]],
            "topology_lcte": [[1]],
        }

    def run_export(self, mutation=None):
        def annotation_text(path, *args, **kwargs):
            segment = path.parent.parent.name
            data = {
                "version": "OpenLaneV2_V0.1",
                "segment_id": segment,
                "timestamp": 1,
                "sensor": {
                    "ring_front_center": {
                        "image_path": f"train/{segment}/image/front.jpg"
                    }
                },
                "annotation": copy.deepcopy(self.annotation),
            }
            if mutation:
                mutation(data)
            return json.dumps(data)

        out, err = io.StringIO(), io.StringIO()
        with (
            mock.patch.object(
                pathlib.Path,
                "glob",
                autospec=True,
                side_effect=lambda path, pattern: iter([path / "1.json"]),
            ),
            mock.patch.object(
                pathlib.Path, "read_text", autospec=True, side_effect=annotation_text
            ),
            mock.patch.object(export_openlane, "dimensions", return_value=(100, 80)),
            contextlib.redirect_stdout(out),
            contextlib.redirect_stderr(err),
        ):
            export_openlane.export(self.args)
        return out.getvalue(), json.loads(err.getvalue())

    def test_positive_deterministic_export(self):
        first, audit = self.run_export()
        second, second_audit = self.run_export()
        self.assertEqual((first, audit), (second, second_audit))
        self.assertIn("G\ttl_00\t10\t20\t10\t20\tred\n", first)
        self.assertEqual(audit["frames"], 2)
        self.assertEqual(audit["observations"], 2)
        self.assertEqual(audit["topology_lcte_edges"], 2)

    def test_mapping_requires_explicit_opt_in_and_exact_tuple(self):
        self.args.local_prefix_diagnostic = False
        with self.assertRaisesRegex(ValueError, "explicitly opt"):
            self.run_export()
        self.args.local_prefix_diagnostic = True
        self.element["category"] = 0
        with self.assertRaisesRegex(ValueError, "mapping"):
            self.run_export()
        self.element["attribute"] = 4
        text, _ = self.run_export()
        self.assertIn("\tunknown\n", text)

    def test_duplicate_identity_policy_preserves_boxes_without_inventing_ids(self):
        self.annotation["traffic_element"].append(copy.deepcopy(self.element))
        self.annotation["topology_lcte"] = [[1, 1]]
        with self.assertRaisesRegex(ValueError, "duplicate evaluation IDs"):
            self.run_export()
        self.args.exclude_ambiguous_ids = True
        text, audit = self.run_export()
        self.assertEqual(text.count("G\t-\t"), 4)
        self.assertEqual(audit["observations"], 4)
        self.assertEqual(audit["ambiguous_identity_observations"], 4)

    def test_clipping_is_explicit_and_never_rescues_degenerate_boxes(self):
        self.element["points"][0][0] = -1
        with self.assertRaisesRegex(ValueError, "outside image"):
            self.run_export()
        self.args.clip_partial_boxes = True
        text, audit = self.run_export()
        self.assertIn("G\ttl_00\t0\t20\t20\t20\tred\n", text)
        self.assertEqual(audit["clipped_boxes"], 2)
        for points in ([[110, 20], [120, 40]], [[10, 20], [10, 40]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                self.element["points"] = points
                self.run_export()

    def test_malformed_schema_topology_and_version_rejected(self):
        mutations = (
            lambda data: data.update(version="OpenLaneV2_V1.1"),
            lambda data: data["annotation"].update(topology_lcte=[[1, 0]]),
            lambda data: data["annotation"].update(topology_lclc=[[2]]),
            lambda data: data["annotation"].pop("traffic_element"),
        )
        for mutation in mutations:
            with (
                self.subTest(mutation=mutation),
                self.assertRaises((ValueError, KeyError)),
            ):
                self.run_export(mutation)

    def test_nonfinite_or_malformed_geometry_rejected(self):
        for points in (
            [[float("nan"), 20], [20, 40]],
            [[10, 20], [float("inf"), 40]],
            [[10, 20, 0], [20, 40]],
        ):
            with (
                self.subTest(points=points),
                self.assertRaisesRegex(ValueError, "geometry"),
            ):
                self.element["points"] = points
                self.run_export()


if __name__ == "__main__":
    unittest.main()
