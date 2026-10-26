"""Tests for OpenLane point identity and geometric metric calculations."""

import unittest

from openlane_projection_accuracy import (
    pinhole_project,
    summarize,
    svg_bev,
    transform_camera_to_ego,
    transform_ego_to_camera,
)


class OpenLaneProjectionAccuracyTest(unittest.TestCase):
    def setUp(self):
        self.camera = {
            "extrinsic": {
                "rotation": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                "translation": [1, 0, 1.5],
            },
            "intrinsic": {
                "K": [[100, 0, 50], [0, 100, 50], [0, 0, 1]],
            },
        }

    def test_camera_extrinsic_and_pinhole_projection(self):
        ego = [11, -2, 0]
        camera = transform_ego_to_camera(ego, self.camera)
        self.assertEqual(camera, [2, 1.5, 10])
        self.assertEqual(transform_camera_to_ego(camera, self.camera), ego)
        self.assertEqual(
            pinhole_project(camera, self.camera["intrinsic"]["K"]),
            (70.0, 65.0),
        )
        self.assertIsNone(
            pinhole_project([0, 0, 0], self.camera["intrinsic"]["K"]))

    def test_metrics_keep_visibility_failures_in_coverage_denominator(self):
        rows = [
            {
                "in_gt_roi": True,
                "status": "projected",
                "error_ego_m": [1.0, -2.0, 0.5],
            },
            {
                "in_gt_roi": True,
                "status": "projection_rejected",
                "error_ego_m": None,
            },
            {
                "in_gt_roi": True,
                "status": "outside_image",
                "error_ego_m": None,
            },
            {
                "in_gt_roi": False,
                "status": "outside_gt_roi",
                "error_ego_m": None,
            },
        ]
        metrics = summarize(rows)
        self.assertEqual(metrics["gt_roi_points"], 3)
        self.assertEqual(metrics["geometrically_visible_points"], 2)
        self.assertEqual(metrics["projection_valid_points"], 1)
        self.assertAlmostEqual(metrics["projection_coverage_of_gt_roi"], 1 / 3)
        self.assertAlmostEqual(
            metrics["projection_coverage_of_geometrically_visible"], 0.5)
        self.assertEqual(metrics["x_forward_m"]["mae"], 1.0)
        self.assertEqual(metrics["y_left_m"]["mean_signed"], -2.0)

    def test_empty_error_bucket_is_null_not_zero(self):
        metrics = summarize([{
            "in_gt_roi": True,
            "status": "projection_rejected",
            "error_ego_m": None,
        }])
        self.assertIsNone(metrics["z_up_m"]["mae"])
        self.assertEqual(metrics["projection_valid_points"], 0)

    def test_bev_svg_interpolates_axis_labels(self):
        svg = svg_bev([], "frame")
        self.assertIn('x="45.0"', svg)
        self.assertIn('>15</text>', svg)
        self.assertNotIn("{x", svg)
        self.assertNotIn("{lateral}", svg)


if __name__ == "__main__":
    unittest.main()
