"""Explicit opt-in GPU/ABI tests against a target-built KITTI engine."""

import ctypes
import os
import unittest

from run_pointpillars import Detection, load_runtime, read_points


@unittest.skipUnless(
    os.getenv("POINTPILLARS_RUNTIME") and os.getenv("POINTPILLARS_ENGINE"),
    "Set POINTPILLARS_RUNTIME and POINTPILLARS_ENGINE for GPU validation",
)
class PointPillarsRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = load_runtime(os.environ["POINTPILLARS_RUNTIME"])
        error = ctypes.create_string_buffer(4096)
        cls.handle = cls.library.pointpillars_runtime_create(
            os.environ["POINTPILLARS_ENGINE"].encode(), 0, error, len(error)
        )
        if not cls.handle:
            raise RuntimeError(error.value.decode())

    @classmethod
    def tearDownClass(cls):
        cls.library.pointpillars_runtime_destroy(cls.handle)

    def infer(self, points):
        output = (Detection * 40000)()
        count = ctypes.c_size_t(999)
        error = ctypes.create_string_buffer(4096)
        ok = self.library.pointpillars_runtime_infer(
            self.handle, points, len(points) // 4, output, len(output),
            ctypes.byref(count), error, len(error),
        )
        return ok, count.value, error.value.decode()

    def test_oversize_pillar_capacity_is_explicit(self):
        # The pinned ONNX accepts 10000 pillars, not params.h's 40000.
        values = []
        for i in range(10001):
            values.extend(
                ((i % 432) * 0.16 + 0.08, (i // 432) * 0.16 - 39.60, -1, 0.5)
            )
        ok, count, error = self.infer((ctypes.c_float * len(values))(*values))
        self.assertFalse(ok)
        self.assertEqual(count, 0)
        self.assertIn("capacity", error)

    def test_repeated_valid_frames_after_rejected_input(self):
        sample = os.getenv("POINTPILLARS_SAMPLE")
        points = (
            read_points(sample)
            if sample
            else (ctypes.c_float * 4)(2, 0, -1, 0.5)
        )
        for _ in range(3):
            ok, count, error = self.infer(points)
            self.assertTrue(ok, error)
            self.assertLessEqual(count, 40000)
        ok, count, error = self.infer(
            (ctypes.c_float * 4)(float("nan"), 0, -1, 0.5)
        )
        self.assertFalse(ok)
        self.assertEqual(count, 0)
        self.assertIn("Nonfinite", error)


if __name__ == "__main__":
    unittest.main()
