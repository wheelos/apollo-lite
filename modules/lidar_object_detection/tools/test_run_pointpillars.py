"""PointPillars offline input and ABI rejection tests (no GPU required)."""

import math
import struct
import tempfile
import unittest
from pathlib import Path

from run_pointpillars import read_points


class PointPillarsInputTest(unittest.TestCase):
    def test_packed_xyzi_and_invalid_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frame.bin"
            path.write_bytes(struct.pack("<4f", 1, 2, 3, 0.5))
            self.assertEqual(list(read_points(path)), [1, 2, 3, 0.5])
            for data in (
                b"",
                b"short",
                struct.pack("<4f", 1, 2, 3, math.nan),
                b"\0" * (300001 * 16),
            ):
                path.write_bytes(data)
                with self.assertRaises(ValueError):
                    read_points(path)


if __name__ == "__main__":
    unittest.main()
