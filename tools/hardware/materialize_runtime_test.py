"""Runtime layout contracts independent of the selected hardware profile."""

import tempfile
import unittest
from pathlib import Path

from materialize_runtime import materialize


class RuntimeLayoutTest(unittest.TestCase):
    def test_paths_and_launch_environment_are_relocatable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            dag = root / "camera.dag"
            dag.write_text(
                'module_library: "/apollo/bazel-bin/modules/drivers/camera/'
                'libcamera.so"\n'
                'config_file_path: "/apollo/modules/drivers/camera/conf/'
                'config.pb.txt"\n'
            )
            library = root / "libcamera.so"
            library.write_bytes(b"fixture")
            output = root / "runtime"
            materialize(
                {
                    "files": [
                        [str(dag), "modules/drivers/camera/dag/camera.dag"],
                        [str(library), "bazel-bin/modules/drivers/camera/libcamera.so"],
                    ],
                    "libraries": [[str(library), "lib/libcamera.so"]],
                },
                output,
            )
            text = (output / "modules/drivers/camera/dag/camera.dag").read_text()
            self.assertNotIn("/apollo/", text)
            self.assertIn('"bazel-bin/modules/drivers/camera/libcamera.so"', text)
            launcher = output / "bin/mainboard-hardware"
            self.assertIn('export CYBER_PATH="${root}"', launcher.read_text())
            self.assertTrue(launcher.stat().st_mode & 0o111)

    def test_conflicting_libraries_fail_instead_of_overwriting(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            one, two = root / "one.so", root / "two.so"
            one.write_bytes(b"one")
            two.write_bytes(b"two")
            with self.assertRaisesRegex(ValueError, "Conflicting runtime files"):
                materialize(
                    {
                        "files": [],
                        "libraries": [
                            [str(one), "lib/libshared.so"],
                            [str(two), "lib/libshared.so"],
                        ],
                    },
                    root / "runtime",
                )

    def test_destination_cannot_escape_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "Invalid runtime destination"):
                materialize(
                    {"files": [["unused", "../escape"]], "libraries": []},
                    Path(directory) / "runtime",
                )


if __name__ == "__main__":
    unittest.main()
