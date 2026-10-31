"""Runtime layout contracts independent of the selected hardware profile."""

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from materialize_runtime import materialize, split_package


class RuntimeLayoutTest(unittest.TestCase):
    def test_package_mapping_restores_executable_and_data_modes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            for name in (
                "bin/mainboard",
                "modules/camera.dag",
                "hardware/manifest.json",
                "lib/libcamera.so",
            ):
                source = runtime / name
                source.parent.mkdir(parents=True, exist_ok=True)
                source.write_bytes(b"fixture")
                source.chmod(0o555)
            output = root / "package"
            split_package(runtime, output)
            self.assertEqual((output / "bin/mainboard").stat().st_mode & 0o777, 0o755)
            self.assertEqual(
                (output / "modules/camera.dag").stat().st_mode & 0o777, 0o644
            )
            self.assertEqual((output / "lib/libcamera.so").read_bytes(), b"fixture")

    def test_package_mapping_rejects_unmapped_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runtime = root / "runtime"
            runtime.mkdir()
            (runtime / "unmapped.txt").write_text("must not be silently omitted")
            with self.assertRaisesRegex(
                ValueError, "Unknown runtime package directory"
            ):
                split_package(runtime, root / "package")

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

    def test_runtime_entry_preserves_exit_arguments_and_isolates_libraries(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mainboard = root / "mainboard"
            mainboard.write_text(
                "#!/usr/bin/env python3\n"
                "import json, os, sys\n"
                "print(json.dumps([sys.argv[1:], os.getcwd(), "
                "os.environ['CYBER_PATH'], os.environ['LD_LIBRARY_PATH']]))\n"
                "sys.exit(13)\n"
            )
            dag = root / "test.dag"
            dag.write_text("")
            output = root / "runtime"
            materialize(
                {
                    "files": [
                        [str(mainboard), "bin/mainboard"],
                        [
                            str(Path(__file__).with_name("runtime_entry.py")),
                            "bin/runtime_entry.py",
                        ],
                        [str(dag), "modules/test.dag"],
                    ],
                    "libraries": [],
                    "default_dag": "modules/test.dag",
                },
                output,
            )
            environment = dict(os.environ, LD_LIBRARY_PATH="/old/build-cache")
            environment.pop("WHEELOS_RUNTIME_LIBRARY_PATH", None)
            for extension in ("", "/explicit/sdk"):
                environment["WHEELOS_RUNTIME_LIBRARY_PATH"] = extension
                result = subprocess.run(
                    [str(output / "bin/run"), "--", "-p", "process with spaces"],
                    env=environment,
                    capture_output=True,
                    text=True,
                    check=False,
                )
                self.assertEqual(result.returncode, 13, result.stderr)
                arguments, cwd, cyber_path, libraries = json.loads(result.stdout)
                self.assertEqual(
                    arguments,
                    [
                        "-d",
                        str(output / "modules/test.dag"),
                        "-p",
                        "process with spaces",
                    ],
                )
                self.assertEqual(cwd, str(output))
                self.assertEqual(cyber_path, str(output))
                self.assertEqual(
                    libraries,
                    str(output / "lib") + (":" + extension if extension else ""),
                )

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
