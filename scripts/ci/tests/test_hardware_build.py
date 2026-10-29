"""Exercise the actual build wrapper without compiling Apollo."""

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


class BuildWrapperTest(unittest.TestCase):
    def run_wrapper(self, *args, command="build", expected_error=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root / "arguments.json"
            bazel = root / "bazel"
            bazel.write_text(
                "#!/usr/bin/env python3\n"
                "import json, os, sys\n"
                "if 'EXPECTED_TEST_LD_LIBRARY_PATH' in os.environ:\n"
                "    assert os.environ.get('LD_LIBRARY_PATH', '') == "
                "os.environ['EXPECTED_TEST_LD_LIBRARY_PATH']\n"
                "with open(os.environ['BAZEL_ARGUMENTS'], 'w') as stream:\n"
                "    json.dump(sys.argv[1:], stream)\n"
            )
            bazel.chmod(0o755)
            env = dict(os.environ)
            env.update(
                PATH=f"{root}:{env['PATH']}",
                BAZEL_ARGUMENTS=str(capture),
                BAZEL_CACHE_DIR=str(root / "cache"),
                USE_GPU_TARGET="0",
            )
            if command == "test":
                env["EXPECTED_TEST_LD_LIBRARY_PATH"] = env.get("LD_LIBRARY_PATH", "")
            result = subprocess.run(
                ["bash", f"scripts/ci/apollo_{command}.sh", "--config=cpu", *args],
                cwd=ROOT,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            if expected_error is not None:
                self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
                self.assertIn(expected_error, result.stdout + result.stderr)
                self.assertFalse(
                    capture.exists(), "Bazel must not run on invalid input"
                )
                return None
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return json.loads(capture.read_text())

    def test_hardware_root_and_bazel_option_boundaries(self):
        args = self.run_wrapper(
            "hardware",
            "--config=hw_x86_devkit",
            "--//tools/hardware:gnss=novatel",
            "--define",
            "vehicle=devkit",
            "--profile=/tmp/profile with spaces.gz",
        )
        self.assertIn("--//tools/hardware:gnss=novatel", args)
        self.assertIn("--profile=/tmp/profile with spaces.gz", args)
        self.assertEqual(args[args.index("--define") + 1], "vehicle=devkit")
        self.assertEqual(args[args.index("--") + 1 :], ["//tools/hardware:hardware"])

    def test_package_uses_selected_root(self):
        args = self.run_wrapper("hardware_package", "--config", "hw_x86_devkit")
        self.assertEqual(args[args.index("--") + 1 :], ["//tools/hardware:install"])

    def test_algorithm_and_hardware_roots_can_be_combined(self):
        args = self.run_wrapper(
            "hardware",
            "algorithms",
            "--config=hw_x86_devkit",
            "--//tools/modules:include=rtk_localization,open_space_planning,control",
        )
        self.assertIn(
            "--//tools/modules:include=rtk_localization,open_space_planning,control",
            args,
        )
        self.assertEqual(
            args[args.index("--") + 1 :],
            ["//tools/hardware:hardware", "//tools/modules:algorithms"],
        )

    def test_algorithms_do_not_add_recursive_or_runtime_tool_roots(self):
        args = self.run_wrapper("algorithms", "--config=algo_ndt_planning")
        self.assertEqual(args[args.index("--") + 1 :], ["//tools/modules:algorithms"])

    def test_explicit_bazel_resources_override_wrapper_defaults(self):
        args = self.run_wrapper(
            "algorithms",
            "--config=algo_rtk_open_space",
            "--local_resources=cpu=1",
        )
        resources = [arg for arg in args if arg.startswith("--local_resources=cpu=")]
        self.assertEqual(resources[-1], "--local_resources=cpu=1")

    def test_resource_aliases_preserve_explicit_option_order(self):
        args = self.run_wrapper(
            "algorithms",
            "--config=algo_rtk_open_space",
            "--jobs=3",
            "--cpus",
            "2",
            "--rams=1024",
        )
        explicit = args[
            args.index("--config=algo_rtk_open_space") + 1 : args.index("--")
        ]
        self.assertEqual(
            [
                arg
                for arg in explicit
                if arg.startswith(("--jobs=", "--local_resources="))
            ],
            [
                "--jobs=3",
                "--local_resources=cpu=2",
                "--local_resources=memory=1024",
            ],
        )

    def test_missing_bazel_option_values_fail_before_bazel(self):
        for command in ["build", "test"]:
            for option in ["--config", "-c", "--define", "--platforms"]:
                with self.subTest(command=command, option=option):
                    self.run_wrapper(
                        "hardware",
                        option,
                        command=command,
                        expected_error=f"Missing value for {option}",
                    )
        for option in ["--jobs", "--cpus", "--rams"]:
            with self.subTest(option=option):
                self.run_wrapper(
                    "hardware", option, expected_error=f"Missing value for {option}"
                )

    def test_selected_tests_use_profile_suite(self):
        args = self.run_wrapper(
            "hardware",
            "--config=hw_x86_devkit",
            "--//tools/hardware:gnss=huace",
            "--jobs=4",
            command="test",
        )
        self.assertIn("--//tools/hardware:gnss=huace", args)
        self.assertEqual(args[args.index("--") + 1 :], ["//tools/hardware:tests"])

    def test_legacy_module_remains_recursive(self):
        args = self.run_wrapper("canbus")
        self.assertIn("//modules/canbus/...", args)

    def test_direct_label_is_not_interpreted_as_directory(self):
        args = self.run_wrapper("//modules/canbus:libcanbus_component.so")
        self.assertEqual(
            args[args.index("--") + 1 :], ["//modules/canbus:libcanbus_component.so"]
        )


if __name__ == "__main__":
    unittest.main()
