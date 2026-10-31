"""Exercise the actual build wrapper without compiling Apollo."""

import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


class LintWrapperTest(unittest.TestCase):
    def test_cpp_lint_runs_standalone_and_propagates_failure(self):
        for exit_code in (0, 1):
            with (
                self.subTest(exit_code=exit_code),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                capture = root / "arguments.json"
                cpp = "pkgs/hardware/camera/camera_component.cc"
                git = root / "git"
                git.write_text(
                    "#!/usr/bin/env python3\n"
                    "import os, sys\n"
                    "if sys.argv[1] == 'diff':\n"
                    f"    print({cpp!r})\n"
                    "else:\n"
                    f"    os.execv({shutil.which('git')!r}, ['git', *sys.argv[1:]])\n"
                )
                git.chmod(0o755)
                cpplint = root / "cpplint"
                cpplint.write_text(
                    "#!/usr/bin/env python3\n"
                    "import json, os, sys\n"
                    "with open(os.environ['LINT_ARGUMENTS'], 'w') as stream:\n"
                    "    json.dump(sys.argv[1:], stream)\n"
                    f"sys.exit({exit_code})\n"
                )
                cpplint.chmod(0o755)
                bazel = root / "bazel"
                bazel.write_text(
                    "#!/bin/sh\n" f"touch {str(root / 'bazel-invoked')!r}\n" "exit 1\n"
                )
                bazel.chmod(0o755)
                env = dict(os.environ)
                env.update(
                    PATH=f"{root}:{env['PATH']}",
                    LINT_ARGUMENTS=str(capture),
                    USE_GPU_TARGET="0",
                )
                result = subprocess.run(
                    [
                        "bash",
                        "scripts/ci/apollo_lint.sh",
                        "--cpp-lint",
                        "--diff",
                        "HEAD",
                    ],
                    cwd=ROOT,
                    env=env,
                    capture_output=True,
                    text=True,
                    check=False,
                )
                self.assertEqual(
                    result.returncode, exit_code, result.stdout + result.stderr
                )
                self.assertEqual(
                    json.loads(capture.read_text()),
                    [
                        "--extensions=c,cc,cpp,h,hpp",
                        "--repository=pkgs/hardware/camera",
                        cpp,
                    ],
                )
                self.assertFalse((root / "bazel-invoked").exists())


class ReleaseWrapperTest(unittest.TestCase):
    def run_release(self, *args):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root / "arguments.jsonl"
            bazel = root / "bazel"
            bazel.write_text(
                "#!/usr/bin/env python3\n"
                "import json, os, sys\n"
                "if sys.argv[1:] == ['--version']:\n"
                f"    print('bazel {(ROOT / '.bazelversion').read_text().strip()}')\n"
                "    sys.exit(0)\n"
                "args = sys.argv[1:]\n"
                "args = args[args.index('cquery' if 'cquery' in args else 'run'):]\n"
                "with open(os.environ['BAZEL_ARGUMENTS'], 'a') as stream:\n"
                "    stream.write(json.dumps(args) + '\\n')\n"
            )
            bazel.chmod(0o755)
            (root / "bazelisk").symlink_to(bazel)
            (root / ".bashrc").write_text("")
            env = dict(os.environ)
            env.update(
                PATH=f"{root}:{env['PATH']}",
                HOME=str(root),
                BAZEL_ARGUMENTS=str(capture),
                USE_GPU_TARGET="0",
            )
            result = subprocess.run(
                [
                    "bash",
                    "scripts/ci/apollo_release.sh",
                    "--cpu",
                    "--prefix",
                    str(root / "output"),
                    *args,
                ],
                cwd=ROOT,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return [json.loads(line) for line in capture.read_text().splitlines()]

    def test_release_uses_official_installer_arguments_and_cleans_only_once(self):
        calls = self.run_release("hardware", "transform", "--clean")
        self.assertEqual(len(calls), 2)
        for call, module in zip(calls, ("hardware", "transform")):
            self.assertIn("--config=dev", call)
            self.assertIn(f"//tools/{module}:install_dir", call)
            self.assertEqual(call[call.index("--") + 1], "--destdir")
            self.assertNotIn("--pre_clean", call)
            self.assertNotIn("--gpu", call)
        self.assertIn("--wipe_destdir", calls[0])
        self.assertNotIn("--wipe_destdir", calls[1])

    def test_release_listing_uses_package_providers_without_installing(self):
        calls = self.run_release("control", "--list")
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][0], "cquery")
        self.assertIn("//modules/control:package_files", calls[0])
        self.assertNotIn("run", calls[0])

    def test_script_installers_are_run_separately(self):
        calls = self.run_release("scripts", "--list")
        self.assertEqual(len(calls), 2)
        self.assertIn("//docker/scripts:package_files", calls[0])
        self.assertIn("//scripts:package_files", calls[1])


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
            "--@hardware.build//tools/hardware:gnss=novatel",
            "--define",
            "vehicle=devkit",
            "--profile=/tmp/profile with spaces.gz",
        )
        self.assertIn("--@hardware.build//tools/hardware:gnss=novatel", args)
        self.assertIn("--profile=/tmp/profile with spaces.gz", args)
        self.assertEqual(args[args.index("--define") + 1], "vehicle=devkit")
        self.assertEqual(args[args.index("--") + 1 :], ["//tools/hardware:hardware"])

    def test_package_uses_selected_root(self):
        args = self.run_wrapper("hardware_package", "--config", "hw_x86_devkit")
        self.assertEqual(args[args.index("--") + 1 :], ["//tools/hardware:package"])

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
            "--@hardware.build//tools/hardware:gnss=huace",
            "--jobs=4",
            command="test",
        )
        self.assertIn("--@hardware.build//tools/hardware:gnss=huace", args)
        self.assertEqual(args[args.index("--") + 1 :], ["//tools/hardware:tests"])

    def test_legacy_module_remains_recursive(self):
        args = self.run_wrapper("canbus")
        self.assertIn("@hardware.vehicle//...", args)

    def test_extracted_module_shorthands_route_builds_and_tests(self):
        for component, targets in [
            (
                "drivers",
                [
                    f"@hardware.{name}//..."
                    for name in (
                        "camera",
                        "camera_gst",
                        "image_decompress",
                        "hal",
                        "canbus",
                        "lidar",
                        "radar",
                        "gnss",
                        "audio",
                        "video",
                    )
                ],
            ),
            ("canbus", ["@hardware.vehicle//..."]),
            ("transform", ["@wheelos_transform//modules/transform/..."]),
        ]:
            for command in ["build", "test"]:
                with self.subTest(component=component, command=command):
                    args = self.run_wrapper(component, command=command)
                    for target in targets:
                        self.assertIn(target, args)

    def test_direct_label_is_not_interpreted_as_directory(self):
        args = self.run_wrapper("@hardware.vehicle//:component")
        self.assertEqual(
            args[args.index("--") + 1 :],
            ["@hardware.vehicle//:component"],
        )


if __name__ == "__main__":
    unittest.main()
