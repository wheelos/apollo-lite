"""Exercise the customer CLI and ensure it never falls back to global builds."""

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


class ProductBuildTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        self.capture = self.path / "bazel.jsonl"
        self.probes = self.path / "probes"
        bazel = self.path / "bazel"
        bazel.write_text(
            "#!/usr/bin/env python3\n"
            "import json, os, sys\n"
            "with open(os.environ['BAZEL_ARGUMENTS'], 'a') as stream:\n"
            "    stream.write(json.dumps(sys.argv[1:]) + '\\n')\n"
            "if sys.argv[1] == 'cquery':\n"
            "    print('Analyzed preview')\n"
            "    sys.exit(int(os.environ.get('PREVIEW_EXIT', '0')))\n"
        )
        bazel.chmod(0o755)
        for name in ["nvidia-smi", "lsmod", "ldconfig", "free"]:
            probe = self.path / name
            probe.write_text(
                "#!/usr/bin/env bash\n"
                f'echo {name} >> "${{PROBE_LOG}}"\n'
                f"echo unexpected-probe-{name} >&2\n"
                "exit 99\n"
            )
            probe.chmod(0o755)
        self.env = dict(
            os.environ,
            PATH=f"{self.path}:{os.environ['PATH']}",
            BAZEL_ARGUMENTS=str(self.capture),
            BAZEL_CACHE_DIR=str(self.path / "cache"),
            PROBE_LOG=str(self.probes),
            USE_GPU_TARGET="1",
        )

    def run_cli(self, *args, code=0):
        result = subprocess.run(
            ["bash", "apollo.sh", *args],
            cwd=ROOT,
            env=self.env,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, code, result.stdout + result.stderr)
        self.assertNotIn("unexpected-probe", result.stderr)
        self.assertFalse(self.probes.exists(), "must not probe the build host")
        calls = (
            [json.loads(line) for line in self.capture.read_text().splitlines()]
            if self.capture.exists()
            else []
        )
        return result, calls

    def test_help_profiles_and_config_do_not_probe_build_environment(self):
        for command in ["--help", "profiles", "config"]:
            with self.subTest(command=command):
                result, calls = self.run_cli(command)
                self.assertFalse(calls)
                self.assertNotIn("Apollo Environment Settings", result.stderr)
        self.assertIn(".bazelrc.user", result.stderr)

    def test_product_previews_then_builds_selected_roots(self):
        _, calls = self.run_cli(
            "build", "--product=devkit-x86", "--mode=dbg", "--jobs=2"
        )
        self.assertEqual([call[0] for call in calls], ["cquery", "build"])
        self.assertIn("--config=product_devkit-x86", calls[1])
        self.assertIn("--config=dbg", calls[1])
        self.assertIn("--jobs=2", calls[1])
        self.assertNotIn("--config=gpu", calls[1])
        self.assertEqual(
            calls[1][calls[1].index("--") + 1 :],
            ["//tools/hardware:hardware", "//tools/modules:algorithms"],
        )

    def test_show_only_analyzes(self):
        _, calls = self.run_cli("build", "--product=devkit-x86", "--show")
        self.assertEqual([call[0] for call in calls], ["cquery"])

    def test_build_help_does_not_probe_environment(self):
        _, calls = self.run_cli("build", "--help")
        self.assertFalse(calls)

    def test_build_phase_skips_runtime_gpu_setup_but_runtime_source_retains_it(self):
        for phase in ["1", None]:
            env = dict(self.env)
            env.pop("APOLLO_BUILD_PHASE", None)
            if phase:
                env["APOLLO_BUILD_PHASE"] = phase
            result = subprocess.run(
                ["bash", "-c", "source scripts/apollo.bashrc"],
                cwd=ROOT,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual("unexpected-probe" in result.stderr, phase is None)

    def test_product_test_is_explicitly_unsupported_without_environment_probes(self):
        result, calls = self.run_cli("test", "--product=devkit-x86", code=2)
        self.assertIn("product testing", result.stderr)
        self.assertFalse(calls)

    def test_legacy_mode_aliases_preserve_configs_and_recursive_targets(self):
        for name, output in {
            "nvidia-smi": "GPU 0",
            "lsmod": "nvgpu",
            "ldconfig": "libcudart",
            "free": "Mem: 4096",
        }.items():
            (self.path / name).write_text(f"#!/bin/sh\nprintf '%s\\n' '{output}'\n")
        for suffix in [
            "opt",
            "dbg",
            "cpu",
            "gpu",
            "opt_cpu",
            "dbg_cpu",
            "opt_gpu",
            "dbg_gpu",
            "prof",
            "teleop",
        ]:
            with self.subTest(suffix=suffix):
                self.capture.unlink(missing_ok=True)
                _, calls = self.run_cli(f"build_{suffix}", "control", "--jobs=2")
                self.assertEqual([call[0] for call in calls], ["build"])
                for mode in suffix.split("_"):
                    self.assertIn(f"--config={mode}", calls[0])
                self.assertIn("--jobs=2", calls[0])
                self.assertIn("//modules/control/...", calls[0])
                self.assertNotIn("//tools/modules:algorithms", calls[0])

    def test_custom_build_normalizes_existing_flags(self):
        _, calls = self.run_cli(
            "build",
            "--modules=rtk_localization,control",
            "--compute=cpu",
            "--hardware=hw_x86_devkit",
        )
        self.assertIn("--config=cpu", calls[-1])
        self.assertIn("--config=opt", calls[-1])
        self.assertIn("--config=hw_x86_devkit", calls[-1])
        self.assertIn("--//tools/modules:include=rtk_localization,control", calls[-1])

    def test_failed_analysis_never_starts_compilation(self):
        self.env["PREVIEW_EXIT"] = "7"
        _, calls = self.run_cli("build", "--product=devkit-x86", code=7)
        self.assertEqual([call[0] for call in calls], ["cquery"])

    def test_invalid_combinations_never_reach_bazel(self):
        cases = [
            ["--product=unknown"],
            ["--show"],
            ["--product=devkit-x86", "--modules=control"],
            ["--product=devkit-x86", "--compute=gpu"],
            ["--modules=control"],
            ["--modules=", "--compute=cpu"],
            ["--all", "--product=devkit-x86"],
            ["--all", "--show"],
            ["--product=devkit-x86", "--jobs=0"],
            ["--product=devkit-x86", "--config=gpu"],
            ["--product=devkit-x86", "planning"],
        ]
        for options in cases:
            with self.subTest(options=options):
                _, calls = self.run_cli("build", *options, code=2)
                self.assertFalse(calls)

    def test_explicit_all_is_the_only_customer_global_build(self):
        _, calls = self.run_cli("build", "--all")
        self.assertEqual([call[0] for call in calls], ["build"])
        self.assertIn("//modules/...", calls[0])


if __name__ == "__main__":
    unittest.main()
