"""Stable commands select one runtime without mutating the build environment."""

import importlib.util
import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "runtime_tool", ROOT / "scripts/runtime/runtime_tool.py"
)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)
spec = importlib.util.spec_from_file_location(
    "materializer", ROOT / "tools/hardware/materialize_runtime.py"
)
materializer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(materializer)


class RuntimeCommandsTest(unittest.TestCase):
    def test_container_environment_forwards_explicit_runtime_selection(self):
        with tempfile.TemporaryDirectory() as directory:
            environment = dict(
                os.environ,
                WHEELOS_RUNTIME_PRODUCT="transform",
                WHEELOS_RUNTIME_ROOT="",
                BAZEL_CACHE_DIR=directory,
            )
            subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1/docker/scripts/env_setup.sh"; '
                    'generate_env dev "$1" "$2" test-image',
                    "test",
                    str(ROOT),
                    directory,
                ],
                env=environment,
                capture_output=True,
                text=True,
                check=True,
            )
            generated = (Path(directory) / ".env.dev.local").read_text()
            self.assertIn("WHEELOS_RUNTIME_PRODUCT=transform\n", generated)
            self.assertIn("WHEELOS_RUNTIME_ROOT=\n", generated)

    def test_selected_shell_does_not_inject_build_cache_libraries(self):
        environment = dict(
            os.environ,
            WHEELOS_RUNTIME_PRODUCT="transform",
            LD_LIBRARY_PATH="/existing/sdk",
            APOLLO_RUNTIME_ENV_LOADED="0",
        )
        environment.pop("WHEELOS_RUNTIME_ROOT", None)
        result = subprocess.run(
            [
                "bash",
                "-c",
                'source "$1/setup.bash"; source "$1/setup.bash"; '
                "command -v mainboard; command -v cyber_launch; "
                "command -v cyber_recorder; command -v cyber_monitor; "
                'printf "%s\\n" "$LD_LIBRARY_PATH" "$APOLLO_ROOT_DIR"',
                "test",
                str(ROOT),
            ],
            env=environment,
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertEqual(
            result.stdout.splitlines(),
            [
                str(ROOT / "scripts/runtime/mainboard"),
                str(ROOT / "scripts/runtime/cyber_launch"),
                str(ROOT / "scripts/runtime/cyber_recorder"),
                str(ROOT / "scripts/runtime/cyber_monitor"),
                "/existing/sdk",
                str(ROOT),
            ],
        )

    def test_container_nonlogin_shell_loads_selected_commands(self):
        bootstrap = (ROOT / "docker/scripts/entrypoint.sh").read_text()
        marker = "cat >> \"${USER_HOME}/.bashrc\" <<'EOF'\n"
        rc_text = bootstrap.split(marker, 1)[1].split("\nEOF", 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            rc = Path(directory) / ".bashrc"
            rc.write_text(rc_text)
            environment = dict(
                os.environ,
                WHEELOS_RUNTIME_PRODUCT="transform",
                WHEELOS_RUNTIME_ROOT="",
                APOLLO_RUNTIME_ENV_LOADED="0",
            )
            result = subprocess.run(
                [
                    "bash",
                    "--noprofile",
                    "--rcfile",
                    str(rc),
                    "-ic",
                    "command -v mainboard; command -v cyber_launch",
                ],
                env=environment,
                capture_output=True,
                text=True,
                check=True,
            )
            self.assertEqual(
                result.stdout.splitlines(),
                [
                    str(ROOT / "scripts/runtime/mainboard"),
                    str(ROOT / "scripts/runtime/cyber_launch"),
                ],
            )

    def test_invalid_or_missing_selection_never_falls_back(self):
        for environment in (
            {},
            {"WHEELOS_RUNTIME_PRODUCT": "../transform"},
            {"WHEELOS_RUNTIME_PRODUCT": "missing"},
            {"WHEELOS_RUNTIME_ROOT": "relative"},
            {
                "WHEELOS_RUNTIME_ROOT": "/tmp/missing",
                "WHEELOS_RUNTIME_PRODUCT": "transform",
            },
        ):
            with self.subTest(environment=environment), self.assertRaises(ValueError):
                tool.command(ROOT, environment, "mainboard", ["-d", "test.dag"])

    def test_relocated_legacy_commands_isolate_environment_and_preserve_exit(self):
        with tempfile.TemporaryDirectory(prefix="runtime with spaces ") as directory:
            root = Path(directory)
            fake = root / "fake.py"
            fake.write_text(
                "#!/usr/bin/env python3\n"
                "import json, os, sys\n"
                "print(json.dumps([sys.argv[1:], os.getcwd(), "
                "os.environ['CYBER_PATH'], os.environ['LD_LIBRARY_PATH'], "
                "os.getenv('GLOG_logtostderr')]))\n"
                "sys.exit(17)\n"
            )
            runtime = root / "runtime"
            materializer.materialize(
                {
                    "files": [
                        [str(fake), "bin/mainboard"],
                        [str(fake), "bin/cyber_launch.py"],
                        [str(fake), "bin/cyber_recorder"],
                        [str(fake), "bin/cyber_monitor"],
                    ],
                    "libraries": [],
                },
                runtime,
            )
            environment = dict(
                os.environ,
                WHEELOS_RUNTIME_ROOT=str(runtime),
                LD_LIBRARY_PATH="/stale/cache",
                WHEELOS_RUNTIME_LIBRARY_PATH="/explicit/sdk",
            )
            environment.pop("WHEELOS_RUNTIME_PRODUCT", None)
            environment.pop("GLOG_log_dir", None)
            environment.pop("GLOG_logtostderr", None)
            for name, arguments in (
                ("mainboard", ["-d", "dag with spaces.dag"]),
                ("cyber_launch", ["start", "launch with spaces.launch"]),
                ("cyber_recorder", ["info", "record with spaces.record"]),
                ("cyber_monitor", ["-h"]),
            ):
                with self.subTest(name=name):
                    result = subprocess.run(
                        [str(ROOT / "scripts/runtime" / name), *arguments],
                        env=environment,
                        capture_output=True,
                        text=True,
                        check=False,
                    )
                    self.assertEqual(result.returncode, 17, result.stderr)
                    self.assertEqual(
                        json.loads(result.stdout),
                        [
                            arguments,
                            (
                                str(Path.cwd())
                                if name in ("cyber_recorder", "cyber_monitor")
                                else str(runtime)
                            ),
                            str(runtime),
                            str(runtime / "lib") + ":/explicit/sdk",
                            "1",
                        ],
                    )
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    'source "$1/bin/setup.bash"; mainboard -d test.dag',
                    "test",
                    str(runtime),
                ],
                env=environment,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 17, result.stderr)
            self.assertEqual(json.loads(result.stdout)[0], ["-d", "test.dag"])
            result = subprocess.run(
                [str(runtime / "bin/commands/mainboard"), "-d", "test.dag"],
                env=dict(environment, GLOG_log_dir=directory, GLOG_logtostderr="0"),
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 17, result.stderr)
            self.assertEqual(json.loads(result.stdout)[-1], "0")

    def test_product_path_is_resolved_at_each_invocation(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            for name in ("opt", "dbg"):
                root = workspace / name / "tools/transform/runtime"
                materializer.materialize({"files": [], "libraries": []}, root)
            output = workspace / "bazel-bin"
            for name in ("opt", "dbg"):
                if output.is_symlink():
                    output.unlink()
                output.symlink_to(workspace / name, target_is_directory=True)
                argv = tool.command(
                    workspace,
                    {"WHEELOS_RUNTIME_PRODUCT": "transform"},
                    "mainboard",
                    ["-d", "test.dag"],
                )
                self.assertEqual(
                    Path(argv[0]).resolve(),
                    workspace / name / "tools/transform/runtime/bin/commands/mainboard",
                )


if __name__ == "__main__":
    unittest.main()
