"""Tests for the host-local Bazel remote cache launcher."""

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/start_bazel_remote.sh"


class BazelRemoteLauncherTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.log = self.root / "docker.jsonl"
        docker = self.root / "docker"
        docker.write_text(
            "#!/usr/bin/env python3\n"
            "import json, os, sys\n"
            "with open(os.environ['DOCKER_LOG'], 'a') as log:\n"
            "    log.write(json.dumps(sys.argv[1:]) + '\\n')\n"
            "args = sys.argv[1:]\n"
            "if args[:2] == ['container', 'inspect']:\n"
            "    sys.exit(0 if os.environ.get('CONTAINER_EXISTS') else 1)\n"
            "if args[0] == 'inspect':\n"
            "    print('false' if os.environ.get('NOT_RUNNING') else 'true')\n"
            "if args[0] == 'run':\n"
            "    sys.exit(int(os.environ.get('RUN_EXIT', '0')))\n"
        )
        docker.chmod(0o755)
        curl = self.root / "curl"
        curl.write_text('#!/bin/sh\nexit "${CURL_EXIT:-0}"\n')
        curl.chmod(0o755)
        sleep = self.root / "sleep"
        sleep.write_text("#!/bin/sh\nexit 0\n")
        sleep.chmod(0o755)
        self.env = dict(
            os.environ,
            HOME=str(self.root),
            PATH=f"{self.root}:{os.environ['PATH']}",
            DOCKER_LOG=str(self.log),
            BAZEL_REMOTE_NAME="test-remote",
        )
        for key in (
            "BAZEL_REMOTE_PORT",
            "BAZEL_REMOTE_CACHE_DIR",
            "BAZEL_REMOTE_MAX_SIZE_GIB",
            "BAZEL_REMOTE_IMAGE",
        ):
            self.env.pop(key, None)

    def run_launcher(self, *args, **env):
        return subprocess.run(
            ["bash", str(SCRIPT), *args],
            env=dict(self.env, **env),
            capture_output=True,
            text=True,
            check=False,
        )

    def calls(self):
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def test_starts_loopback_cache_as_current_user(self):
        result = self.run_launcher()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("http://127.0.0.1:19090", result.stdout)
        run = next(call for call in self.calls() if call[0] == "run")
        self.assertIn("127.0.0.1:19090:8080", run)
        self.assertIn(f"{os.getuid()}:{os.getgid()}", run)
        default_cache = self.root / ".cache/bazel-remote"
        self.assertIn(f"type=bind,src={default_cache},dst=/data", run)
        self.assertIn("buchgr/bazel-remote-cache:v2.6.1", run)
        self.assertEqual(run[-4:], ["--dir", "/data", "--max_size", "5"])

    def test_existing_container_is_not_replaced(self):
        result = self.run_launcher(CONTAINER_EXISTS="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stderr)
        self.assertFalse(
            any(call[0] in ("run", "start", "rm") for call in self.calls())
        )

    def test_explicit_settings_are_used(self):
        cache = self.root / "custom-cache"
        result = self.run_launcher(
            BAZEL_REMOTE_PORT="19191",
            BAZEL_REMOTE_CACHE_DIR=str(cache),
            BAZEL_REMOTE_MAX_SIZE_GIB="10",
            BAZEL_REMOTE_IMAGE="example/cache:v1",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        run = next(call for call in self.calls() if call[0] == "run")
        self.assertIn("127.0.0.1:19191:8080", run)
        self.assertIn(f"type=bind,src={cache},dst=/data", run)
        self.assertIn("example/cache:v1", run)
        self.assertEqual(run[-2:], ["--max_size", "10"])

    def test_invalid_configuration_does_not_invoke_docker(self):
        for env in (
            {"BAZEL_REMOTE_PORT": "65536"},
            {"BAZEL_REMOTE_PORT": "0"},
            {"BAZEL_REMOTE_MAX_SIZE_GIB": "0"},
            {"BAZEL_REMOTE_CACHE_DIR": "relative"},
        ):
            with self.subTest(env=env):
                result = self.run_launcher(**env)
                self.assertEqual(result.returncode, 2)
                self.assertEqual(self.calls(), [])

    def test_unready_container_is_reported_without_removing_it(self):
        result = self.run_launcher(CURL_EXIT="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("failed to become ready", result.stderr)
        self.assertTrue(any(call[0] == "logs" for call in self.calls()))
        calls = self.calls()
        self.assertFalse(any(call[0] in ("stop", "rm") for call in calls))

    def test_docker_run_failure_does_not_report_ready(self):
        result = self.run_launcher(RUN_EXIT="1")
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("ready", result.stdout)
        self.assertFalse(any(call[0] == "inspect" for call in self.calls()))


if __name__ == "__main__":
    unittest.main()
