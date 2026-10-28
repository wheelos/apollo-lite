import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts.agents.runtime import (
    ROOT,
    checkpoint,
    evaluate,
    file_record,
    load,
    preflight,
    snapshot,
)


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.repo = self.root / "repo"
        self.repo.mkdir()
        self.git("init", "--quiet")
        self.git("config", "user.name", "Fixture")
        self.git("config", "user.email", "fixture@example.invalid")
        self.source = self.repo / "source.txt"
        self.source.write_text("before\n", encoding="utf-8")
        self.git("add", "source.txt")
        self.git("-c", "commit.gpgsign=false", "commit", "--quiet", "-m", "Fixture")
        self.extra = self.root / "fixture.json"
        self.extra.write_text('{"value": 1}\n', encoding="utf-8")
        self.log = self.root / "result.log"
        self.log.write_text("actual fixture check result\n", encoding="utf-8")
        self.data = {
            "schema_version": 1,
            "task_id": "fixture",
            "goal": "Check the fixture",
            "scope": "Disposable repository only",
            "status": "complete",
            "next_action": "Report actual evidence",
            "decisions": [],
            "blockers": [],
            "pending_approvals": [],
            "required_checks": ["fixture-check"],
            "fingerprint": snapshot({"repo": self.repo}, [self.extra]),
            "evidence": [
                {
                    "check": "fixture-check",
                    "command": "fixture test",
                    "status": "passed",
                    "artifacts": [file_record(self.log)],
                }
            ],
        }

    def git(self, *args):
        return subprocess.check_output(
            ["git", "-C", str(self.repo), *args], stderr=subprocess.PIPE
        )

    def test_unchanged_evidence_is_valid_but_does_not_authorize(self):
        result = checkpoint(self.data)
        self.assertEqual("valid", result["status"])
        self.assertEqual("not_granted", result["authorization"])

    def test_dirty_tracked_source_and_explicit_fixture_invalidate(self):
        self.source.write_text("after\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "stale fingerprint"):
            checkpoint(self.data)
        self.source.write_text("before\n", encoding="utf-8")
        self.extra.write_text('{"value": 2}\n', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "stale fingerprint"):
            checkpoint(self.data)

    def test_deleted_tracked_file_invalidates(self):
        self.source.unlink()
        with self.assertRaisesRegex(ValueError, "stale fingerprint"):
            checkpoint(self.data)

    def test_staged_only_change_invalidates_even_if_worktree_is_restored(self):
        self.source.write_text("staged change\n", encoding="utf-8")
        self.git("add", "source.txt")
        self.source.write_text("before\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "stale fingerprint"):
            checkpoint(self.data)

    def test_complete_rejects_missing_checks_approval_and_false_pass(self):
        for field, value in (
            ("evidence", []),
            ("pending_approvals", ["publish"]),
            ("blockers", ["missing tool"]),
        ):
            data = copy.deepcopy(self.data)
            data[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                checkpoint(data)
        self.data["evidence"][0]["status"] = "skipped"
        with self.assertRaisesRegex(ValueError, "passing evidence"):
            checkpoint(self.data)

    def test_artifact_mutation_invalidates_evidence(self):
        self.log.write_text("changed result\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "changed artifact"):
            checkpoint(self.data)

    def test_untracked_scope_is_explicit_and_directories_rejected(self):
        untracked = self.repo / "untracked.txt"
        untracked.write_text("new input\n", encoding="utf-8")
        before = snapshot({"repo": self.repo}, [untracked])
        untracked.write_text("changed input\n", encoding="utf-8")
        self.assertNotEqual(before, snapshot({"repo": self.repo}, [untracked]))
        with self.assertRaisesRegex(ValueError, "absolute file"):
            snapshot({"repo": self.repo}, [self.repo])

    def test_preflight_does_not_accept_host_or_root(self):
        with patch("scripts.agents.runtime.os.getuid", return_value=0):
            report = preflight(
                self.repo, os.getuid(), ["definitely-missing-agent-tool"]
            )
        self.assertEqual("blocked", report["status"])
        self.assertTrue(
            any(
                item["check"] == "mapped_nonroot_user" and item["status"] == "blocked"
                for item in report["checks"]
            )
        )

    def evaluation_report(self):
        suite = load(ROOT / "scripts/agents/evaluation-cases.json")
        report = {
            "schema_version": 1,
            "suite_version": suite["suite_version"],
            "run_id": "unit-fixture-not-agent-run",
            "configuration": "Synthetic report used only to test aggregation",
            "reviewer": "unit-test fixture",
            "cases": [
                {
                    "id": case["id"],
                    "status": "completed",
                    "gates": {gate: True for gate in case["gates"]},
                    "artifacts": [file_record(self.log)],
                }
                for case in suite["cases"]
            ],
        }
        return suite, report

    def test_gate_failure_cannot_be_averaged_away(self):
        suite, report = self.evaluation_report()
        self.assertEqual("passed", evaluate(suite, report)["status"])
        report["cases"][0]["gates"]["root_cause_supported"] = False
        result = evaluate(suite, report)
        self.assertEqual("failed", result["status"])
        self.assertEqual(
            ["local-defect:root_cause_supported"], result["hard_gate_failures"]
        )

    def test_missing_cases_and_non_boolean_gates_do_not_pass(self):
        suite, report = self.evaluation_report()
        report["cases"].pop()
        self.assertEqual(["review-only"], evaluate(suite, report)["not_run"])
        report["cases"][0]["gates"]["root_cause_supported"] = "true"
        with self.assertRaisesRegex(ValueError, "booleans"):
            evaluate(suite, report)

    def test_evaluation_requires_actual_artifacts_and_exact_suite(self):
        suite, report = self.evaluation_report()
        report["suite_version"] = "other"
        with self.assertRaisesRegex(ValueError, "suite mismatch"):
            evaluate(suite, report)
        report["suite_version"] = suite["suite_version"]
        report["cases"][0]["artifacts"] = []
        with self.assertRaisesRegex(ValueError, "needs artifacts"):
            evaluate(suite, report)

    def test_not_run_cases_cannot_claim_gates(self):
        suite, report = self.evaluation_report()
        report["cases"][0]["status"] = "not_run"
        with self.assertRaisesRegex(ValueError, "cannot claim"):
            evaluate(suite, report)

    def test_cli_snapshot_checkpoint_and_stale_input_exit_codes(self):
        tool = ROOT / "scripts/agents/runtime.py"
        capture = subprocess.run(
            [
                sys.executable,
                str(tool),
                "snapshot",
                "--repo",
                f"fixture={self.repo}",
                "--input",
                str(self.extra),
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        self.data["fingerprint"] = json.loads(capture.stdout)
        path = self.root / "checkpoint.json"
        path.write_text(json.dumps(self.data), encoding="utf-8")
        command = [sys.executable, str(tool), "checkpoint", str(path)]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        self.assertEqual("valid", json.loads(result.stdout)["status"])
        self.extra.write_text("changed fixture\n", encoding="utf-8")
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(2, result.returncode)
        self.assertIn("stale fingerprint", json.loads(result.stdout)["error"])

    def test_cli_evaluation_exposes_failed_and_missing_case_status(self):
        suite, report = self.evaluation_report()
        report["cases"] = []
        path = self.root / "evaluation.json"
        path.write_text(json.dumps(report), encoding="utf-8")
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts/agents/runtime.py"),
                "evaluate",
                str(path),
            ],
            capture_output=True,
            text=True,
        )
        self.assertEqual(1, result.returncode)
        self.assertEqual(len(suite["cases"]), len(json.loads(result.stdout)["not_run"]))
