#!/usr/bin/env python3
"""Read-only preflight, explicit-input fingerprints, checkpoints, and rubrics."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def text(value):
    return isinstance(value, str) and bool(value.strip())


def string_list(value):
    return isinstance(value, list) and all(text(item) for item in value)


def load(path):
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"expected JSON object: {path}")
    return value


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def git(root, *args):
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), *args], stderr=subprocess.PIPE
        )
    except subprocess.CalledProcessError as error:
        raise ValueError(error.stderr.decode(errors="replace").strip()) from error


def repo_fingerprint(root):
    require(root.is_absolute(), "repository paths must be absolute")
    root = root.resolve()
    actual = Path(os.fsdecode(git(root, "rev-parse", "--show-toplevel")).strip())
    require(actual == root, f"not a Git checkout root: {root}")
    hasher = hashlib.sha256()
    for name in sorted(set(git(root, "ls-files", "-z").split(b"\0")) - {b""}):
        path = root / os.fsdecode(name)
        hasher.update(len(name).to_bytes(8, "big") + name)
        if path.is_symlink():
            content = b"link:" + os.fsencode(os.readlink(path))
        elif path.is_file():
            content = f"file:{path.stat().st_mode & 0o777}:{digest(path)}".encode()
        elif path.is_dir():
            raise ValueError(
                f"nested Git input must be fingerprinted explicitly: {path}"
            )
        else:
            content = b"missing"
        hasher.update(len(content).to_bytes(8, "big") + content)
    return {
        "path": str(root),
        "head": git(root, "rev-parse", "HEAD").decode().strip(),
        "index_sha256": hashlib.sha256(
            git(root, "diff", "--cached", "--binary", "--no-ext-diff")
        ).hexdigest(),
        "tracked_sha256": hasher.hexdigest(),
    }


def file_record(path):
    require(path.is_absolute() and path.is_file(), f"expected absolute file: {path}")
    return {"path": str(path), "sha256": digest(path)}


def snapshot(repositories, inputs):
    require(bool(repositories), "at least one named repository is required")
    require(
        all(text(name) for name in repositories), "repository names must be nonempty"
    )
    return {
        "repositories": {
            name: repo_fingerprint(path) for name, path in sorted(repositories.items())
        },
        "inputs": [file_record(path) for path in inputs],
    }


def check_fingerprint(value):
    require(isinstance(value, dict), "fingerprint must be an object")
    require(isinstance(value.get("repositories"), dict), "missing repositories")
    records = value["repositories"]
    require(
        all(
            isinstance(item, dict) and text(item.get("path"))
            for item in records.values()
        ),
        "invalid repository record",
    )
    require(isinstance(value.get("inputs"), list), "missing fingerprint inputs")
    paths = []
    for item in value["inputs"]:
        require(
            isinstance(item, dict) and text(item.get("path")), "invalid input record"
        )
        paths.append(Path(item["path"]))
    current = snapshot(
        {name: Path(item["path"]) for name, item in records.items()}, paths
    )
    require(
        current == value, "stale fingerprint: code, configuration, or inputs changed"
    )


def check_artifacts(artifacts, required=True):
    require(isinstance(artifacts, list), "artifacts must be a list")
    require(
        not required or bool(artifacts), "passed/completed evidence needs artifacts"
    )
    for item in artifacts:
        require(isinstance(item, dict) and text(item.get("path")), "invalid artifact")
        require(
            file_record(Path(item["path"])) == item,
            f"missing or changed artifact: {item['path']}",
        )


def checkpoint(data):
    require(data.get("schema_version") == 1, "unsupported checkpoint schema")
    for key in ("task_id", "goal", "scope", "next_action"):
        require(text(data.get(key)), f"missing {key}")
    require(
        data.get("status") in ("in_progress", "blocked", "complete"), "invalid status"
    )
    for key in ("decisions", "blockers", "pending_approvals", "required_checks"):
        require(string_list(data.get(key)), f"{key} must be a string list")
    checks = data["required_checks"]
    require(bool(checks) and len(checks) == len(set(checks)), "invalid required checks")
    check_fingerprint(data.get("fingerprint"))
    evidence = data.get("evidence")
    require(isinstance(evidence, list), "evidence must be a list")
    seen = set()
    all_passed = True
    for item in evidence:
        require(isinstance(item, dict), "invalid evidence item")
        name = item.get("check")
        require(
            text(name) and name in checks and name not in seen,
            "unknown or duplicate evidence check",
        )
        seen.add(name)
        require(text(item.get("command")), "evidence needs actual command")
        status = item.get("status")
        require(
            status in ("passed", "failed", "blocked", "skipped", "not_run"),
            "invalid evidence status",
        )
        check_artifacts(item.get("artifacts"), required=status == "passed")
        all_passed = all_passed and status == "passed"
    if data["status"] == "complete":
        require(
            not data["blockers"] and not data["pending_approvals"],
            "complete task still has blockers or pending approvals",
        )
        require(
            all_passed and seen == set(checks),
            "complete task lacks all required passing evidence",
        )
    return {
        "status": "valid",
        "task_status": data["status"],
        "evidence_count": len(evidence),
        "authorization": "not_granted",
    }


def evaluate(suite, data):
    require(
        suite.get("schema_version") == data.get("schema_version") == 1,
        "unsupported evaluation schema",
    )
    require(data.get("suite_version") == suite.get("suite_version"), "suite mismatch")
    for key in ("run_id", "configuration", "reviewer"):
        require(text(data.get(key)), f"missing evaluation {key}")
    cases = suite["cases"]
    require(isinstance(cases, list) and bool(cases), "empty case set")
    definitions = {}
    for case in cases:
        require(isinstance(case, dict) and text(case.get("id")), "invalid case")
        require(case["id"] not in definitions, "duplicate case definition")
        for key in ("prompt", "setup", "expected_routing"):
            require(text(case.get(key)), f"missing case {key}")
        require(case.get("partition") in ("tuning", "holdout"), "invalid partition")
        gates = case.get("gates")
        require(isinstance(gates, dict) and bool(gates), "missing case gates")
        require(
            all(text(name) and type(hard) is bool for name, hard in gates.items()),
            "invalid gate definition",
        )
        definitions[case["id"]] = case
    require(isinstance(data.get("cases"), list), "missing case results")
    completed = set()
    seen = set()
    failures = []
    hard_failures = []
    for result in data["cases"]:
        require(isinstance(result, dict), "invalid case result")
        case_id = result.get("id")
        require(
            text(case_id) and case_id in definitions and case_id not in seen,
            "unknown or duplicate case result",
        )
        seen.add(case_id)
        require(result.get("status") in ("completed", "not_run"), "invalid case status")
        if result["status"] == "not_run":
            require(not result.get("gates"), "not-run case cannot claim gate results")
            continue
        case = definitions[case_id]
        gates = result.get("gates")
        require(
            isinstance(gates, dict) and gates.keys() == case["gates"].keys(),
            f"gate coverage mismatch: {case_id}",
        )
        require(
            all(type(value) is bool for value in gates.values()),
            "gate results must be booleans",
        )
        check_artifacts(result.get("artifacts"))
        completed.add(case_id)
        for gate, passed in gates.items():
            if not passed:
                finding = f"{case_id}:{gate}"
                failures.append(finding)
                if case["gates"][gate]:
                    hard_failures.append(finding)
    missing = sorted(set(definitions) - completed)
    return {
        "status": "passed" if not failures and not missing else "failed",
        "completed": len(completed),
        "not_run": missing,
        "failed_gates": failures,
        "hard_gate_failures": hard_failures,
        "judgment_source": "reviewer_supplied; not independently verified",
    }


def preflight(root, expected_uid, tools):
    checks = []

    def record(name, passed, remedy):
        checks.append(
            {
                "check": name,
                "status": "passed" if passed else "blocked",
                "remedy": "" if passed else remedy,
            }
        )

    record(
        "mapped_nonroot_user",
        os.getuid() != 0 and os.getuid() == expected_uid,
        "Use whl exec dev/test as the mapped host user; do not change ownership.",
    )
    record(
        "managed_workspace",
        root.resolve() == Path("/apollo") and Path("/.dockerenv").is_file(),
        "Enter the managed container; this check does not authorize host builds.",
    )
    record("git_checkout", (root / ".git").exists(), "Use the mounted source checkout.")
    for name in tools:
        record(
            f"tool:{name}",
            shutil.which(name) is not None,
            f"Tool {name} is missing; diagnose and request dependency approval.",
        )
    record(
        "context_catalog",
        (root / "ad_agents/context/context-catalog.json").is_file(),
        "Initialize the context checkout from ad_agents.repos after reviewing it.",
    )
    cache = root / ".cache/bazel"
    record(
        "existing_bazel_cache",
        cache.is_dir() and os.access(cache, os.W_OK | os.X_OK),
        "Inspect the shared cache permissions; do not clean it or switch output roots.",
    )
    return {
        "status": (
            "ready" if all(item["status"] == "passed" for item in checks) else "blocked"
        ),
        "checks": checks,
        "scope": "identity, tools, context, cache only; no GPU/fixture/runtime proof",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    prepare = commands.add_parser("preflight")
    prepare.add_argument("--root", type=Path, default=ROOT)
    prepare.add_argument("--expected-uid", type=int, required=True)
    prepare.add_argument("--tool", action="append", default=["python3", "git"])
    capture = commands.add_parser("snapshot")
    capture.add_argument("--repo", action="append", required=True, metavar="NAME=PATH")
    capture.add_argument("--input", type=Path, action="append", default=[])
    resume = commands.add_parser("checkpoint")
    resume.add_argument("file", type=Path)
    judge = commands.add_parser("evaluate")
    judge.add_argument("file", type=Path)
    judge.add_argument(
        "--suite", type=Path, default=Path(__file__).with_name("evaluation-cases.json")
    )
    args = parser.parse_args()
    try:
        if args.action == "preflight":
            report = preflight(args.root, args.expected_uid, args.tool)
        elif args.action == "snapshot":
            repos = {}
            for spec in args.repo:
                name, separator, path = spec.partition("=")
                require(
                    separator and text(name) and path and name not in repos,
                    "expected unique NAME=absolute-path",
                )
                repos[name] = Path(path)
            report = snapshot(repos, args.input)
        elif args.action == "checkpoint":
            report = checkpoint(load(args.file))
        else:
            report = evaluate(load(args.suite), load(args.file))
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        subprocess.CalledProcessError,
    ) as error:
        print(json.dumps({"status": "invalid", "error": str(error)}))
        return 2
    print(json.dumps(report, indent=2))
    return 1 if report.get("status") in ("blocked", "failed") else 0


if __name__ == "__main__":
    sys.exit(main())
