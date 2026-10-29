#!/usr/bin/env python3
"""Compare recursive and selected builds without clearing shared caches."""

import argparse
import json
import os
import subprocess
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument(
        "--rebuild-local-sources",
        action="store_true",
        help="also force matched drivers/canbus compiles with an unused per-file macro",
    )
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be positive")
    if os.getuid() == 0 or not Path("/.dockerenv").exists():
        parser.error("run as the mapped non-root user in the managed container")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    common = [
        "bazel",
        "build",
        "--config=cpu",
        "--config=opt",
        "--jobs=4",
        "--local_resources=cpu=4",
        "--platforms=//tools/hardware:linux_x86_64",
        "--workspace_status_command=bash scripts/hardware_workspace_status.sh",
        "--define=vehicle=devkit",
    ]
    variants = {
        "recursive": [
            "--",
            "//modules/drivers/...",
            "//modules/canbus/...",
        ],
        "selected": ["--config=hw_x86_devkit", "--", "//tools/hardware:hardware"],
    }
    report = {
        "revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "cache_policy": "existing shared caches; one warmup per variant; no clean",
        "scope": "drivers and canbus; hardware inventory intentionally differs",
        "source_status": subprocess.check_output(
            ["bash", "scripts/hardware_workspace_status.sh"], text=True
        ).splitlines(),
        "environment": subprocess.check_output(["uname", "-sm"], text=True).strip(),
        "shared_machine": "not isolated; no other processes stopped",
        "samples": [],
    }
    probe = time.time_ns()
    for name, targets in variants.items():
        samples = list(range(args.samples + 1))
        if args.rebuild_local_sources:
            samples.append("rebuild")
        for sample in samples:
            stem = args.output_dir / f"{name}-{sample}"
            rebuild_options = []
            if sample == "rebuild":
                probe += 1
                rebuild_options = [
                    "--per_file_copt=modules/(drivers|canbus)/.*@"
                    f"-DHARDWARE_PROFILE_TIMING_PROBE={probe}"
                ]
            command = (
                common
                + [
                    f"--profile={stem}.profile.gz",
                    f"--build_event_json_file={stem}.bep.json",
                ]
                + rebuild_options
                + targets
            )
            start = time.monotonic()
            with stem.with_suffix(".log").open("w") as log:
                result = subprocess.run(
                    command, stdout=log, stderr=subprocess.STDOUT, check=False
                )
            entry = {
                "variant": name,
                "sample": sample,
                "warmup": sample == 0,
                "local_source_recompile": sample == "rebuild",
                "command": command,
                "wall_seconds": time.monotonic() - start,
                "returncode": result.returncode,
            }
            report["samples"].append(entry)
            (args.output_dir / "timings.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print(json.dumps(entry), flush=True)
            if result.returncode:
                raise SystemExit(
                    f"Stopped at first failed build; inspect {stem}.log. "
                    "Failed builds cannot establish speedup."
                )


if __name__ == "__main__":
    main()
