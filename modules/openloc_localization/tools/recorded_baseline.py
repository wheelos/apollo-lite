#!/usr/bin/env python3
"""Reproduce the diagnostic local Cyber baseline; no third-party Python packages."""

import argparse
import collections
import csv
import hashlib
import html
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import statistics
import subprocess


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def checked(path, expected):
    if digest(path) != expected:
        raise ValueError(f"frozen input changed: {path}")
    return Path(path)


def rows(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def finite(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("nonfinite diagnostic evidence")
    return value


def summarize(publications, imu, reference, maximum_age=0.1):
    if not publications:
        raise ValueError("no actual Cyber publications")
    periods, ages, differences, new_times = [], [], [], []
    last_sequence = 0
    reasons = collections.Counter()
    usable = 0
    for item in publications:
        sequence = int(item["sequence"])
        if sequence < last_sequence or not 1 <= sequence <= len(imu):
            raise ValueError("publication source frontier invalid")
        source = imu[sequence - 1]
        if (
            finite(item["measurement_s"]) != int(source["timestamp_ns"]) / 1e9
            or finite(item["receive_s"]) != int(source["receive_ns"]) / 1e9
        ):
            raise ValueError("original source clocks changed")
        age = finite(item["age_s"])
        period = int(item["publication_interval_ns"])
        if age < 0 or period < 0:
            raise ValueError("negative age/period")
        ages.append(age)
        periods.append(period)
        reasons[item["reason"]] += 1
        if item["usable"] == "1":
            usable += 1
            if age > maximum_age:
                raise ValueError("stale source incorrectly usable")
            state = {
                name: finite(item[name])
                for name in ("px", "py", "pz", "qx", "qy", "qz", "qw", "vx", "vy", "vz")
            }
            if abs(sum(state[k] ** 2 for k in ("qx", "qy", "qz", "qw")) - 1) > 1e-8:
                raise ValueError("nonunit published rotation")
            if item["new_measurement"] == "1":
                if sequence <= last_sequence and new_times:
                    raise ValueError("held sample mislabeled new")
                new_times.append(finite(item["measurement_s"]))
                before = reference[sequence - 1]
                differences.append(
                    math.sqrt(
                        sum(
                            (state[k] - finite(before[k])) ** 2
                            for k in ("px", "py", "pz")
                        )
                    )
                )
        elif any(
            item[k]
            for k in ("px", "py", "pz", "qx", "qy", "qz", "qw", "vx", "vy", "vz")
        ):
            raise ValueError("unusable output carries a pose")
        last_sequence = sequence
    if last_sequence != len(imu) or not differences:
        raise ValueError("did not consume usable output and final source frontier")
    return {
        "messages": len(publications),
        "usable": usable,
        "new_usable": len(new_times),
        "maximum_interval_ns": max(periods),
        "intervals_over_requested_10ms": sum(value > 10_000_000 for value in periods),
        "whole_period_misses_20ms": sum(
            max(0, value // 10_000_000 - 1) for value in periods
        ),
        "maximum_host_local_age_s": max(ages),
        "maximum_new_source_gap_s": max(
            (b - a for a, b in zip(new_times, new_times[1:])), default=0
        ),
        "maximum_position_difference_from_sync_m": max(differences),
        "reason_counts": dict(reasons),
        "accuracy_error_claimed": False,
    }


def scan_frontiers(actual, reference):
    if len(actual) != len(reference):
        raise ValueError("incomplete scan decisions")
    for after, before in zip(actual, reference):
        if after["sequence"] != before["sequence"]:
            raise ValueError("scan source identity changed")
        for old, new in (
            ("raw_end_ns", "raw_end_s"),
            ("source_receive_ns", "source_receive_s"),
            ("reference_ns", "reference_s"),
            ("processing_frontier_receive_ns", "frontier_receive_s"),
        ):
            if int(before[old]) / 1e9 != finite(after[new]):
                raise ValueError(f"scan source/reference frontier changed: {new}")


def profile_text(source, arm, session, protocol):
    (extrinsic,) = rows(source / "extrinsic.csv")
    fields = {
        "imu_topic": f"/openloc/baseline/{session}/imu",
        "lidar_topic": f"/openloc/baseline/{session}/lidar",
        "session": session,
        "lidar_frame": "livox_frame",
        "calibration_id": "mid360-manual-T_I_L",
        "journal": str(arm / "lidar.csv"),
    }
    text = "# Original Unix input, manual extrinsic, diagnostic only.\n"
    text += "".join(f"{name}: {json.dumps(value)}\n" for name, value in fields.items())
    text += (
        "imu_measurement_to_unix_offset: 0\nlidar_measurement_to_unix_offset: 0\n"
        "imu_from_lidar_translation { "
        f'x: {extrinsic["tx"]} y: {extrinsic["ty"]} z: {extrinsic["tz"]}'
        " }\nimu_from_lidar_rotation { qx: 0 qy: 0 qz: 0 qw: 1 }\n"
        f'bootstrap_voxel_m: {protocol["bootstrap_voxel_m"]}\n'
        f'lidar_commit_delay_ms: {protocol["lidar_commit_delay_ms"]}\n'
        "allow_imu_only_initialization: true\nrecorded_receive_time_from_header: true\n"
    )
    for parameter in rows(source / "config.csv"):
        finite(parameter["value"])
        text += (
            f'parameter {{ name: {json.dumps(parameter["name"])} '
            f'value: {parameter["value"]} }}\n'
        )
    return text


def runtime_identity(binary):
    if "k8-opt" not in binary.parts:
        raise ValueError("baseline requires the reference's opt compilation mode")
    ldd = subprocess.check_output(["ldd", str(binary)], text=True)
    if "not found" in ldd:
        raise ValueError("unresolved host library")
    libraries = re.findall(r"=> (/[^\s]+)", ldd)
    return {
        "binary": str(binary),
        "sha256": digest(binary),
        "ldd": re.sub(r"0x[0-9a-fA-F]+", "0xADDR", ldd),
        "libraries_sha256": {name: digest(name) for name in libraries},
    }


def sources(workspace, native):
    files = [
        workspace / name
        for name in ("BUILD", "MODULE.bazel", ".bazelrc", ".bazelrc.dev")
    ]
    files += [p for p in workspace.iterdir() if p.suffix in (".cc", ".h", ".proto")]
    for directory in ("tools", "testdata", "patches"):
        files += [
            p for p in (workspace / directory).rglob("*")
            if p.is_file() and "__pycache__" not in p.parts
        ]
    for directory in ("include", "src/localization"):
        files += [
            p for p in (native / directory).rglob("*")
            if p.suffix in (".cc", ".h")
        ]
    files += [native / "BUILD", native / "MODULE.bazel"]
    return {str(path.resolve()): digest(path) for path in files}


def archive(output, directory, pins):
    for name, expected in pins.items():
        source = checked(name, expected)
        destination = output / directory / source.relative_to("/")
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        checked(destination, expected)


def exercise(native, output, protocol, capture, index, mode, binaries):
    arm = output / f'{capture["id"]}-{index}-{mode}'
    arm.mkdir()
    source = native / capture["input"]
    manifest_path = checked(source / "manifest.json", capture["manifest_sha256"])
    manifest = json.loads(manifest_path.read_text())
    if manifest["external_odometry_used"] or manifest["external_initial_pose_used"]:
        raise ValueError("baseline must not consume external odometry/initial pose")
    for name, expected in manifest["files"].items():
        path = (source / name).resolve()
        if not path.is_relative_to(source.resolve()):
            raise ValueError("manifest path escaped input directory")
        checked(path, expected)
    reference_dir = native / protocol["reference_root"] / capture["id"] / "replay"
    reference = rows(
        checked(reference_dir / "odometry.csv", capture["odometry_sha256"])
    )
    lidar_reference = rows(
        checked(reference_dir / "lidar.csv", capture["lidar_sha256"])
    )
    imu = rows(source / "imu.csv")
    if len(imu) != capture["imu_samples"] or len(reference) != len(imu):
        raise ValueError("IMU source/reference count changed")
    for before, after in zip(imu, reference):
        if any(before[key] != after[key] for key in ("timestamp_ns", "receive_ns")):
            raise ValueError("reference uses different source clocks")
    session = "baseline-" + capture["id"] + "-" + str(index) + "-" + mode
    profile = arm / "profile.pb.txt"
    profile.write_text(profile_text(source, arm, session, protocol))
    host_command = [str(binaries["host"]), str(profile)]
    replay_command = [
        str(binaries["replay"]), str(source), str(profile), str(arm / "replay")
    ]
    if mode == "prefetch":
        replay_command.append("--prefetch-cloud")
    write_json(arm / "commands.json", {"host": host_command, "replay": replay_command})
    with (arm / "host.log").open("x") as host_log:
        child = subprocess.Popen(
            host_command, stdout=host_log, stderr=subprocess.STDOUT
        )
        try:
            with (arm / "replay.log").open("x") as log:
                subprocess.run(
                    replay_command, stdout=log, stderr=subprocess.STDOUT,
                    timeout=60, check=True,
                )
            child.send_signal(signal.SIGINT)
            if child.wait(timeout=10) != 0:
                raise RuntimeError("raw localization host failed")
        finally:
            if child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
                    raise
    ingress, = rows(arm / "lidar.csv.ingress.csv")
    if ingress != {
        "imu_enqueued": str(capture["imu_samples"]),
        "scans_enqueued": str(capture["scans"]), "recorded_receive_time": "1",
    }:
        raise ValueError("Cyber admission count changed")
    scan_frontiers(rows(arm / "lidar.csv"), lidar_reference)
    tasks = rows(arm / "lidar.csv.tasks.csv")
    statuses = collections.Counter(
        task["status"] + ":" + task["reason"] for task in tasks
    )
    if not statuses["committed:NONE"]:
        raise ValueError("no LIO corrections actually committed")
    metrics = summarize(rows(arm / "replay/publications.csv"), imu, reference)
    summary, = rows(arm / "replay/summary.csv")
    if summary["tail_received"] != "1" or int(summary["tf_received"]) == 0:
        raise ValueError("actual diagnostic output/TF incomplete")
    timing = rows(arm / "replay/send_timing.csv")
    if len(timing) != len(imu):
        raise ValueError("incomplete sender timing evidence")
    for sequence, (sample, recorded) in enumerate(zip(timing, imu), 1):
        if (
            int(sample["sequence"]) != sequence
            or sample["receive_ns"] != recorded["receive_ns"]
        ):
            raise ValueError("sender timing source identity changed")
    lateness = sorted(int(sample["send_lateness_ns"]) for sample in timing)
    if lateness[0] < 0 or lateness[-1] != int(summary["maximum_send_lateness_ns"]):
        raise ValueError("invalid sender timing summary")
    metrics["send_lateness_ns"] = {
        "p50": lateness[math.ceil(0.50 * len(lateness)) - 1],
        "p95": lateness[math.ceil(0.95 * len(lateness)) - 1],
        "maximum": lateness[-1],
        "over_10ms": sum(value > 10_000_000 for value in lateness),
    }
    return {
        "capture": capture["id"],
        "repeat": index,
        "mode": mode,
        "manifest_sha256": digest(manifest_path),
        "scan_source_and_reference_frontiers_equal": True,
        "ingress": ingress,
        "tasks": dict(statuses),
        "metrics": metrics,
        "sender": summary,
        "artifacts_sha256": {
            str(p.relative_to(arm)): digest(p) for p in arm.rglob("*") if p.is_file()
        },
    }


def render(output, report):
    document = [
        '<!doctype html><meta charset="utf-8"><title>OpenLoc local baseline</title>',
        "<h1>OpenLoc recorded local baseline</h1>",
        "<p>Diagnostic admission-confirmed replay, not live 100 Hz or absolute "
        "accuracy. Map/GNSS/global host, target graph error model, reset and "
        "independent truth are not covered.</p>",
        "<table border=1><tr><th>Capture/run</th><th>LIO commits/late</th>"
        "<th>New usable</th><th>Max timer ms</th><th>Sender late ms</th>"
        "<th>Sync trajectory difference m (not accuracy)</th></tr>",
    ]
    for run in report["runs"]:
        metrics, tasks = run["metrics"], run["tasks"]
        document.append(
            f'<tr><td>{html.escape(run["capture"])} / '
            f'{run["repeat"]} / {run["mode"]}</td>'
            f'<td>{tasks.get("committed:NONE", 0)} / '
            f'{tasks.get("deadline_missed:LIDAR_STALE", 0)}</td>'
            f'<td>{metrics["new_usable"]}</td>'
            f'<td>{metrics["maximum_interval_ns"]/1e6:.3f}</td>'
            f'<td>{int(run["sender"]["maximum_send_lateness_ns"])/1e6:.3f}</td>'
            f'<td>{metrics["maximum_position_difference_from_sync_m"]:.6f}</td></tr>'
        )
    document.append(
        "</table><p>Full source/runtime pins, original input hashes, per-sample "
        "source clocks/pose/age and task outcomes are retained in report.json "
        "and run directories.</p>"
    )
    for capture in report["protocol"]["captures"]:
        document.append(
            f'<h2>{html.escape(capture["id"])}: local ODOM XY, no truth</h2>'
        )
        series = []
        for run in report["runs"]:
            if run["capture"] != capture["id"]:
                continue
            name = f'{run["capture"]}-{run["repeat"]}-{run["mode"]}'
            points = [
                (finite(p["px"]), finite(p["py"]))
                for p in rows(output / name / "replay/publications.csv")
                if p["usable"] == "1" and p["new_measurement"] == "1"
            ]
            series.append((name, points[:: max(1, len(points) // 600)]))
        all_points = [p for _, points in series for p in points]
        xmin, ymin = (min(p[i] for p in all_points) for i in (0, 1))
        scale = max(
            max(p[0] for p in all_points) - xmin,
            max(p[1] for p in all_points) - ymin, 1,
        )
        document.append(
            '<svg viewBox="0 0 660 660" width="660" height="660" '
            'style="border:1px solid">'
        )
        for index, (name, points) in enumerate(series):
            color = ("blue", "orange", "green", "red", "purple", "brown")[index % 6]
            coordinates = " ".join(
                f"{20+600*(x-xmin)/scale:.2f},{640-600*(y-ymin)/scale:.2f}"
                for x, y in points
            )
            document.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="1" '
                f'points="{coordinates}">'
                f'<title>{html.escape(name)}</title></polyline>'
            )
        document.append(
            "</svg><p>" + html.escape(", ".join(name for name, _ in series)) + "</p>"
        )
    if "comparison" in report:
        document.append(
            "<h2>Paired sender optimization (not an accuracy claim)</h2><pre>"
            + html.escape(json.dumps(report["comparison"], indent=2)) + "</pre>"
        )
    (output / "index.html").write_text("\n".join(document))


def compare(report):
    results = []
    protocol = report["protocol"]
    for capture in protocol["captures"]:
        groups = {
            mode: [
                r for r in report["runs"]
                if r["capture"] == capture["id"] and r["mode"] == mode
            ]
            for mode in ("serialized", "prefetch")
        }
        if not all(groups.values()):
            continue
        if (
            [r["repeat"] for r in groups["serialized"]]
            != [r["repeat"] for r in groups["prefetch"]]
        ):
            raise ValueError("unpaired optimization evidence")
        control, optimized = (
            statistics.median(
                r["metrics"]["send_lateness_ns"]["p95"] for r in groups[mode]
            )
            for mode in ("serialized", "prefetch")
        )
        reduction = (control - optimized) / max(1, control)
        sufficient = (
            len(groups["serialized"]) >= protocol["optimization_minimum_repeats"]
        )
        results.append({
            "capture": capture["id"],
            "pairs": len(groups["serialized"]),
            "median_p95_send_lateness_ns": {
                "serialized": control, "prefetch": optimized,
            },
            "reduction_fraction": reduction,
            "meets_declared_sender_goal": sufficient and reduction >= protocol[
                "optimization_minimum_median_p95_lateness_reduction_fraction"
            ],
            "median_new_usable": {
                mode: statistics.median(r["metrics"]["new_usable"] for r in group)
                for mode, group in groups.items()
            },
            "median_lio_commits": {
                mode: statistics.median(r["tasks"]["committed:NONE"] for r in group)
                for mode, group in groups.items()
            },
        })
    return results


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest", type=Path,
        default=workspace / "testdata/recorded_baseline.json",
    )
    parser.add_argument(
        "--openloc-root", type=Path, default=workspace / "../../pkgs/openloc"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument(
        "--replay-mode", choices=("serialized", "prefetch", "paired"),
        default="serialized",
    )
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    if os.environ.get("LD_LIBRARY_PATH") or os.environ.get("LD_PRELOAD"):
        parser.error(
            "unset LD_LIBRARY_PATH and LD_PRELOAD; do not inject other workspace DSOs"
        )
    native, output = args.openloc_root.resolve(), args.output.resolve()
    protocol = json.loads(args.manifest.read_text())
    if protocol["schema"] != "openloc.local-cyber-baseline.v1":
        raise ValueError("unrecognized baseline protocol")
    binaries = {
        "host": (workspace / "bazel-bin/native_lio_diagnostic").resolve(),
        "replay": (workspace / "bazel-bin/native_lio_cyber_replay").resolve(),
    }
    identity = {name: runtime_identity(path) for name, path in binaries.items()}
    source_pins = sources(workspace, native)
    output.mkdir(parents=True, exist_ok=False)
    archive(output, "sources", source_pins)
    runtime_pins = {}
    for item in identity.values():
        runtime_pins[item["binary"]] = item["sha256"]
        runtime_pins.update(item["libraries_sha256"])
    archive(output, "runtime", runtime_pins)
    report = {
        "schema": protocol["schema"],
        "diagnostic_only": True,
        "complete_global_host": False,
        "accuracy_qualified": False,
        "live_or_100hz_qualified": False,
        "manifest_sha256": digest(args.manifest),
        "protocol": protocol,
        "source_sha256": source_pins,
        "runtime": identity,
        "environment": {
            "platform": platform.platform(),
            "cyber_path": os.environ.get("CYBER_PATH"),
        },
        "runs": [],
    }
    for index in range(args.repeats):
        for capture in protocol["captures"]:
            modes = (
                ["serialized", "prefetch"]
                if args.replay_mode == "paired" else [args.replay_mode]
            )
            if index % 2:
                modes.reverse()
            for mode in modes:
                result = exercise(
                    native, output, protocol, capture, index, mode, binaries
                )
                report["runs"].append(result)
                write_json(output / "progress.json", report)
                print(
                    capture["id"], index, mode, json.dumps(result["metrics"]),
                    flush=True,
                )
    if sources(workspace, native) != source_pins:
        raise RuntimeError("source/configuration changed during baseline")
    for name, path in binaries.items():
        if runtime_identity(path) != identity[name]:
            raise RuntimeError("binary/runtime changed during baseline")
    report["comparison"] = compare(report)
    write_json(output / "report.json", report)
    render(output, report)
    print(f"Baseline complete: {output / 'report.json'}")


if __name__ == "__main__":
    main()
