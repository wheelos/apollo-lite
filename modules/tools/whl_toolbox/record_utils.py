from __future__ import annotations

import json
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
RECORD_TOOL_TARGET = "//modules/tools/whl_toolbox:record_tool"
RECORD_TOOL_BINARY = REPO_ROOT / "bazel-bin/modules/tools/whl_toolbox/record_tool"


def _ensure_record_tool_binary() -> Path:
    if RECORD_TOOL_BINARY.exists() and RECORD_TOOL_BINARY.is_file():
        return RECORD_TOOL_BINARY
    raise RuntimeError(
        f"missing binary for {RECORD_TOOL_TARGET}: {RECORD_TOOL_BINARY}. "
        "Please compile the required target inside the Apollo container before running whl-toolbox."
    )


def _run_record_tool(args: list[str]) -> dict[str, Any]:
    binary = _ensure_record_tool_binary()
    process = subprocess.run(
        [str(binary), *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if process.returncode != 0:
        message = process.stderr.strip() or process.stdout.strip() or "record_tool failed"
        raise RuntimeError(message)
    payload = process.stdout.strip()
    if not payload:
        raise RuntimeError("record_tool returned empty output")
    return json.loads(payload)


def inspect_records(raw_path: str) -> dict[str, Any]:
    return _run_record_tool(["--mode=inspect", f"--input_path={raw_path}"])


def extract_pointcloud_dataset(
    raw_path: str,
    pointcloud_topic: str,
    output_dir: Path,
    *,
    pose_topic: str = "",
    pose_match_threshold_sec: float = 0.05,
    max_frames: int = 0,
    progress_cb=None,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    progress_file = output_dir / "extract_progress.json"
    if progress_file.exists():
        progress_file.unlink()

    binary = _ensure_record_tool_binary()
    command = [
        str(binary),
        "--mode=extract",
        f"--input_path={raw_path}",
        f"--pointcloud_topic={pointcloud_topic}",
        f"--output_dir={output_dir}",
        f"--pose_match_threshold_sec={pose_match_threshold_sec}",
        f"--progress_file={progress_file}",
    ]
    if pose_topic:
        command.append(f"--pose_topic={pose_topic}")
    if max_frames > 0:
        command.append(f"--max_frames={max_frames}")

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    last_progress_raw = ""
    while process.poll() is None:
        if progress_cb and progress_file.exists():
            raw = progress_file.read_text(encoding="utf-8")
            if raw != last_progress_raw:
                last_progress_raw = raw
                try:
                    payload = json.loads(raw)
                except Exception:
                    payload = {}
                progress_cb(
                    int(payload.get("current", 0) or 0),
                    int(payload.get("total", 0) or 0),
                    str(payload.get("message", "")),
                )
        time.sleep(0.2)

    stdout, stderr = process.communicate()
    if progress_cb and progress_file.exists():
        try:
            payload = json.loads(progress_file.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
        progress_cb(
            int(payload.get("current", 0) or 0),
            int(payload.get("total", 0) or 0),
            str(payload.get("message", "")),
        )

    if process.returncode != 0:
        message = (stderr or stdout).strip() or "record_tool extract failed"
        raise RuntimeError(message)
    payload = stdout.strip()
    if not payload:
        raise RuntimeError("record_tool extract returned empty output")
    return json.loads(payload)


def count_topic_messages(raw_path: str, topic: str) -> int:
    payload = _run_record_tool(
        ["--mode=count", f"--input_path={raw_path}", f"--topic={topic}"]
    )
    return int(payload.get("count", 0) or 0)


def copy_tree(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
