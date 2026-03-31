from __future__ import annotations

import os
import queue
import shlex
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

try:
    from .job_manager import JobContext
except ImportError:  # pragma: no cover
    from job_manager import JobContext


APOLLO_ROOT = Path("/apollo")


def repo_root_from_file() -> Path:
    return Path(__file__).resolve().parents[3]


REPO_ROOT = repo_root_from_file()


def _ensure_in_apollo_container() -> None:
    if REPO_ROOT != APOLLO_ROOT:
        raise RuntimeError(
            "whl-toolbox only supports running inside the Apollo container at /apollo. "
            "Please enter the container and run python3 modules/tools/whl_toolbox/server.py there."
        )


def host_to_container(path: Path) -> str:
    resolved = path.resolve()
    repo_root = REPO_ROOT.resolve()
    try:
        relative = resolved.relative_to(repo_root)
    except ValueError as exc:
        raise ValueError(f"path is outside repo mount: {resolved}") from exc
    return str(APOLLO_ROOT / relative)


def bazel_target_to_binary_path(target: str) -> str:
    label = target[2:] if target.startswith("//") else target
    package, binary = label.split(":")
    return str(APOLLO_ROOT / "bazel-bin" / package / binary)


def container_command(script: str) -> list[str]:
    _ensure_in_apollo_container()
    inner = (
        "cd /apollo && "
        "source /apollo/cyber/setup.bash >/dev/null 2>&1 && "
        f"{script}"
    )
    return ["bash", "-lc", inner]


def run_container_script(
    script: str,
    *,
    job: Optional[JobContext] = None,
    progress_file: Optional[Path] = None,
    extra_env: Optional[dict[str, str]] = None,
) -> None:
    _ensure_in_apollo_container()
    env = os.environ.copy()
    if extra_env:
        env.update(extra_env)
    command = container_command(script)
    if job:
        job.log("$ " + " ".join(shlex.quote(part) for part in command))
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
    )
    output_queue: "queue.Queue[str]" = queue.Queue()

    def _reader() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            output_queue.put(line.rstrip())

    reader = threading.Thread(target=_reader, daemon=True)
    reader.start()

    last_progress_mtime = 0.0
    while True:
        try:
            line = output_queue.get(timeout=0.2)
            if job and line:
                job.log(line)
        except queue.Empty:
            pass

        if progress_file and progress_file.exists():
            mtime = progress_file.stat().st_mtime
            if mtime != last_progress_mtime:
                last_progress_mtime = mtime
                if job:
                    job.read_progress_file(progress_file)

        if process.poll() is not None and output_queue.empty():
            break

    if progress_file and progress_file.exists() and job:
        job.read_progress_file(progress_file)

    if process.returncode != 0:
        raise RuntimeError(f"container command failed with exit code {process.returncode}")


def _container_binary_exists(binary_path: str) -> bool:
    _ensure_in_apollo_container()
    return os.access(binary_path, os.X_OK)


def container_binary_supports_flag(binary_path: str, flag: str) -> bool:
    _ensure_in_apollo_container()
    command = container_command(f"{shlex.quote(binary_path)} --help")
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return flag in result.stdout


def ensure_container_binary(
    target: str, job: Optional[JobContext] = None, *, try_build: bool = True
) -> str:
    del try_build
    _ensure_in_apollo_container()
    binary_path = bazel_target_to_binary_path(target)
    existed = _container_binary_exists(binary_path)
    if existed:
        if job:
            job.log(f"using existing binary for {target}: {binary_path}")
        return binary_path
    raise RuntimeError(
        f"missing binary for {target}: {binary_path}. "
        "Please compile the required target inside the Apollo container before running whl-toolbox."
    )


def wait_for_file(path: Path, timeout_sec: float = 10.0) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if path.exists():
            return True
        time.sleep(0.1)
    return path.exists()
