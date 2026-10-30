#!/usr/bin/env python3
"""Verify mainboard loads selected plugin ELFs without initializing sensors."""

import argparse
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path


def verify(runtime):
    runtime = runtime.resolve()
    manifest = json.loads((runtime / "hardware/manifest.json").read_text())
    plugins = [
        runtime / path for path in manifest["component_files"] if path.endswith(".so")
    ]
    if not plugins:
        raise ValueError("Select at least one plugin for the loader smoke check")
    environment = dict(os.environ)
    environment.pop("LD_LIBRARY_PATH", None)
    environment.pop("RUNFILES_DIR", None)
    environment["GLOG_logtostderr"] = "1"
    with tempfile.TemporaryDirectory(prefix="hardware-loader-") as directory:
        dag = runtime / "hardware" / "load-only.dag"
        with (Path(directory) / "mainboard.log").open("w+") as log:
            process = subprocess.Popen(
                [str(runtime / "bin/mainboard-hardware"), "-d", str(dag)],
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 20
                while process.poll() is None and time.monotonic() < deadline:
                    maps = Path(f"/proc/{process.pid}/maps").read_text()
                    if all(str(plugin) in maps for plugin in plugins):
                        for line in maps.splitlines():
                            path = line.split()[-1]
                            if "/.cache/bazel/" in path and not path.startswith(
                                str(runtime) + "/"
                            ):
                                raise RuntimeError(
                                    f"Runtime loaded a build-cache library: {path}"
                                )
                        print(f"mainboard mapped {len(plugins)} selected plugins")
                        return
                    time.sleep(0.1)
                log.seek(0)
                raise RuntimeError(
                    "Selected plugins were not mapped by mainboard:\n" + log.read()
                )
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runtime", type=Path)
    verify(parser.parse_args().runtime)


if __name__ == "__main__":
    main()
