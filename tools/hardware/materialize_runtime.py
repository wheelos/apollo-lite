#!/usr/bin/env python3
"""Create a relocatable hardware runtime from explicitly declared Bazel files."""

import filecmp
import json
import shutil
import sys
from pathlib import Path, PurePosixPath

TEXT_SUFFIXES = {".dag", ".launch", ".txt", ".conf", ".yaml", ".yml"}


def materialize(spec, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    for source, relative in spec["files"] + spec["libraries"]:
        path = PurePosixPath(relative)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError(f"Invalid runtime destination: {relative}")
        source = Path(source)
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            if not filecmp.cmp(source, destination, shallow=False):
                raise ValueError(f"Conflicting runtime files: {relative}")
            continue
        if source.suffix in TEXT_SUFFIXES:
            destination.write_text(source.read_text().replace("/apollo/", ""))
            destination.chmod(0o644)
        else:
            shutil.copyfile(source, destination)
            destination.chmod(0o755)
    launcher = output / "bin" / "mainboard-hardware"
    launcher.parent.mkdir(parents=True, exist_ok=True)
    launcher.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        'root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"\n'
        'export CYBER_PATH="${root}"\n'
        'export APOLLO_ROOT_DIR="${root}"\n'
        'export LD_LIBRARY_PATH="${root}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"\n'
        'cd "${root}"\n'
        'exec "${root}/bin/mainboard" "$@"\n'
    )
    launcher.chmod(0o755)
    dag = output / "hardware" / "load-only.dag"
    dag.parent.mkdir(parents=True, exist_ok=True)
    dag.write_text(
        "\n".join(
            f'module_config {{ module_library: "{relative}" }}'
            for _, relative in spec["files"]
            if relative.startswith("bazel-bin/") and relative.endswith(".so")
        )
        + "\n"
    )
    dag.chmod(0o644)


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "--package":
        split_package(Path(sys.argv[2]), Path(sys.argv[3]))
        return
    if len(sys.argv) != 3:
        raise SystemExit("usage: materialize_runtime.py SPEC OUTPUT")
    materialize(json.loads(Path(sys.argv[1]).read_text()), sys.argv[2])


def split_package(runtime, output):
    directories = {
        "bin": 0o755,
        "lib": 0o755,
        "bazel-bin": 0o755,
        "modules": 0o644,
        "conf": 0o644,
        "hardware": 0o644,
    }
    for directory in directories:
        (output / directory).mkdir(parents=True, exist_ok=True)
    for source in runtime.rglob("*"):
        if not source.is_file():
            continue
        relative = source.relative_to(runtime)
        if relative.parts[0] not in directories:
            raise ValueError(f"Unknown runtime package directory: {relative}")
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        destination.chmod(directories[relative.parts[0]])


if __name__ == "__main__":
    main()
