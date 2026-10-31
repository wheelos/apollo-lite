#!/usr/bin/env python3
"""Create a relocatable hardware runtime from explicitly declared Bazel files."""

import filecmp
import json
import shutil
import sys
from pathlib import Path, PurePosixPath

TEXT_SUFFIXES = {".dag", ".launch", ".txt", ".conf", ".yaml", ".yml"}


def launcher_text(command, parents="..", change_directory=True):
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f'root="$(cd "$(dirname "${{BASH_SOURCE[0]}}")/{parents}" && pwd)"\n'
        'export CYBER_PATH="${root}"\n'
        'export APOLLO_ROOT_DIR="${root}"\n'
        'export LD_LIBRARY_PATH="${root}/lib'
        '${WHEELOS_RUNTIME_LIBRARY_PATH:+:${WHEELOS_RUNTIME_LIBRARY_PATH}}"\n'
        'if [[ -z "${GLOG_log_dir:-}" && -z "${GLOG_logtostderr:-}" ]]; then\n'
        "  export GLOG_logtostderr=1\n"
        "fi\n"
        'export TERM="${TERM:-xterm-256color}"\n'
        + ('cd "${root}"\n' if change_directory else "")
        + f'exec {command} "$@"\n'
    )


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
    launcher.write_text(launcher_text('"${root}/bin/mainboard"'))
    launcher.chmod(0o755)
    entry = output / "bin/run"
    entry.write_text(launcher_text('python3 "${root}/bin/runtime_entry.py"'))
    entry.chmod(0o755)
    for name, command in {
        "mainboard": '"${root}/bin/mainboard"',
        "cyber_launch": 'python3 "${root}/bin/cyber_launch.py"',
        "cyber_recorder": '"${root}/bin/cyber_recorder"',
        "cyber_monitor": '"${root}/bin/cyber_monitor"',
    }.items():
        wrapper = output / "bin/commands" / name
        wrapper.parent.mkdir(parents=True, exist_ok=True)
        wrapper.write_text(
            launcher_text(
                command,
                "../..",
                change_directory=name not in ("cyber_recorder", "cyber_monitor"),
            )
        )
        wrapper.chmod(0o755)
    setup = output / "bin/setup.bash"
    setup.write_text(
        "#!/usr/bin/env bash\n"
        'PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/commands:${PATH}"\n'
        "export PATH\n"
    )
    setup.chmod(0o755)
    config = output / "conf/runtime.json"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(json.dumps({"default_dag": spec.get("default_dag", "")}) + "\n")
    config.chmod(0o644)
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
