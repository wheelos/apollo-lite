"""Exercise one core and recorder with FakeCAN and a synthetic sensor only."""

import fcntl
import hashlib
import json
import os
import pty
import select
import shutil
import signal
import struct
import subprocess
import sys
import tarfile
import tempfile
import termios
import time
import unittest
import uuid
from pathlib import Path

from python.runfiles import runfiles


class SystemIntegration(unittest.TestCase):
    def stop(self, process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
                self.fail("runtime did not shut down after SIGINT")

    def observe(self, root, environment, topics):
        master, slave = pty.openpty()
        fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 30, 180, 0, 0))
        process = subprocess.Popen(
            [str(root / "bin/commands/cyber_monitor")],
            env=dict(environment, TERM="xterm"),
            stdin=slave,
            stdout=slave,
            stderr=slave,
            start_new_session=True,
        )
        output = b""
        try:
            deadline = time.monotonic() + 20
            while not all(topic.encode() in output for topic in topics):
                self.assertIsNone(process.poll(), output.decode(errors="replace"))
                self.assertLess(
                    time.monotonic(), deadline, output.decode(errors="replace")
                )
                if select.select([master], [], [], 0.2)[0]:
                    output += os.read(master, 65536)
            self.stop(process)
            self.assertEqual(process.returncode, 0)
        finally:
            self.stop(process)
            os.close(master)
            os.close(slave)

    def test_source_archive_and_install_share_one_core(self):
        resolver = runfiles.Create()
        source, probe, archive, installer = [
            Path(resolver.Rlocation(argument)) for argument in sys.argv[1:]
        ]
        for variant in ("source", "archive", "install"):
            with (
                self.subTest(variant=variant),
                tempfile.TemporaryDirectory() as temporary,
            ):
                directory = Path(temporary)
                original = directory / "original"
                if variant == "source":
                    shutil.copytree(source, original)
                elif variant == "archive":
                    with tarfile.open(archive) as package:
                        for member in package.getmembers():
                            path = Path(member.name)
                            self.assertFalse(path.is_absolute() or ".." in path.parts)
                            self.assertFalse(member.issym() or member.islnk())
                        package.extractall(original)
                else:
                    result = subprocess.run(
                        [str(installer), "--destdir", str(original)],
                        capture_output=True,
                        text=True,
                        timeout=30,
                    )
                    self.assertEqual(
                        result.returncode, 0, result.stdout + result.stderr
                    )
                root = directory / "relocated system"
                original.rename(root)
                manifest = json.loads((root / "share/system-manifest.json").read_text())
                self.assertEqual(manifest["modules"], ["vehicle", "sensors"])
                for relative, digest in manifest["files"].items():
                    self.assertEqual(
                        hashlib.sha256((root / relative).read_bytes()).hexdigest(),
                        digest,
                    )
                model = root / "models/example/model.engine"
                self.assertEqual(model.read_bytes(), b"model fixture\n")
                if variant != "source":
                    self.assertEqual(model.stat().st_mode & 0o777, 0o644)
                for tool in ("mainboard", "cyber_recorder", "cyber_monitor"):
                    self.assertEqual(
                        len(list(root.rglob(tool))), 2
                    )  # Native tool plus wrapper.
                identity = uuid.uuid4().hex[:12]
                chassis_topic = f"/system/test/{identity}/chassis"
                image_topic = f"/system/test/{identity}/image"
                flags = root / "conf/vehicle/vehicle.conf"
                flags.chmod(0o644)
                with flags.open("a") as output:
                    output.write(
                        f"\n--chassis_topic={chassis_topic}\n"
                        f"--control_command_topic=/system/test/{identity}"
                        "/unused_control\n"
                    )
                environment = dict(
                    os.environ,
                    CYBER_DOMAIN_ID=str(180 + int(identity[:4], 16) % 40),
                    CYBER_IP="127.0.0.1",
                    GLOG_logtostderr="1",
                    SENSOR_INFRA_TEST_TOPIC=image_topic,
                )
                environment.pop("WHEELOS_CONFIG_ROOT", None)
                processes = []
                logs = []
                try:
                    for name, arguments in (
                        ("vehicle", ["start", "launch/vehicle/vehicle.launch"]),
                        ("sensor", ["start", "launch/sensors/synthetic.launch"]),
                        (
                            "recorder",
                            [
                                "record",
                                "-c",
                                chassis_topic,
                                image_topic,
                                "-o",
                                "./shared.record",
                                "-i",
                                "0",
                                "-m",
                                "0",
                            ],
                        ),
                    ):
                        log = directory / f"{name}.log"
                        logs.append(log)
                        tool = (
                            "cyber_recorder" if name == "recorder" else "cyber_launch"
                        )
                        with log.open("w") as output:
                            processes.append(
                                subprocess.Popen(
                                    [str(root / "bin/commands" / tool), *arguments],
                                    cwd=directory,
                                    env=environment,
                                    stdout=output,
                                    stderr=subprocess.STDOUT,
                                    start_new_session=True,
                                )
                            )
                    self.observe(root, environment, [chassis_topic, image_topic])
                    time.sleep(0.5)
                    for process, log in zip(processes, logs):
                        self.assertIsNone(process.poll(), log.read_text()[-8000:])
                    # Stopping one launch must not stop the other module or recorder.
                    self.stop(processes[0])
                    self.assertEqual(
                        processes[0].returncode, 0, logs[0].read_text()[-8000:]
                    )
                    time.sleep(1)
                    for process, log in zip(processes[1:], logs[1:]):
                        self.assertIsNone(process.poll(), log.read_text()[-8000:])
                    self.stop(processes[2])
                    self.assertEqual(
                        processes[2].returncode, 0, logs[2].read_text()[-8000:]
                    )
                    records = list(directory.glob("shared.record*"))
                    self.assertEqual(len(records), 1, logs[2].read_text()[-8000:])
                    verified = subprocess.run(
                        [str(probe), str(records[0]), chassis_topic, image_topic],
                        env=dict(environment, LD_LIBRARY_PATH=str(root / "lib")),
                        capture_output=True,
                        text=True,
                        timeout=15,
                    )
                    self.assertEqual(
                        verified.returncode, 0, verified.stdout + verified.stderr
                    )
                    counts = json.loads(verified.stdout)
                    self.assertGreater(
                        counts["last_image_time"] - counts["last_chassis_time"],
                        500_000_000,
                    )
                    self.stop(processes[1])
                    self.assertEqual(
                        processes[1].returncode, 0, logs[1].read_text()[-8000:]
                    )
                    self.assertIn("Cleanup Canbus component", logs[0].read_text())
                    self.assertIn("Cleanup synthetic sensor", logs[1].read_text())
                    print(
                        f"{variant}: one core/recorder; monitor saw both topics; "
                        f"{counts}"
                    )
                except (AssertionError, OSError, subprocess.SubprocessError):
                    for log in logs:
                        print(log.name, log.read_text()[-8000:], file=sys.stderr)
                    raise
                finally:
                    for process in reversed(processes):
                        self.stop(process)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
