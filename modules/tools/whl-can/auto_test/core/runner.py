import threading
import time
from typing import Callable, Optional

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2

from core.ui import UIHandler
from config.manager import ConfigManager
from util import create_base_command


class CommandHeartbeat(threading.Thread):
    """Sends control commands at fixed 100Hz frequency."""

    def __init__(self, writer, interval=0.01):
        super().__init__(name="CmdHeartbeat", daemon=True)
        self.writer = writer
        self.interval = interval
        self.active = True
        self.current_cmd = control_cmd_pb2.ControlCommand()
        self.lock = threading.Lock()
        self.seq_num = 0

    def update(self, cmd: control_cmd_pb2.ControlCommand):
        with self.lock:
            self.current_cmd.CopyFrom(cmd)

    def run(self):
        while self.active:
            with self.lock:
                self.seq_num += 1
                self.current_cmd.header.sequence_num = self.seq_num
                self.current_cmd.header.timestamp_sec = cyber.time.Time.now().to_sec()
                self.current_cmd.header.module_name = "chassis_tester"
                self.writer.write(self.current_cmd)
            time.sleep(self.interval)

    def stop(self):
        self.active = False


class TestRunner:
    def __init__(self, ui: UIHandler, config: ConfigManager, level: int):
        self.ui = ui
        self.config = config
        self.run_level = level

        self.node = cyber.Node("chassis_tester_node")
        self.writer = self.node.create_writer(
            config.topics["control"], control_cmd_pb2.ControlCommand
        )
        self.node.create_reader(
            config.topics["chassis"], chassis_pb2.Chassis, self._chassis_cb
        )

        self._chassis_msg = None
        self._chassis_lock = threading.Lock()
        self._last_chassis_ts = 0.0

        self.heartbeat = CommandHeartbeat(self.writer)
        self.estop_triggered = threading.Event()

        self.test_cases = []

    def _chassis_cb(self, msg):
        with self._chassis_lock:
            self._chassis_msg = msg
            self._last_chassis_ts = time.time()

    def register_test_case(self, func, name, level):
        self.test_cases.append({"func": func, "name": name, "level": level})

    def start(self):
        self.ui.draw_header("Apollo Chassis Auto-Tester", self.run_level)
        self.heartbeat.start()
        # Start safety monitoring thread
        self.safety_thread = threading.Thread(target=self._safety_monitor, daemon=True)
        self.safety_thread.start()

        self.ui.log("System Initialized.", "CYAN")
        if not self.wait_for_chassis(
            self.config.thresholds.get("chassis_wait_sec", 5.0)
        ):
            self.ui.log("No chassis feedback detected. Check /apollo/chassis.", "RED")
        else:
            self.ui.log("Chassis feedback OK.", "GREEN")
        self.ui.log("Press [Enter] to begin testing...", "YELLOW")
        self.ui.wait_for_enter()

    def update_cmd(self, cmd):
        """Update the command being sent by the heartbeat."""
        self.heartbeat.update(cmd)

    def update_command(self, cmd):
        """Compatibility alias for test cases."""
        self.update_cmd(cmd)

    def get_chassis(self) -> Optional[chassis_pb2.Chassis]:
        with self._chassis_lock:
            return self._chassis_msg

    def get_latest_chassis(self) -> Optional[chassis_pb2.Chassis]:
        return self.get_chassis()

    def wait_for_chassis(self, timeout_sec: float) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.get_chassis() is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_for_condition(
        self, condition_fn: Callable[[], bool], timeout_sec: float, desc: str
    ) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            if self.estop_triggered.is_set():
                self.ui.log(f"Aborted: {desc}", "RED")
                return False
            if condition_fn():
                return True
            time.sleep(0.01)
        self.ui.log(f"Timeout: {desc}", "YELLOW")
        return False

    def wait_for_chassis_condition(
        self,
        predicate: Callable[[chassis_pb2.Chassis], bool],
        timeout_sec: float,
        desc: str,
    ) -> bool:
        def condition() -> bool:
            msg = self.get_latest_chassis()
            return bool(msg and predicate(msg))

        return self.wait_for_condition(
            condition,
            timeout_sec,
            desc,
        )

    def prepare_for_drive(self) -> bool:
        """Request AUTO + Drive with parking brake released in a safe sequence."""
        if not self.wait_for_chassis(
            self.config.thresholds.get("chassis_wait_sec", 5.0)
        ):
            return False

        cmd = create_base_command()
        cmd.brake = 20.0
        cmd.parking_brake = True
        cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
        self.update_cmd(cmd)

        if not self.wait_for_chassis_condition(
            lambda msg: msg.driving_mode == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE,
            5.0,
            "Enter Auto Mode",
        ):
            return False

        cmd.parking_brake = False
        cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
        cmd.brake = 0.0
        self.update_cmd(cmd)

        return True

    def _safety_monitor(self):
        """Watchdog: Checks for Spacebar press and Chassis Errors."""
        while not self.estop_triggered.is_set():
            # 1. Check Keyboard
            key = self.ui.get_input()
            if key == ord(" "):
                self.trigger_estop("User requested Emergency Stop (Spacebar)")

            # 2. Check Chassis Error (Optional: Configurable)
            # msg = self.get_chassis()
            # if msg and msg.error_code != ...

            time.sleep(0.05)

    def trigger_estop(self, reason):
        if self.estop_triggered.is_set():
            return
        self.estop_triggered.set()
        self.ui.log(f"!!! ESTOP TRIGGERED: {reason} !!!", "RED")

        # Construct Safe Stop Command
        cmd = create_base_command()
        cmd.throttle = 0.0
        cmd.brake = self.config.limits.get("emergency_brake_val", 50.0)
        cmd.parking_brake = True
        cmd.driving_mode = chassis_pb2.Chassis.COMPLETE_MANUAL

        self.update_cmd(cmd)

    def reset_to_safe_state(self):
        self.ui.log("Resetting to safe state (N, Brake)...", "CYAN")
        cmd = create_base_command()
        cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
        cmd.brake = 20.0
        cmd.parking_brake = True
        self.update_cmd(cmd)
        time.sleep(1.0)

    def run_sequence(self):
        tests = [t for t in self.test_cases if t["level"] <= self.run_level]

        if not tests:
            self.ui.log("No tests found for this level.", "YELLOW")
            return

        for i, case in enumerate(tests):
            if self.estop_triggered.is_set():
                break

            self.reset_to_safe_state()
            self.ui.log(f"[{i+1}/{len(tests)}] Running: {case['name']}", "DEFAULT")

            try:
                # Pass self (runner) to the test function
                result = case["func"](self)

                status = "PASS" if result.get("pass") else "FAIL"
                color = "GREEN" if result.get("pass") else "RED"
                self.ui.log(f"  -> {status}: {result.get('details')}", color)

            except Exception as e:
                self.ui.log(f"  -> EXCEPTION: {e}", "RED")

            time.sleep(2)

        if self.estop_triggered.is_set():
            self.ui.log("Sequence aborted due to ESTOP.", "RED")
        else:
            self.ui.log("All tests completed.", "GREEN")
            self.ui.log("Press [Enter] to exit.", "CYAN")
            self.ui.wait_for_enter()

    def cleanup(self):
        self.heartbeat.stop()
        self.estop_triggered.set()
        if hasattr(self, "safety_thread"):
            self.safety_thread.join(timeout=1.0)
        cyber.shutdown()
