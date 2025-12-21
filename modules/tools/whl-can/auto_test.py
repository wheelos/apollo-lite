#!/usr/bin/env python

"""
A comprehensive test runner for Apollo's chassis control functionalities.
This version implements an expanded set of test cases based on a detailed
checklist, covering linearity, response times, and functional safety.

Features:
- Tiered test levels (L1: Static, L2: Low-Speed/Dynamic, L3: High-Dynamic).
- Robust safety mechanism with keyboard-based emergency stop.
- Interactive Curses-based UI for real-time reporting.
- Based on industry best practices for vehicle testing.
"""

import curses
import threading
import time
import logging
import argparse
from datetime import datetime
from typing import Dict, Any, List

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2


# --- Global Configuration ---
NODE_NAME = "chassis_auto_tester"
DEFAULT_MODULE_NAME = "chassis_tester"
CONTROL_TOPIC = "/apollo/control"
CHASSIS_TOPIC = "/apollo/chassis"
LOGGING_LEVEL = logging.INFO

# --- Curses Color Pair IDs ---
COLOR_PAIR_DEFAULT = 1
COLOR_PAIR_GREEN = 2
COLOR_PAIR_RED = 3
COLOR_PAIR_YELLOW = 4
COLOR_PAIR_CYAN = 5


def calculate_response_metrics(
    command_time: float, data_stream: list, target_value: float, is_brake: bool = False
):
    """
    Calculate response time, overshoot, and steady-state error.
    - command_time: precise timestamp when command is sent
    - data_stream: [(timestamp, value), ...]
    - is_brake: brake response uses different criteria
    """
    if not data_stream:
        return {
            "response_time_ms": -1,
            "overshoot_percent": -1,
            "steady_state_error_percent": -1,
        }

    initial_value = data_stream[0][1]

    # Response time (t90)
    t90_threshold = initial_value + (target_value - initial_value) * 0.9
    t90_time = -1
    for t, val in data_stream:
        if t < command_time:
            continue
        condition_met = (
            (val >= t90_threshold)
            if (target_value > initial_value)
            else (val <= t90_threshold)
        )
        if condition_met:
            t90_time = (t - command_time) * 1000  # ms
            break

    # Overshoot and steady-state error
    relevant_data = [(t, v) for t, v in data_stream if t >= command_time]
    if not relevant_data:
        return {
            "response_time_ms": t90_time,
            "overshoot_percent": -1,
            "steady_state_error_percent": -1,
        }

    peak_value = (
        max(v for t, v in relevant_data)
        if target_value > initial_value
        else min(v for t, v in relevant_data)
    )
    steady_state_data = [
        v for t, v in relevant_data if t > command_time + 2.0
    ]  # Data after 2s of command

    overshoot = (
        ((peak_value - target_value) / target_value) * 100 if target_value != 0 else 0
    )

    if steady_state_data:
        steady_state_avg = sum(steady_state_data) / len(steady_state_data)
        steady_state_error = steady_state_avg - target_value
        steady_state_error_percent = (
            (steady_state_error / target_value) * 100 if target_value != 0 else 0
        )
    else:
        steady_state_error_percent = -1

    return {
        "response_time_ms": round(t90_time, 1),
        "overshoot_percent": round(overshoot, 2),
        "steady_state_error_percent": round(steady_state_error_percent, 2),
    }


def create_base_command() -> control_cmd_pb2.ControlCommand:
    """
    Create a ControlCommand with safe default values and timestamp.
    Header should be updated on each call.
    """
    cmd = control_cmd_pb2.ControlCommand()
    cmd.pad_msg.driving_mode = chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE
    cmd.pad_msg.action = control_cmd_pb2.PadMessage.START
    cmd.throttle, cmd.brake, cmd.steering_target, cmd.speed, cmd.acceleration = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    cmd.parking_brake = True
    cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
    return cmd


def update_cmd_header(cmd: control_cmd_pb2.ControlCommand, seq_num: int) -> None:
    """Update command header information."""
    cmd.header.module_name = DEFAULT_MODULE_NAME
    cmd.header.sequence_num = seq_num
    cmd.header.timestamp_sec = cyber.time.Time.now().to_sec()


class TestRunner:
    """
    A fully-featured autonomous chassis test executor.
    """

    def __init__(self, screen, level: int):
        self.screen = screen
        self.run_level = level
        self.cmd_seq_num = 0

        cyber.init()
        self.node = cyber.Node(NODE_NAME)
        self.control_writer = self.node.create_writer(
            CONTROL_TOPIC, control_cmd_pb2.ControlCommand
        )
        self.chassis_reader = self.node.create_reader(
            CHASSIS_TOPIC, chassis_pb2.Chassis, self.chassis_callback
        )

        self.latest_chassis_msg = None
        self.lock = threading.Lock()
        self.test_cases = []
        self.emergency_stop_triggered = threading.Event()
        self.safety_thread = None

        self._setup_logging()
        self._setup_screen()

    def _setup_logging(self):
        logging.basicConfig(
            level=LOGGING_LEVEL, format="%(asctime)s - %(levelname)s - %(message)s"
        )

    def _setup_screen(self):
        curses.curs_set(0)
        self.screen.nodelay(True)
        curses.start_color()
        curses.init_pair(COLOR_PAIR_DEFAULT, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(COLOR_PAIR_GREEN, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(COLOR_PAIR_RED, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(COLOR_PAIR_YELLOW, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(COLOR_PAIR_CYAN, curses.COLOR_CYAN, curses.COLOR_BLACK)

    def chassis_callback(self, msg: chassis_pb2.Chassis):
        with self.lock:
            self.latest_chassis_msg = msg

    def register_test_case(self, func, name: str, level: int):
        self.test_cases.append({"func": func, "name": name, "level": level})

    def get_latest_chassis(self) -> chassis_pb2.Chassis:
        time.sleep(0.02)
        with self.lock:
            return self.latest_chassis_msg

    def send_control_command(self, cmd: control_cmd_pb2.ControlCommand):
        self.cmd_seq_num += 1
        update_cmd_header(cmd, self.cmd_seq_num)
        self.control_writer.write(cmd)

    def _perform_emergency_stop(self):
        self.log_to_screen("!!! EMERGENCY STOP TRIGGERED !!!", COLOR_PAIR_RED)
        cmd = create_base_command()
        cmd.brake = 100.0
        cmd.throttle = 0.0
        cmd.speed = 0.0
        cmd.acceleration = 0.0
        # cmd.parking_brake = True
        self.send_control_command(cmd)
        time.sleep(0.1)
        self.send_control_command(cmd)

    def _safety_monitor(self):
        while not self.emergency_stop_triggered.is_set():
            try:
                key = self.screen.getch()
                if key == ord(" "):
                    self.emergency_stop_triggered.set()
                    self._perform_emergency_stop()
                    break
            except curses.error:
                pass
            time.sleep(0.02)

    def wait_for_condition(
        self, condition_func, timeout: float, description: str
    ) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.emergency_stop_triggered.is_set():
                return False
            if condition_func():
                return True
            time.sleep(0.05)
        self.log_to_screen(f"Timeout waiting for: {description}", COLOR_PAIR_YELLOW)
        return False

    def reset_to_safe_state(self, duration: float = 1.0):
        self.log_to_screen(
            "Resetting vehicle to safe state (Brake, N, EPB)...", COLOR_PAIR_CYAN
        )
        cmd = create_base_command()
        cmd.brake = 50.0
        cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
        cmd.parking_brake = True
        self.send_control_command(cmd)
        time.sleep(duration)

    def prepare_for_drive(self) -> bool:
        """Prepare for dynamic tests: shift to DRIVE, release EPB."""
        self.log_to_screen("  Preparing for dynamic test...", COLOR_PAIR_CYAN)
        cmd = create_base_command()
        cmd.brake = 20.0  # Hold brake
        cmd.parking_brake = False
        self.send_control_command(cmd)
        if not self.wait_for_condition(
            lambda: self.get_latest_chassis()
            and not self.get_latest_chassis().parking_brake,
            2.0,
            "EPB release",
        ):
            return False

        cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
        self.send_control_command(cmd)
        if not self.wait_for_condition(
            lambda: self.get_latest_chassis()
            and self.get_latest_chassis().gear_location
            == chassis_pb2.Chassis.GEAR_DRIVE,
            3.0,
            "Shift to DRIVE",
        ):
            return False

        self.log_to_screen("  Ready for dynamic test.", COLOR_PAIR_GREEN)
        return True

    def run(self):
        self.safety_thread = threading.Thread(target=self._safety_monitor, daemon=True)
        self.safety_thread.start()

        self.screen.clear()
        self.log_to_screen("Apollo Auto-Tester Professional V2", COLOR_PAIR_CYAN)
        self.log_to_screen(
            f"Running tests up to LEVEL {self.run_level}", COLOR_PAIR_YELLOW
        )
        self.log_to_screen("-" * 60)
        self.log_to_screen(
            "🚨 PRESS [SPACEBAR] AT ANY TIME FOR EMERGENCY STOP 🚨", COLOR_PAIR_RED
        )
        self.log_to_screen("Press [Enter] to start tests...")

        self.screen.nodelay(False)
        while True:
            if self.emergency_stop_triggered.is_set():
                return
            try:
                key = self.screen.getch()
                if key == ord("\n"):
                    break
            except curses.error:
                pass
        self.screen.nodelay(True)

        tests_to_run = [tc for tc in self.test_cases if tc["level"] <= self.run_level]
        if not tests_to_run:
            self.log_to_screen(
                "No tests found for the selected level.", COLOR_PAIR_YELLOW
            )
            return

        for i, case in enumerate(tests_to_run):
            if self.emergency_stop_triggered.is_set():
                self.log_to_screen(
                    "Test suite aborted due to emergency stop.", COLOR_PAIR_RED
                )
                break

            self.reset_to_safe_state()

            self.log_to_screen(
                f"\n[{i+1}/{len(tests_to_run)}] Running (L{case['level']}): {case['name']}..."
            )

            result = {
                "pass": False,
                "details": "Test function did not return a result.",
            }
            try:
                result = case["func"](self)
            except Exception as e:
                result = {"pass": False, "details": f"Caught exception: {e}"}

            status_color = COLOR_PAIR_GREEN if result.get("pass") else COLOR_PAIR_RED
            status_text = "✅ PASS" if result.get("pass") else "❌ FAIL"
            self.log_to_screen(
                f"  -> Result: {status_text} | Details: {result.get('details', 'N/A')}",
                status_color,
            )
            time.sleep(2)

        if not self.emergency_stop_triggered.is_set():
            self.log_to_screen("\n--- All tests completed. ---", COLOR_PAIR_GREEN)

    def cleanup(self):
        self.log_to_screen("Cleaning up and shutting down...", COLOR_PAIR_CYAN)
        self.emergency_stop_triggered.set()
        if self.safety_thread:
            self.safety_thread.join(timeout=1)
        self._perform_emergency_stop()
        cyber.shutdown()
        self.log_to_screen("Shutdown complete.")

    def log_to_screen(self, message: str, color_pair: int = COLOR_PAIR_DEFAULT):
        max_y, max_x = self.screen.getmaxyx()
        if curses.getsyx()[0] >= max_y - 2:
            self.screen.clear()
            curses.setsyx(0, 0)
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        full_message = f"[{timestamp}] {message}"
        y, x = curses.getsyx()
        self.screen.addstr(y, 0, full_message, curses.color_pair(color_pair))
        self.screen.clrtoeol()
        self.screen.move(y + 1, 0)
        self.screen.refresh()


# --- Utility Functions and Test Case Results ---


def fail(details: str, **kwargs) -> Dict[str, Any]:
    return {"pass": False, "details": details, **kwargs}


def success(details: str, **kwargs) -> Dict[str, Any]:
    return {"pass": True, "details": details, **kwargs}


# --- Level 1 Test Cases (Static) ---
def test_l1_driving_mode_transition(runner: TestRunner) -> Dict[str, Any]:
    """L1: TC-FUNC-01: Enter/Exit Autonomous Driving Mode"""
    runner.log_to_screen(
        "  This test requires manual intervention. Please follow instructions.",
        COLOR_PAIR_YELLOW,
    )

    # 1. Enter autonomous driving mode
    runner.log_to_screen("  Step 1: Requesting AUTO_DRIVE mode via control command...")
    cmd = create_base_command()
    # pad_msg is already set to enter auto drive in create_base_command
    runner.send_control_command(cmd)

    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().driving_mode
        == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE,
        timeout=5.0,
        description="transition to COMPLETE_AUTO_DRIVE",
    ):
        return fail(
            "Failed to enter auto drive mode via command.",
            mode_transition_correct=False,
        )
    runner.log_to_screen("  Entered AUTO_DRIVE successfully.", COLOR_PAIR_GREEN)

    # 2. Exit autonomous driving mode (simulate driver takeover)
    runner.log_to_screen(
        "  Step 2: Please press the brake pedal now to disengage...", COLOR_PAIR_YELLOW
    )

    # In real tests, manual operation is required here. Script waits for state change.
    # We can use Chassis's brake_percentage to judge
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().driving_mode
        == chassis_pb2.Chassis.COMPLETE_MANUAL,
        timeout=10.0,
        description="disengagement to COMPLETE_MANUAL",
    ):
        return fail(
            "Did not disengage to manual mode after intervention.",
            mode_transition_correct=True,
            disengage_reason_correct=False,
        )

    runner.log_to_screen("  Disengaged to MANUAL mode successfully.", COLOR_PAIR_GREEN)

    # Check disengagement reason
    chassis = runner.get_latest_chassis()
    # Apollo usually reports MANUAL_INTERVENTION, some OEMs may have more specific error codes
    if chassis.error_code == chassis_pb2.Chassis.MANUAL_INTERVENTION:
        details = "Mode transition and disengagement reason (MANUAL_INTERVENTION) are correct."
        return success(
            details, mode_transition_correct=True, disengage_reason_correct=True
        )
    else:
        details = f"Disengaged, but reason code is {chassis.error_code}, not MANUAL_INTERVENTION."
        runner.log_to_screen(f"  Warning: {details}", COLOR_PAIR_YELLOW)
        return success(
            details, mode_transition_correct=True, disengage_reason_correct=False
        )  # Considered success, but with a warning


def test_l1_steering_performance_static(runner: TestRunner) -> Dict[str, Any]:
    """L1: TC-CTRL-07: Steering Angle Tracking Test (Accuracy and Response Time)"""
    cmd = create_base_command()
    cmd.brake = 20.0
    cmd.parking_brake = False
    runner.send_control_command(cmd)

    target_angle = 50.0  # Test a typical steering angle
    runner.log_to_screen(f"  Commanding steering step to {target_angle}%...")

    cmd.steering_target = 0.0
    runner.send_control_command(cmd)
    time.sleep(2)  # Stabilize at 0

    # Start collecting data
    data_stream = []

    def collector_callback(msg):
        data_stream.append((cyber.time.Time.now().to_sec(), msg.steering_percentage))

    collector_reader = runner.node.create_reader(
        CHASSIS_TOPIC, chassis_pb2.Chassis, collector_callback
    )

    cmd.steering_target = target_angle
    command_time = time.time()
    runner.send_control_command(cmd)

    time.sleep(4.0)  # Collect data for 4 seconds
    collector_reader.shutdown()

    if not data_stream:
        return fail("Failed to collect chassis data for analysis.")

    metrics = calculate_response_metrics(command_time, data_stream, target_angle)

    response_time = metrics["response_time_ms"]
    overshoot = metrics["overshoot_percent"]
    error = metrics["steady_state_error_percent"]

    pass_criteria = {
        "response_time": response_time < 500,  # Static response time < 500ms
        "steady_state_error": abs(error) <= 2.0,  # Steady-state error < 2%
        "overshoot": overshoot < 5.0,  # Overshoot < 5%
    }

    details = f"Response Time: {response_time:.0f}ms, Overshoot: {overshoot:.2f}%, Steady-State Err: {error:.2f}%"

    if all(pass_criteria.values()):
        return success(details, **metrics)
    else:
        failed_items = [k for k, v in pass_criteria.items() if not v]
        return fail(f"{details}. Failed criteria: {', '.join(failed_items)}", **metrics)


def test_l1_epb_toggle(runner: TestRunner) -> Dict[str, Any]:
    """L1: TC-FUNC-04: EPB Engage/Release"""
    cmd = create_base_command()
    cmd.parking_brake = True
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().parking_brake,
        2.0,
        "EPB engage",
    ):
        return fail("EPB engage failed.", engage_correct=False)
    cmd.parking_brake = False
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and not runner.get_latest_chassis().parking_brake,
        2.0,
        "EPB release",
    ):
        return fail("EPB release failed.", engage_correct=True, release_correct=False)
    return success(
        "EPB toggled successfully.", engage_correct=True, release_correct=True
    )


def test_l1_static_gear_shift(runner: TestRunner) -> Dict[str, Any]:
    """L1: TC-FUNC-02: Static Gear Shift"""
    cmd = create_base_command()
    cmd.brake = 50.0
    cmd.parking_brake = False
    for gear_cmd, gear_str in [(1, "D"), (0, "N"), (2, "R"), (3, "P")]:
        if runner.emergency_stop_triggered.is_set():
            return fail("Aborted")
        cmd.gear_location = gear_cmd
        runner.send_control_command(cmd)
        if not runner.wait_for_condition(
            lambda: runner.get_latest_chassis()
            and runner.get_latest_chassis().gear_location == gear_cmd,
            3.0,
            f"shift to {gear_str}",
        ):
            return fail(
                f"Failed to shift to {gear_str}.",
                command_gear=gear_str,
                feedback_gear=runner.get_latest_chassis().gear_location,
            )
    return success("All static gear shifts successful.")


def test_l1_signal_control(runner: TestRunner) -> Dict[str, Any]:
    """L1: TC-SIG-01/02/03/04: Turn Signals, Beams, Hazard, Horn Control"""
    cmd = create_base_command()
    # TC-SIG-01: Turn Signals
    cmd.signal.turn_signal = 1  # TURN_LEFT
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.turn_signal == 1,
        1.0,
        "Left turn signal ON",
    ):
        return fail("Left turn signal failed.", left_turn_ok=False)
    cmd.signal.turn_signal = 3  # TURN_NONE
    runner.send_control_command(cmd)
    cmd.signal.turn_signal = 2  # TURN_RIGHT
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.turn_signal == 2,
        1.0,
        "Right turn signal ON",
    ):
        return fail("Right turn signal failed.", left_turn_ok=True, right_turn_ok=False)
    cmd.signal.turn_signal = 3
    runner.send_control_command(cmd)

    # TC-SIG-02: Beams
    cmd.signal.high_beam = True
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.high_beam,
        1.0,
        "High beam ON",
    ):
        return fail("High beam failed.", high_beam_ok=False)
    cmd.signal.high_beam = False
    cmd.signal.low_beam = True
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.low_beam,
        1.0,
        "Low beam ON",
    ):
        return fail("Low beam failed.", high_beam_ok=True, low_beam_ok=False)
    cmd.signal.low_beam = False
    runner.send_control_command(cmd)

    # TC-SIG-03: Emergency Light
    cmd.signal.emergency_light = True
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.emergency_light,
        1.0,
        "Emergency light ON",
    ):
        return fail("Emergency light failed.", emergency_light_ok=False)
    cmd.signal.emergency_light = False
    runner.send_control_command(cmd)

    # TC-SIG-04: Horn
    cmd.signal.horn = True
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis() and runner.get_latest_chassis().signal.horn,
        1.0,
        "Horn ON",
    ):
        return fail("Horn ON failed.", horn_on_ok=False)
    cmd.signal.horn = False
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and not runner.get_latest_chassis().signal.horn,
        1.0,
        "Horn OFF",
    ):
        return fail("Horn OFF failed.", horn_on_ok=True, horn_off_ok=False)

    return success("All signal controls work as expected.")


# --- Level 2 Test Cases (Low-Speed / Dynamic) ---


def test_l2_throttle_linearity(runner: TestRunner) -> Dict[str, Any]:
    """L2: TC-CTRL-01: Throttle Command Linearity Test (on dyno or safe area)"""
    runner.log_to_screen(
        "WARNING: This test will engage throttle. Ensure vehicle is on a dyno or in a safe, clear area.",
        COLOR_PAIR_YELLOW,
    )
    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

    command_values, feedback_values, errors = [], [], []
    for throttle_cmd in range(0, 81, 10):
        if runner.emergency_stop_triggered.is_set():
            return fail("Aborted")

        cmd.throttle = float(throttle_cmd)
        runner.send_control_command(cmd)
        time.sleep(4.0)  # Stabilization time

        chassis = runner.get_latest_chassis()
        if not chassis:
            return fail("No chassis feedback.")

        feedback = chassis.throttle_percentage
        error = abs(feedback - throttle_cmd)
        runner.log_to_screen(
            f"  CMD: {throttle_cmd:3.0f}% -> FDBK: {feedback:5.1f}%, Error: {error:4.1f}%"
        )

        command_values.append(throttle_cmd)
        feedback_values.append(feedback)
        errors.append(error)

        if error > 2.0:
            return fail(
                f"Error {error:.1f}% > 2.0% at {throttle_cmd}% throttle.",
                command_values=command_values,
                feedback_values=feedback_values,
                max_error=max(errors),
            )

    return success(
        f"Throttle linearity test passed. Max error: {max(errors):.2f}%",
        command_values=command_values,
        feedback_values=feedback_values,
        max_error=max(errors),
    )


def test_l2_brake_linearity(runner: TestRunner) -> Dict[str, Any]:
    """L2: TC-CTRL-04: Brake Command Linearity Test"""
    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    # Let the vehicle move a bit first
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.throttle = 10.0
    runner.send_control_command(cmd)
    time.sleep(2)
    cmd.throttle = 0.0
    runner.send_control_command(cmd)

    command_values, feedback_values, errors = [], [], []
    for brake_cmd in range(0, 81, 10):
        if runner.emergency_stop_triggered.is_set():
            return fail("Aborted")

        cmd.brake = float(brake_cmd)
        runner.send_control_command(cmd)
        time.sleep(4.0)  # Stabilization time

        chassis = runner.get_latest_chassis()
        if not chassis:
            return fail("No chassis feedback.")

        feedback = chassis.brake_percentage
        error = abs(feedback - brake_cmd)
        runner.log_to_screen(
            f"  CMD: {brake_cmd:3.0f}% -> FDBK: {feedback:5.1f}%, Error: {error:4.1f}%"
        )

        command_values.append(brake_cmd)
        feedback_values.append(feedback)
        errors.append(error)

        if error > 3.0:
            return fail(
                f"Error {error:.1f}% > 3.0% at {brake_cmd}% brake.",
                command_values=command_values,
                feedback_values=feedback_values,
                max_error=max(errors),
            )

    return success(
        f"Brake linearity test passed. Max error: {max(errors):.2f}%",
        command_values=command_values,
        feedback_values=feedback_values,
        max_error=max(errors),
    )


def test_l2_throttle_response_time(runner: TestRunner) -> Dict[str, Any]:
    """L2: TC-CTRL-02: Throttle Response Time Test"""
    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    runner.send_control_command(cmd)
    time.sleep(1)

    target_throttle = 50.0
    response_threshold = target_throttle * 0.9  # 45%

    cmd.throttle = target_throttle
    start_time = time.time()
    runner.send_control_command(cmd)

    response_time = -1
    timeout = 2.0
    while time.time() - start_time < timeout:
        chassis = runner.get_latest_chassis()
        if chassis and chassis.throttle_percentage >= response_threshold:
            response_time = (time.time() - start_time) * 1000  # ms
            break
        time.sleep(0.01)

    if response_time < 0:
        return fail(f"Did not reach {response_threshold}% throttle within {timeout}s.")
    if response_time > 200:
        return fail(
            f"Response time {response_time:.0f}ms > 200ms.",
            response_time_ms=response_time,
        )

    return success(
        f"Response time to {response_threshold}% throttle is {response_time:.0f}ms.",
        response_time_ms=response_time,
    )


def test_l2_brake_response_time(runner: TestRunner) -> Dict[str, Any]:
    """L2: TC-CTRL-05: Brake Response Time Test"""
    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    # Bring vehicle to low speed for brake test
    runner.log_to_screen("  Bringing vehicle to low speed for brake test...")
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.speed = 1.5  # Target low speed 1.5 m/s
    runner.send_control_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().speed_mps > 1.0,
        5.0,
        "vehicle to reach >1.0 m/s",
    ):
        return fail("Vehicle failed to reach testing speed.")

    # Stabilize, ensure brake initial state is 0
    cmd.speed = 0.0  # Stop requesting speed, but do not brake
    runner.send_control_command(cmd)
    time.sleep(1)

    target_brake = 50.0
    response_threshold = target_brake * 0.9  # 45%

    runner.log_to_screen(f"  Applying {target_brake}% brake step command...")
    cmd.brake = target_brake
    start_time = time.time()
    runner.send_control_command(cmd)

    response_time = -1
    timeout = 2.0
    while time.time() - start_time < timeout:
        chassis = runner.get_latest_chassis()
        if chassis and chassis.brake_percentage >= response_threshold:
            response_time = (time.time() - start_time) * 1000  # ms
            break
        time.sleep(0.01)

    if response_time < 0:
        return fail(f"Did not reach {response_threshold}% brake within {timeout}s.")

    # According to checklist, brake response time requirement < 180ms
    if response_time > 180:
        return fail(
            f"Response time {response_time:.0f}ms > 180ms.",
            response_time_ms=response_time,
        )

    return success(
        f"Response time to {response_threshold}% brake is {response_time:.0f}ms.",
        response_time_ms=response_time,
    )


def test_l2_speed_control_loop(runner: TestRunner) -> Dict[str, Any]:
    """L2: TC-CTRL-09: Target Speed Closed-Loop Control Test (2 m/s)"""
    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    target_speed = 2.0  # m/s
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

    # --- Acceleration phase ---
    runner.log_to_screen(f"  Stage 1: Accelerating to {target_speed} m/s...")
    cmd.speed = target_speed
    runner.send_control_command(cmd)

    start_time = None
    acceleration_time = -1

    # Wait for vehicle to start moving to start timer
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().speed_mps > 0.1,
        5.0,
        "vehicle to start moving",
    ):
        return fail("Vehicle did not move.")

    start_time = time.time()

    # Wait for vehicle to reach target speed
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().speed_mps >= target_speed,
        10.0,
        f"reach target speed {target_speed} m/s",
    ):
        return fail("Failed to reach target speed.", acceleration_time_sec="timeout")

    acceleration_time = time.time() - start_time
    runner.log_to_screen(
        f"    -> Acceleration time to {target_speed} m/s: {acceleration_time:.2f}s",
        COLOR_PAIR_GREEN,
    )

    # --- Steady speed phase ---
    runner.log_to_screen("  Stage 2: Maintaining speed for 5 seconds...")
    time.sleep(1.0)  # Enter steady state

    speed_readings = []
    stable_start_time = time.time()
    while time.time() - stable_start_time < 5.0:
        speed_readings.append(runner.get_latest_chassis().speed_mps)
        time.sleep(0.1)

    avg_speed = sum(speed_readings) / len(speed_readings)
    speed_error = abs(avg_speed - target_speed)
    runner.log_to_screen(
        f"    -> Average stable speed: {avg_speed:.2f} m/s, Error: {speed_error:.2f} m/s"
    )

    if speed_error > 0.3:  # Allow 0.3m/s steady-state error
        return fail(
            f"Speed control steady-state error ({speed_error:.2f}m/s) exceeds 0.3m/s.",
            acceleration_time_sec=acceleration_time,
            speed_error_mps=speed_error,
        )

    # --- Deceleration phase ---
    runner.log_to_screen("  Stage 3: Decelerating to 0 m/s...")
    cmd.speed = 0.0
    runner.send_control_command(cmd)

    decel_start_time = time.time()
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().speed_mps < 0.1,
        8.0,
        "decelerate to stop",
    ):
        return fail("Failed to decelerate to a stop.", deceleration_time_sec="timeout")

    deceleration_time = time.time() - decel_start_time
    runner.log_to_screen(
        f"    -> Deceleration time: {deceleration_time:.2f}s", COLOR_PAIR_GREEN
    )

    return success(
        f"Accel Time: {acceleration_time:.2f}s, Speed Err: {speed_error:.3f}m/s, Decel Time: {deceleration_time:.2f}s",
        acceleration_time_sec=acceleration_time,
        speed_error_mps=speed_error,
        deceleration_time_sec=deceleration_time,
    )


def test_l2_acceleration_control_loop(runner: TestRunner) -> Dict[str, Any]:
    """L2: TC-CTRL-ACCEL: Acceleration Closed-Loop Control Test (Tracking Accuracy, Response Time, Stability)"""
    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    target_accel = 1.0  # m/s^2, a comfortable starting acceleration
    test_duration = 4.0  # s, duration of acceleration test

    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

    # Set a high speed limit to ensure speed control is not triggered during test
    cmd.speed = 20.0  # m/s (72 km/h)

    runner.log_to_screen(
        f"  Commanding constant acceleration of {target_accel:.2f} m/s^2..."
    )
    cmd.acceleration = target_accel

    # --- Response time measurement ---
    start_time = time.time()
    runner.send_control_command(cmd)

    response_time = -1
    response_threshold = target_accel * 0.9
    timeout = 3.0  # Response timeout
    while time.time() - start_time < timeout:
        chassis = runner.get_latest_chassis()
        if not chassis or not hasattr(chassis, "instantaneous_acceleration"):
            time.sleep(0.01)
            continue

        if chassis.instantaneous_acceleration >= response_threshold:
            response_time = (time.time() - start_time) * 1000  # ms
            runner.log_to_screen(
                f"    -> Response time to 90% target accel: {response_time:.0f}ms",
                COLOR_PAIR_GREEN,
            )
            break
        time.sleep(0.01)

    if response_time < 0:
        return fail(
            f"Acceleration response timeout. Did not reach {response_threshold:.2f} m/s^2."
        )
    if response_time > 400:  # Response time KPI < 400ms
        return fail(
            f"Response time {response_time:.0f}ms > 400ms.",
            response_time_ms=response_time,
        )

    # --- Steady-state tracking accuracy and stability measurement ---
    runner.log_to_screen(
        f"  Maintaining target acceleration for {test_duration}s for stability analysis..."
    )

    accel_readings = []
    stable_start_time = time.time()
    while time.time() - stable_start_time < test_duration:
        if runner.emergency_stop_triggered.is_set():
            return fail("Aborted")

        chassis = runner.get_latest_chassis()
        if chassis and hasattr(chassis, "instantaneous_acceleration"):
            # Only collect data while vehicle is still accelerating
            if chassis.speed_mps > 0.1:
                accel_readings.append(chassis.instantaneous_acceleration)

        # If vehicle unexpectedly stops, terminate test
        if (
            chassis
            and chassis.speed_mps < 0.05
            and time.time() - stable_start_time > 2.0
        ):
            break

        time.sleep(0.05)

    if len(accel_readings) < 20:  # Ensure enough data points for stability analysis
        return fail(
            f"Not enough data points collected for stability analysis (collected {len(accel_readings)})."
        )

    avg_accel = sum(accel_readings) / len(accel_readings)
    accel_std_dev = math.sqrt(
        sum([(x - avg_accel) ** 2 for x in accel_readings]) / len(accel_readings)
    )
    steady_state_error = avg_accel - target_accel

    runner.log_to_screen(
        f"    -> Avg Accel: {avg_accel:.3f} m/s^2 (Error: {steady_state_error:+.3f})",
        COLOR_PAIR_GREEN,
    )
    runner.log_to_screen(
        f"    -> Stability (StdDev): {accel_std_dev:.3f}", COLOR_PAIR_GREEN
    )

    # Pass criteria
    # Steady-state error absolute value < 0.15 m/s^2
    # Stability (standard deviation) < 0.2 m/s^2
    if abs(steady_state_error) > 0.15:
        return fail(
            f"Tracking error {abs(steady_state_error):.3f} > 0.15 m/s^2.",
            response_time_ms=response_time,
            steady_state_error=steady_state_error,
            stability_stddev=accel_std_dev,
        )
    if accel_std_dev > 0.2:
        return fail(
            f"Poor stability. StdDev {accel_std_dev:.3f} > 0.2.",
            response_time_ms=response_time,
            steady_state_error=steady_state_error,
            stability_stddev=accel_std_dev,
        )

    return success(
        "Acceleration control tracking test passed.",
        response_time_ms=response_time,
        steady_state_error=round(steady_state_error, 3),
        stability_stddev=round(accel_std_dev, 3),
    )


def test_l2_gear_protection(runner: TestRunner) -> Dict[str, Any]:
    """L2: TC-FUNC-03: Gear Protection Logic Test"""
    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    # Let vehicle drive at 5km/h (about 1.4m/s)
    target_speed = 1.4
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.speed = target_speed
    runner.send_control_command(cmd)

    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().speed_mps > 1.0,
        5.0,
        "vehicle to reach >1.0 m/s",
    ):
        return fail("Vehicle failed to reach testing speed.")

    runner.log_to_screen(
        f"  Vehicle at {runner.get_latest_chassis().speed_mps:.1f} m/s. Attempting to shift to REVERSE..."
    )

    # Try to shift to reverse
    cmd.gear_location = chassis_pb2.Chassis.GEAR_REVERSE
    runner.send_control_command(cmd)
    time.sleep(2.0)  # Observation time

    chassis = runner.get_latest_chassis()
    if chassis.gear_location == chassis_pb2.Chassis.GEAR_REVERSE:
        return fail(
            "Gear shifted to REVERSE while moving forward.",
            gear_remains_unchanged=False,
        )

    # Check error code
    error_reported = chassis.error_code != chassis_pb2.Chassis.NO_ERROR
    runner.log_to_screen(
        f"  Gear remained in DRIVE. Error code: {chassis.error_code} ({'Reported' if error_reported else 'Not Reported'})"
    )

    if not error_reported:
        runner.log_to_screen(
            "  Warning: Gear shift was rejected, but no specific error code was reported.",
            COLOR_PAIR_YELLOW,
        )

    return success(
        "Gear shift to REVERSE was correctly rejected while moving forward.",
        gear_remains_unchanged=True,
        error_code_reported=str(chassis.error_code),
    )


# --- Level 3 Test Cases (High-Dynamic) ---
def test_l3_emergency_brake(runner: TestRunner) -> Dict[str, Any]:
    """L3: TC-CTRL-06: Emergency Brake Test (30km/h)"""
    runner.log_to_screen(
        "WARNING: High-speed braking test starting. ENSURE HUGE SAFE AREA!",
        COLOR_PAIR_YELLOW,
    )
    time.sleep(5)

    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    target_speed_kph = 30
    target_speed_mps = target_speed_kph / 3.6

    # Accelerate to 30km/h
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.speed = target_speed_mps + 1  # Set slightly higher to ensure reaching target
    runner.send_control_command(cmd)

    runner.log_to_screen(f"  Accelerating to {target_speed_kph} km/h...")
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().speed_mps >= target_speed_mps,
        10.0,
        f"reach {target_speed_kph} km/h",
    ):
        return fail(
            f"Failed to reach {target_speed_kph} km/h for braking test. Max speed: {runner.get_latest_chassis().speed_mps * 3.6:.1f} km/h"
        )

    runner.log_to_screen(
        f"  At speed {runner.get_latest_chassis().speed_mps * 3.6:.1f} km/h. Triggering 100% BRAKE!",
        COLOR_PAIR_RED,
    )

    # Send emergency brake command
    cmd.speed = 0.0
    cmd.throttle = 0.0
    cmd.brake = 100.0
    start_time = time.time()
    runner.send_control_command(cmd)

    timeout = 2.0
    time_to_max_brake = -1.0
    max_brake_feedback = 0.0

    while time.time() - start_time < timeout:
        chassis = runner.get_latest_chassis()
        if not chassis:
            continue

        max_brake_feedback = max(max_brake_feedback, chassis.brake_percentage)

        # Check if 95% brake is reached
        if time_to_max_brake < 0 and chassis.brake_percentage >= 95.0:
            time_to_max_brake = (time.time() - start_time) * 1000  # ms

        # Exit when vehicle stops
        if chassis.speed_mps < 0.1:
            break
        time.sleep(0.01)

    if time_to_max_brake < 0:
        return fail(
            f"Did not reach 95% brake within {timeout}s. Max brake feedback: {max_brake_feedback:.1f}%",
            time_to_max_brake_ms=time_to_max_brake,
            max_brake_feedback=max_brake_feedback,
        )

    if time_to_max_brake > 300:
        return fail(
            f"Time to max brake {time_to_max_brake:.0f}ms > 300ms.",
            time_to_max_brake_ms=time_to_max_brake,
            max_brake_feedback=max_brake_feedback,
        )

    return success(
        f"Emergency brake OK. Time to 95% brake: {time_to_max_brake:.0f}ms. Max feedback: {max_brake_feedback:.1f}%",
        time_to_max_brake_ms=time_to_max_brake,
        max_brake_feedback=max_brake_feedback,
    )


def test_l3_staged_acceleration(runner: TestRunner) -> Dict[str, Any]:
    """L3: TC-CTRL-10: Staged Acceleration Performance Test (0-30, 30-60 km/h)"""
    runner.log_to_screen(
        "WARNING: High-speed acceleration test. ENSURE HUGE SAFE AREA!",
        COLOR_PAIR_YELLOW,
    )
    time.sleep(5)

    if not runner.prepare_for_drive():
        return fail("Failed to prepare for drive.")

    # Target speeds (km/h) and corresponding m/s
    targets_kph = [30, 60]
    targets_mps = [k / 3.6 for k in targets_kph]
    results = {}

    # Set maximum acceleration
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.acceleration = 3.0  # Request high acceleration m/s^2
    runner.send_control_command(cmd)

    runner.log_to_screen("  Starting full acceleration...")

    # Wait for vehicle to start moving to start total timer
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().speed_mps > 0.1,
        5.0,
        "vehicle to start moving",
    ):
        return fail("Vehicle did not move.")

    start_time = time.time()
    last_target_time = start_time

    for i, target_mps in enumerate(targets_mps):
        prev_kph = targets_kph[i - 1] if i > 0 else 0
        current_kph = targets_kph[i]

        runner.log_to_screen(f"  ...timing for {prev_kph}-{current_kph} km/h...")

        if not runner.wait_for_condition(
            lambda: runner.get_latest_chassis()
            and runner.get_latest_chassis().speed_mps >= target_mps,
            timeout=15.0,
            description=f"reach {current_kph} km/h",
        ):
            max_speed = runner.get_latest_chassis().speed_mps * 3.6
            return fail(
                f"Timeout waiting to reach {current_kph} km/h. Max speed: {max_speed:.1f} km/h"
            )

        current_target_time = time.time()
        segment_time = current_target_time - last_target_time
        results[f"time_{prev_kph}_to_{current_kph}_kph"] = round(segment_time, 2)
        last_target_time = current_target_time

        runner.log_to_screen(
            f"    -> {prev_kph}-{current_kph} km/h took: {segment_time:.2f}s",
            COLOR_PAIR_GREEN,
        )

    return success(
        f"Staged acceleration test passed. 0-30: {results.get('time_0_to_30_kph')}s, 30-60: {results.get('time_30_to_60_kph')}s",
        **results,
    )


def test_l3_staged_braking_performance(runner: TestRunner) -> Dict[str, Any]:
    """L3: TC-CTRL-06: Staged Braking Performance Test (at 60km/h, 50% and 100% brake)"""
    runner.log_to_screen(
        "WARNING: High-speed braking test. ENSURE HUGE SAFE AREA!", COLOR_PAIR_YELLOW
    )
    time.sleep(5)

    initial_speed_kph = 60
    initial_speed_mps = initial_speed_kph / 3.6
    brake_levels = [50.0, 100.0]
    results = {}

    for brake_cmd in brake_levels:
        runner.log_to_screen(
            f"\n--- Testing {brake_cmd}% Braking from {initial_speed_kph} km/h ---",
            COLOR_PAIR_CYAN,
        )
        if runner.emergency_stop_triggered.is_set():
            return fail("Aborted")

        if not runner.prepare_for_drive():
            return fail(f"Failed to prepare for drive for {brake_cmd}% test.")

        # Accelerate to target speed
        cmd = create_base_command()
        cmd.parking_brake = False
        cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
        cmd.speed = (
            initial_speed_mps + 2
        )  # Set slightly higher to ensure reaching target
        runner.send_control_command(cmd)

        if not runner.wait_for_condition(
            lambda: runner.get_latest_chassis()
            and runner.get_latest_chassis().speed_mps >= initial_speed_mps,
            15.0,
            f"reach {initial_speed_kph} km/h",
        ):
            return fail(f"Failed to reach speed for {brake_cmd}% braking test.")

        # Trigger braking
        runner.log_to_screen(
            f"  At speed. Triggering {brake_cmd}% BRAKE!", COLOR_PAIR_RED
        )
        cmd.speed = 0.0
        cmd.throttle = 0.0
        cmd.brake = brake_cmd
        runner.send_control_command(cmd)

        start_time = time.time()
        # Wait for vehicle to stop
        if not runner.wait_for_condition(
            lambda: runner.get_latest_chassis()
            and runner.get_latest_chassis().speed_mps < 0.1,
            10.0,
            "decelerate to stop",
        ):
            return fail(
                f"Vehicle failed to stop during {brake_cmd}% braking test.",
                braking_time="timeout",
            )

        braking_time = time.time() - start_time
        results[f"braking_time_from_60kph_{int(brake_cmd)}pct"] = round(braking_time, 2)
        runner.log_to_screen(
            f"  -> {brake_cmd}% braking time from {initial_speed_kph} km/h: {braking_time:.2f}s",
            COLOR_PAIR_GREEN,
        )

        runner.reset_to_safe_state(duration=2)  # Reset after each loop

    return success(
        f"Braking tests passed. 50%: {results.get('braking_time_from_60kph_50pct')}s, 100%: {results.get('braking_time_from_60kph_100pct')}s",
        **results,
    )


def main(screen):
    parser = argparse.ArgumentParser(
        description="Run Automated Chassis Tests for Apollo."
    )
    parser.add_argument(
        "--level",
        type=int,
        default=1,
        choices=[1, 2, 3],
        help="Specify the maximum test level to run.",
    )
    args = parser.parse_args()

    runner = TestRunner(screen, args.level)

    # --- Register all test cases ---
    # Level 1: Static and low-risk tests
    runner.register_test_case(
        test_l1_driving_mode_transition, "TC-FUNC-01: Driving Mode Transition", 1
    )
    runner.register_test_case(
        test_l1_steering_performance_static,
        "TC-CTRL-07: Static Steering Performance",
        1,
    )
    runner.register_test_case(test_l1_epb_toggle, "TC-FUNC-04: EPB Toggle", 1)
    runner.register_test_case(
        test_l1_static_gear_shift, "TC-FUNC-02: Static Gear Shift", 1
    )
    runner.register_test_case(
        test_l1_signal_control, "TC-SIG-ALL: All Signal Controls (Lights, Horn)", 1
    )

    # Level 2: Low-speed dynamic/characteristic tests
    runner.register_test_case(
        test_l2_speed_control_loop, "TC-CTRL-09: Speed Control Loop", 2
    )
    runner.register_test_case(
        test_l2_acceleration_control_loop,
        "TC-CTRL-ACCEL: Acceleration Control Loop",
        2,
    )
    runner.register_test_case(
        test_l2_throttle_linearity, "TC-CTRL-01: Throttle Linearity", 2
    )
    runner.register_test_case(test_l2_brake_linearity, "TC-CTRL-04: Brake Linearity", 2)
    runner.register_test_case(
        test_l2_throttle_response_time, "TC-CTRL-02: Throttle Response Time", 2
    )
    runner.register_test_case(
        test_l2_brake_response_time, "TC-CTRL-05: Brake Response Time", 2
    )
    runner.register_test_case(
        test_l2_gear_protection, "TC-FUNC-03: Gear Protection Logic", 2
    )

    # Level 3: High-dynamic/performance tests
    runner.register_test_case(
        test_l3_staged_acceleration, "TC-CTRL-10: Staged Acceleration Performance", 3
    )
    runner.register_test_case(
        test_l3_staged_braking_performance, "TC-CTRL-06: Staged Braking Performance", 3
    )
    runner.register_test_case(
        test_l3_emergency_brake, "TC-CTRL-06: Emergency Braking from 30km/h", 3
    )

    try:
        runner.run()
    except Exception as e:
        runner.log_to_screen(
            f"FATAL: An unhandled exception occurred in main: {e}", COLOR_PAIR_RED
        )
    finally:
        runner.log_to_screen("Test sequence finished. Cleaning up...", COLOR_PAIR_CYAN)
        runner.cleanup()
        time.sleep(2)
        with runner.screen_lock:
            curses.nocbreak()
            screen.keypad(False)
            curses.echo()
            curses.endwin()
        print("Auto-Tester has shut down gracefully.")


if __name__ == "__main__":
    if not cyber.ok():
        print("ERROR: Apollo Cyber RT environment is not initialized.")
        print("Please run 'source /apollo/cyber/setup.bash' first.")
        exit(1)
    try:
        curses.wrapper(main)
    except curses.error as e:
        print(
            f"Failed to start curses wrapper. Your terminal might not support it. Error: {e}"
        )
    except Exception as e:
        print(f"A critical error occurred: {e}")
