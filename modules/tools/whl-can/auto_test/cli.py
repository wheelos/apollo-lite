#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
A robust test runner for validating Apollo's chassis control interface.
Implements V-Model testing tiers (Static -> Dynamic Low -> Dynamic High).

Key Features:
- 100Hz Command Heartbeat (prevents chassis timeouts)
- Safety Watchdog (monitors thread health & chassis errors)
- Tiered Test Execution (L1/L2/L3)
- Real-time Curses UI
"""

import argparse
import time
import curses
import sys

try:
    from cyber.python.cyber_py3 import cyber
except ImportError:
    print("FATAL: Apollo Cyber RT python modules not found.")
    print("Please ensure you are in the Apollo docker environment.")
    sys.exit(1)

from core.runner import TestRunner
from core.ui import UIHandler
from config.manager import ConfigManager

from tests.l1_static import (
    test_l1_driving_mode_transition,
    test_l1_steering_performance_static,
    test_l1_static_max_steering_rate,
    test_l1_static_brake_consistency,
    test_l1_epb_toggle,
    test_l1_static_gear_shift,
    test_l1_signal_control,
)
from tests.l2_dynamic_low import (
    test_l2_speed_control_loop,
    test_l2_throttle_linearity,
    test_l2_brake_linearity,
    test_l2_throttle_response_time,
    test_l2_brake_response_time,
    test_l2_gear_protection,
    test_l2_mode_protection,
)
from tests.l3_dynamic_high import (
    test_l3_staged_acceleration,
    test_l3_staged_braking_performance,
    test_l3_emergency_brake,
)


def main(stdscr):
    # 1. Parse Arguments
    parser = argparse.ArgumentParser(description="Apollo Chassis Auto-Tester")
    parser.add_argument(
        "--level",
        type=int,
        default=1,
        choices=[1, 2, 3],
        help="Test Level (1: Static, 2: Low-Speed, 3: High-Dynamic)",
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/default.yaml",
        help="Path to test configuration file",
    )
    parser.add_argument(
        "--allow-high-risk",
        action="store_true",
        help="Enable high-risk L3 case (100km/h emergency brake)",
    )
    # Curses wrapper passes extra args awkwardly, so we parse known args only
    # In a real CLI, we might handle argv differently before curses.wrapper
    args, _ = parser.parse_known_args()

    # 2. Initialize Components
    config = ConfigManager(args.config)
    ui = UIHandler(stdscr)
    runner = TestRunner(ui, config, args.level)

    # 3. Register Test Cases
    # --- Level 1: Static / Functional ---
    runner.register_test_case(
        test_l1_driving_mode_transition, "TC-FUNC-01: Mode Transition", 1
    )
    runner.register_test_case(
        test_l1_steering_performance_static, "TC-CTRL-07: Static Steer Step", 1
    )
    runner.register_test_case(
        test_l1_static_max_steering_rate, "TC-CTRL-08: Static Max Steer Test", 1
    )
    runner.register_test_case(
        test_l1_static_brake_consistency, "TC-CTRL-04-S: Static Brake Consistency", 1
    )
    runner.register_test_case(test_l1_epb_toggle, "TC-FUNC-04: EPB Toggle", 1)
    runner.register_test_case(
        test_l1_static_gear_shift, "TC-FUNC-02: Static Gear Shift", 1
    )
    runner.register_test_case(test_l1_signal_control, "TC-SIG-ALL: Body Signals", 1)

    # --- Level 2: Dynamic Low Speed ---
    runner.register_test_case(
        test_l2_speed_control_loop, "TC-CTRL-09: Low Speed Loop", 2
    )
    runner.register_test_case(
        test_l2_throttle_linearity, "TC-CTRL-01: Throttle Linearity", 2
    )
    runner.register_test_case(test_l2_brake_linearity, "TC-CTRL-04: Brake Linearity", 2)
    runner.register_test_case(
        test_l2_throttle_response_time, "TC-CTRL-02: Throttle Resp.", 2
    )
    runner.register_test_case(test_l2_brake_response_time, "TC-CTRL-05: Brake Resp.", 2)
    runner.register_test_case(test_l2_gear_protection, "TC-FUNC-03: Gear Protection", 2)
    runner.register_test_case(test_l2_mode_protection, "TC-FUNC-06: Mode Protection", 2)

    # --- Level 3: Dynamic High Performance ---
    runner.register_test_case(
        test_l3_staged_acceleration, "TC-CTRL-10: Staged Accel", 3
    )
    runner.register_test_case(
        test_l3_staged_braking_performance, "TC-CTRL-06: Staged Brake", 3
    )
    if args.allow_high_risk:
        runner.register_test_case(
            test_l3_emergency_brake, "TC-SAFETY-01: Emergency Brake", 3
        )

    # 4. Execution Loop
    try:
        runner.start()  # Starts Heartbeat & Watchdog
        runner.run_sequence()
    except KeyboardInterrupt:
        ui.log("User interrupted (Ctrl+C). Stopping...", "RED")
    except Exception as e:
        ui.log(f"CRITICAL ERROR: {str(e)}", "RED")
        import traceback

        ui.log(traceback.format_exc(), "RED")
        time.sleep(3)  # Give user time to see the error
    finally:
        ui.log("Shutting down...", "CYAN")
        runner.cleanup()


if __name__ == "__main__":
    # Environment Check
    if not cyber.ok():
        print("Initializing Cyber RT...")
        cyber.init()

    try:
        # Using curses.wrapper to handle terminal setup/teardown automatically
        curses.wrapper(main)
    except Exception as e:
        print(f"Failed to initialize UI: {e}")
        cyber.shutdown()
