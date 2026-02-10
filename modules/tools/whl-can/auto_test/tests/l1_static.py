import time
from util import success, fail, create_base_command
from modules.common_msgs.chassis_msgs import chassis_pb2


def test_l1_driving_mode_transition(runner):
    """TC-FUNC-01: Driving Mode Transition"""
    runner.ui.log("Step 1: Requesting AUTO...", "CYAN")
    cmd = create_base_command()
    runner.update_command(cmd)

    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis().driving_mode
        == chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE,
        5.0,
        "Enter Auto",
    ):
        return fail("Failed to enter Auto Mode")

    runner.ui.log("Step 2: Press Brake to disengage...", "YELLOW")
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis().driving_mode
        == chassis_pb2.Chassis.COMPLETE_MANUAL,
        15.0,
        "Manual Disengagement",
    ):
        return fail("Failed to disengage")

    return success("Mode transition verified")


def test_l1_steering_performance_static(runner):
    """TC-CTRL-07: Static Steering Tracking"""
    cmd = create_base_command()
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.parking_brake = False
    runner.update_command(cmd)

    target = 40.0
    runner.ui.log(f"Commanding Steering {target}%...", "CYAN")

    cmd.steering_target = target
    runner.update_command(cmd)

    # Wait for steady state
    time.sleep(3.0)

    chassis = runner.get_latest_chassis()
    if not chassis:
        return fail("No chassis feedback")
    actual = chassis.steering_percentage
    err = abs(actual - target)

    if err < 2.0:
        return success(f"Target: {target}%, Actual: {actual:.1f}%")
    else:
        return fail(f"Error {err:.1f}% too high")


def test_l1_static_max_steering_rate(runner):
    """
    TC-CTRL-08: Static Max Steering Rate (Dry Steering)
    Verify the system stability under extreme load conditions when turning in place
    (without overheating or current overload protection).
    """
    cmd = create_base_command()
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.parking_brake = False
    runner.update_command(cmd)

    # 1. Get back on track first
    runner.ui.log("Centering Steering...", "CYAN")
    cmd.steering_target = 0.0
    runner.update_command(cmd)
    time.sleep(3.0)

    # 2. Full sweep from full left to full right
    max_rate_check_points = [-100.0, 100.0]

    for target in max_rate_check_points:
        if runner.estop_triggered.is_set():
            return fail("Aborted")

        runner.ui.log(f"High Speed Turn to {target}%...", "YELLOW")

        # Record start time
        start_time = time.time()

        cmd.steering_target = target
        # Assuming protocol supports setting steering rate, set to 100%
        # cmd.steering_rate = 100.0
        runner.update_command(cmd)

        # Allow sufficient timeout
        timeout = 5.0
        success_flag = False

        while time.time() - start_time < timeout:
            chassis = runner.get_latest_chassis()
            actual = chassis.steering_percentage

            # Check for error codes (such as EPS overheating).
            if chassis.error_code != chassis_pb2.Chassis.NO_ERROR:
                return fail(f"Chassis Error during turn: {chassis.error_code}")

            # Arrival determination: Error < 1%
            if abs(actual - target) < 1.0:
                duration = time.time() - start_time
                runner.ui.log(f"  -> Reached in {duration:.2f}s", "GREEN")
                success_flag = True
                break

            time.sleep(0.1)

        if not success_flag:
            return fail(f"Timeout: Steering failed to reach {target}% under load")

    # 3. Return to center
    cmd.steering_target = 0.0
    runner.update_command(cmd)

    return success("Max Steering Rate Stability OK")


def test_l1_static_brake_consistency(runner):
    """
    TC-CTRL-04-S: Static Brake Command Consistency
    Verify if the drive-by-wire brake accurately responds to commands when the vehicle is stationary.
    """
    # 1. Safety Initialization: P gear + EPB released (testing Service Brake)
    cmd = create_base_command()
    cmd.gear_location = chassis_pb2.Chassis.GEAR_PARKING
    cmd.parking_brake = False
    runner.update_command(cmd)

    runner.ui.log("Setup: Park Gear, EPB Released. Testing Service Brake...", "YELLOW")
    time.sleep(2.0)

    # 2. Test points: cover low, medium, and high ranges
    test_points = [0, 30, 70, 100]
    tolerance = 1.0  # Allow 1% error
    try:
        for target in test_points:
            # Emergency stop check
            if runner.estop_triggered.is_set():
                return fail("User Aborted")

            runner.ui.log(f"Commanding Brake: {target}%...", "CYAN")
            cmd.brake = float(target)
            runner.update_command(cmd)

            # Wait for hydraulic build-up to stabilize (usually quick when stationary, 2s is sufficient)
            time.sleep(2.0)

            # Get feedback
            chassis = runner.get_latest_chassis()
            if not chassis:
                return fail("No chassis feedback")

            actual = chassis.brake_percentage
            diff = abs(actual - target)

            # 3. Verify consistency
            runner.ui.log(f"  -> Feedback: {actual:.1f}% (Diff: {diff:.1f}%)", "WHITE")

            if diff > tolerance:
                return fail(f"Mismatch at {target}%: Cmd={target}, Act={actual:.1f}")

    finally:
        # Restore safe state
        runner.reset_to_safe_state()

    return success("Static Brake Consistency OK")


def test_l1_static_gear_shift(runner):
    """TC-FUNC-02: Static Gear Shift"""
    cmd = create_base_command()
    cmd.brake = 50.0

    for gear in [
        chassis_pb2.Chassis.GEAR_REVERSE,
        chassis_pb2.Chassis.GEAR_DRIVE,
        chassis_pb2.Chassis.GEAR_PARKING,
    ]:
        cmd.gear_location = gear
        runner.update_command(cmd)
        if not runner.wait_for_condition(
            lambda: runner.get_latest_chassis().gear_location == gear,
            3.0,
            f"Shift Gear {gear}",
        ):
            return fail(f"Failed to shift to Gear {gear}")
    return success("Gear shift OK")


def test_l1_epb_toggle(runner):
    """TC-FUNC-04: EPB Toggle"""
    cmd = create_base_command()

    cmd.parking_brake = True
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis().parking_brake, 3.0, "EPB Engage"
    ):
        return fail("EPB Engage Failed")

    cmd.parking_brake = False
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: not runner.get_latest_chassis().parking_brake, 3.0, "EPB Release"
    ):
        return fail("EPB Release Failed")
    return success("EPB OK")


def test_l1_signal_control(runner):
    """TC-SIG-01~04: Turn Signals, Beams, Hazard, Horn"""
    cmd = create_base_command()

    # TC-SIG-01: Turn Signals
    cmd.signal.turn_signal = 1  # LEFT
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.turn_signal == 1,
        1.0,
        "Left turn signal",
    ):
        return fail("Left Turn Signal Failed")

    cmd.signal.turn_signal = 2  # RIGHT
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.turn_signal == 2,
        1.0,
        "Right turn signal",
    ):
        return fail("Right Turn Signal Failed")

    cmd.signal.turn_signal = 3  # NONE
    runner.update_command(cmd)

    # TC-SIG-02: High/Low Beam
    cmd.signal.high_beam = True
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.high_beam,
        1.0,
        "High beam",
    ):
        return fail("High Beam Failed")
    cmd.signal.high_beam = False
    cmd.signal.low_beam = True
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.low_beam,
        1.0,
        "Low beam",
    ):
        return fail("Low Beam Failed")
    cmd.signal.low_beam = False
    runner.update_command(cmd)

    # TC-SIG-03: Hazard
    cmd.signal.emergency_light = True
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and runner.get_latest_chassis().signal.emergency_light,
        1.0,
        "Emergency light",
    ):
        return fail("Emergency Light Failed")
    cmd.signal.emergency_light = False
    runner.update_command(cmd)

    # TC-SIG-04: Horn
    cmd.signal.horn = True
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis() and runner.get_latest_chassis().signal.horn,
        1.0,
        "Horn ON",
    ):
        return fail("Horn Failed")

    cmd.signal.horn = False
    runner.update_command(cmd)
    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis()
        and not runner.get_latest_chassis().signal.horn,
        1.0,
        "Horn OFF",
    ):
        return fail("Horn Release Failed")

    return success("Signals OK")
