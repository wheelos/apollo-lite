import time
from util import success, fail, create_base_command
from core.reporter import DataRecorder

from modules.common_msgs.chassis_msgs import chassis_pb2


def _run_linearity_sequence(
    runner,
    cmd,
    test_points,
    hold_sec,
    tolerance,
    recorder,
    feedback_getter,
    command_setter,
):
    for target in test_points:
        if runner.estop_triggered.is_set():
            return fail("Aborted")

        command_setter(cmd, float(target))
        runner.update_command(cmd)

        start_hold = time.time()
        while time.time() - start_hold < hold_sec:
            chassis = runner.get_latest_chassis()
            if chassis:
                recorder.record(float(target), feedback_getter(chassis), chassis.speed_mps)
            time.sleep(0.01)

        chassis = runner.get_latest_chassis()
        if not chassis:
            return fail("No chassis feedback")

        actual = feedback_getter(chassis)
        if abs(actual - target) > tolerance:
            return fail(f"Linearity error at {target}%")

    return success("Linearity OK")


def _run_step_response_test(
    runner,
    recorder,
    cmd,
    target,
    timeout_sec,
    feedback_getter,
    command_setter,
):
    recorder.start()
    command_setter(cmd, target)
    start_time = time.time()
    runner.update_command(cmd)

    t90 = -1.0

    try:
        while time.time() - start_time < timeout_sec:
            chassis = runner.get_latest_chassis()
            if chassis:
                actual = feedback_getter(chassis)
                recorder.record(target, actual, chassis.speed_mps)
                if t90 < 0 and actual >= target * 0.9:
                    t90 = (time.time() - start_time) * 1000
            time.sleep(0.01)
    finally:
        recorder.stop()

    if t90 < 0:
        return fail("Response Timeout")
    if t90 > 400:
        return fail(f"Response too slow: {t90:.0f}ms")
    return success(f"Response Time: {t90:.0f}ms")


def test_l2_throttle_linearity(runner):
    """TC-CTRL-01: Throttle Linearity (0-30%)"""
    if not runner.prepare_for_drive():
        return fail("Prep Failed")

    recorder = DataRecorder("L2_Throttle_Linearity")

    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

    recorder.start()
    try:
        result = _run_linearity_sequence(
            runner=runner,
            cmd=cmd,
            test_points=[0, 10, 20, 30],
            hold_sec=3.0,
            tolerance=3.0,
            recorder=recorder,
            feedback_getter=lambda msg: msg.throttle_percentage,
            command_setter=lambda c, v: setattr(c, "throttle", v),
        )
    finally:
        recorder.stop()
        path = recorder.save_and_plot("Linearity Test", "Throttle %", "throttle_lin")
        runner.ui.log(f"Plot saved: {path}", "CYAN")
    return result


def test_l2_brake_linearity(runner):
    """TC-CTRL-04: Brake Linearity (0-30%)"""
    if not runner.prepare_for_drive():
        return fail("Prep Failed")

    recorder = DataRecorder("L2_Brake_Linearity")

    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE

    # Build minimal speed before brake steps
    cmd.throttle = 10.0
    runner.update_command(cmd)
    time.sleep(2.0)
    cmd.throttle = 0.0
    runner.update_command(cmd)

    recorder.start()
    try:
        result = _run_linearity_sequence(
            runner=runner,
            cmd=cmd,
            test_points=[0, 10, 20, 30],
            hold_sec=3.0,
            tolerance=3.0,
            recorder=recorder,
            feedback_getter=lambda msg: msg.brake_percentage,
            command_setter=lambda c, v: setattr(c, "brake", v),
        )
    finally:
        recorder.stop()
        path = recorder.save_and_plot("Linearity Test", "Brake %", "brake_lin")
        runner.ui.log(f"Plot saved: {path}", "CYAN")
    return result


def test_l2_brake_response_time(runner):
    """TC-CTRL-05: Brake Response Time (Step 0->20%)"""
    cmd = create_base_command()
    cmd.parking_brake = False
    runner.update_command(cmd)
    time.sleep(1.0)

    recorder = DataRecorder("L2_Brake_Response")

    target = 20.0
    result = _run_step_response_test(
        runner=runner,
        recorder=recorder,
        cmd=cmd,
        target=target,
        timeout_sec=2.0,
        feedback_getter=lambda msg: msg.brake_percentage,
        command_setter=lambda c, v: setattr(c, "brake", v),
    )
    if not result["pass"]:
        path = recorder.save_and_plot("Step Response", "Brake %", "brake_resp")
        return fail(result["details"], plot=path)
    path = recorder.save_and_plot("Step Response", "Brake %", "brake_resp")
    return success(result["details"], plot=path)


def test_l2_speed_control_loop(runner):
    """TC-CTRL-09: Low Speed Loop (2 m/s)"""
    if not runner.prepare_for_drive():
        return fail("Prep Failed")

    recorder = DataRecorder("L2_Speed_Loop")
    recorder.start()

    target_speed = 2.0
    cmd = create_base_command()
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.parking_brake = False
    cmd.speed = target_speed
    runner.update_command(cmd)

    start = time.time()
    try:
        while time.time() - start < 10.0:
            chassis = runner.get_latest_chassis()
            if chassis:
                recorder.record(target_speed, chassis.speed_mps)
            if runner.estop_triggered.is_set():
                break
            time.sleep(0.01)

        chassis = runner.get_latest_chassis()
        if not chassis:
            return fail("No chassis feedback")
        final_speed = chassis.speed_mps
        err = abs(final_speed - target_speed)

        # Stop
        cmd.speed = 0
        cmd.brake = 20.0
        runner.update_command(cmd)

    finally:
        recorder.stop()
        recorder.save_and_plot("Speed Control", "Speed (m/s)", "speed_loop")

    if err > 0.3:
        return fail(f"Speed Error {err:.2f} m/s")
    return success(f"Stable Speed: {final_speed:.2f} m/s")


def test_l2_gear_protection(runner):
    """TC-FUNC-03: Gear Protection (Reverse while moving)"""
    if not runner.prepare_for_drive():
        return fail("Prep Failed")

    # 1. Move forward > 1m/s
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.speed = 1.5
    runner.update_command(cmd)

    try:
        if not runner.wait_for_chassis_condition(
            lambda msg: msg.speed_mps > 1.0, 5.0, "Speed > 1m/s"
        ):
            return fail("Could not build speed")

        # 2. Try Reverse
        runner.ui.log("Attempting Shift to REVERSE...", "YELLOW")
        cmd.gear_location = chassis_pb2.Chassis.GEAR_REVERSE
        runner.update_command(cmd)
        time.sleep(2.0)

        chassis = runner.get_latest_chassis()
        if not chassis:
            return fail("No chassis feedback")

        final_gear = chassis.gear_location

        if final_gear == chassis_pb2.Chassis.GEAR_REVERSE:
            return fail("Safety Failure: Shifted to R while moving!")
        return success("Gear Protection Active")
    finally:
        # Stop
        cmd.speed = 0
        cmd.brake = 50.0
        cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
        runner.update_command(cmd)


def test_l2_mode_protection(runner):
    """TC-FUNC-06: Motion Mode Protection (Park while moving)"""
    if not runner.prepare_for_drive():
        return fail("Prep Failed")

    # 1. Move forward
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.speed = 2.0
    runner.update_command(cmd)

    if not runner.wait_for_chassis_condition(
        lambda msg: msg.speed_mps > 1.5, 5.0, "Speed > 1.5m/s"
    ):
        return fail("Could not build speed")

    # 2. Try Shift to PARK
    runner.ui.log("Attempting Shift to PARK while moving...", "YELLOW")
    cmd.gear_location = chassis_pb2.Chassis.GEAR_PARKING
    runner.update_command(cmd)
    time.sleep(2.0)

    chassis = runner.get_latest_chassis()
    if chassis is None:
        # Cleanup before failing to keep runner in a safe state
        cmd.speed = 0
        cmd.brake = 50.0
        cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
        runner.update_command(cmd)
        return fail("No chassis feedback available")

    final_gear = chassis.gear_location
    # Cleanup
    cmd.speed = 0
    cmd.brake = 50.0
    cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
    runner.update_command(cmd)

    if final_gear == chassis_pb2.Chassis.GEAR_PARKING:
        return fail("Safety Failure: Shifted to PARK while moving!")

    return success("Park Inhibition Active")


def test_l2_throttle_response_time(runner):
    """TC-CTRL-02: Throttle Response Time (Step 0->20%)"""
    if not runner.prepare_for_drive():
        return fail("Prep Failed")

    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.throttle = 0.0
    runner.update_command(cmd)
    time.sleep(0.5)

    recorder = DataRecorder("L2_Throttle_Response")

    target = 20.0
    result = _run_step_response_test(
        runner=runner,
        recorder=recorder,
        cmd=cmd,
        target=target,
        timeout_sec=2.0,
        feedback_getter=lambda msg: msg.throttle_percentage,
        command_setter=lambda c, v: setattr(c, "throttle", v),
    )
    if not result["pass"]:
        path = recorder.save_and_plot("Step Response", "Throttle %", "throttle_resp")
        return fail(result["details"], plot=path)
    path = recorder.save_and_plot("Step Response", "Throttle %", "throttle_resp")
    return success(result["details"], plot=path)
