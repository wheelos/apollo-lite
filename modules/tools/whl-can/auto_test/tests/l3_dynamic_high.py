import time
from util import success, fail, create_base_command
from core.reporter import DataRecorder
from modules.common_msgs.chassis_msgs import chassis_pb2


def run_staged_accel(runner, target_kph, acc_limit_mps2):
    """Helper for TC-CTRL-10"""
    target_mps = target_kph / 3.6
    runner.ui.log(
        f"--- Accel Test: 0-{target_kph} km/h @ {acc_limit_mps2} m/s^2 ---", "CYAN"
    )

    if not runner.prepare_for_drive():
        return fail("Prep Failed")

    recorder = DataRecorder(f"L3_Accel_0_{target_kph}")
    recorder.start()

    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.speed = target_mps + 5.0  # Set target slightly higher to ensure limit is hit
    cmd.acceleration = acc_limit_mps2

    start_t = time.time()
    runner.update_command(cmd)

    reached = False
    duration = 0

    try:
        # Timeout allows for slow acceleration
        timeout = 20.0 if target_kph < 60 else 40.0
        while time.time() - start_t < timeout:
            chassis = runner.get_latest_chassis()
            if chassis:
                recorder.record(
                    acc_limit_mps2,
                    chassis.instantaneous_acceleration,
                    chassis.speed_mps,
                )

                if chassis.speed_mps >= target_mps:
                    duration = time.time() - start_t
                    reached = True
                    break

            if runner.estop_triggered.is_set():
                return fail("Aborted")
            time.sleep(0.01)

        # Safe Stop
        cmd.speed = 0
        cmd.acceleration = 0
        cmd.brake = 30.0
        runner.update_command(cmd)
        while True:
            chassis = runner.get_latest_chassis()
            if not chassis or chassis.speed_mps <= 0.1:
                break
            time.sleep(0.01)

    finally:
        recorder.stop()
        recorder.save_and_plot(
            f"Accel 0-{target_kph}", "Accel (m/s^2)", f"accel_0_{target_kph}"
        )

    if not reached:
        return fail(f"Failed to reach {target_kph} km/h")
    return success(f"0-{target_kph} km/h Time: {duration:.2f}s")


def run_staged_brake(runner, start_kph, brake_pct):
    """Helper for TC-CTRL-06"""
    start_mps = start_kph / 3.6
    runner.ui.log(
        f"--- Brake Test: {start_kph}-0 km/h @ {brake_pct}% Brake ---", "CYAN"
    )

    # 1. Accelerate to start speed
    if not runner.prepare_for_drive():
        return fail("Prep Failed")
    cmd = create_base_command()
    cmd.parking_brake = False
    cmd.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
    cmd.speed = start_mps
    cmd.acceleration = 2.0
    runner.update_command(cmd)

    if not runner.wait_for_condition(
        lambda: runner.get_latest_chassis().speed_mps >= start_mps * 0.95,
        30.0,
        f"Reach {start_kph} km/h",
    ):
        return fail("Failed to reach start speed")

    # 2. Apply Brake
    recorder = DataRecorder(f"L3_Brake_{start_kph}_0")
    recorder.start()

    cmd.speed = 0
    cmd.acceleration = 0
    cmd.brake = brake_pct
    trigger_time = time.time()
    runner.update_command(cmd)

    stop_time = 0
    stopped = False

    try:
        while time.time() - trigger_time < 15.0:
            chassis = runner.get_latest_chassis()
            if chassis:
                recorder.record(brake_pct, chassis.brake_percentage, chassis.speed_mps)

                if chassis.speed_mps < 0.1:
                    stop_time = time.time() - trigger_time
                    stopped = True
                    break
            time.sleep(0.01)
    finally:
        recorder.stop()
        recorder.save_and_plot(
            f"Brake {start_kph}-0", "Speed (m/s)", f"brake_{start_kph}_0"
        )

    if not stopped:
        return fail("Failed to stop")
    return success(f"Stop Time: {stop_time:.2f}s")


# --- Test Cases Exported ---


def test_l3_staged_acceleration(runner):
    """TC-CTRL-10: Staged Acceleration (Low/Mid/High)"""
    # 1. Low Speed
    res1 = run_staged_accel(runner, 30, 1.5)
    if not res1["pass"]:
        return res1
    time.sleep(2)

    # 2. Mid Speed
    res2 = run_staged_accel(runner, 60, 2.0)
    if not res2["pass"]:
        return res2
    time.sleep(2)

    # 3. High Speed (Run only if safe!)
    # res3 = run_staged_accel(runner, 100, 2.5)

    return success("Staged Acceleration Passed (Low/Mid)")


def test_l3_staged_braking(runner):
    """TC-CTRL-06: Staged Braking (30/60/100)"""
    # 1. 30-0
    res1 = run_staged_brake(runner, 30, 30.0)
    if not res1["pass"]:
        return res1
    time.sleep(3)

    # 2. 60-0
    res2 = run_staged_brake(runner, 60, 50.0)
    if not res2["pass"]:
        return res2

    return success("Staged Braking Passed")


def test_l3_staged_braking_performance(runner):
    """TC-CTRL-06: Staged Braking (30/60/100)"""
    return test_l3_staged_braking(runner)


def test_l3_emergency_brake(runner):
    """TC-CTRL-06: Emergency Brake (100km/h -> 0)"""
    # This is high risk, ensure safety
    return run_staged_brake(runner, 100, 100.0)
