# util.py
from modules.common_msgs.control_msgs import control_cmd_pb2
from modules.common_msgs.chassis_msgs import chassis_pb2
from typing import Dict, Any


def fail(details: str, **kwargs) -> Dict[str, Any]:
    return {"pass": False, "details": details, **kwargs}


def success(details: str, **kwargs) -> Dict[str, Any]:
    return {"pass": True, "details": details, **kwargs}


def create_base_command() -> control_cmd_pb2.ControlCommand:
    """Creates a base command with safe defaults (AUTO mode, Neutral, Brake)."""
    cmd = control_cmd_pb2.ControlCommand()
    cmd.pad_msg.driving_mode = chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE
    cmd.pad_msg.action = 2  # START
    cmd.throttle = 0.0
    cmd.brake = 0.0
    cmd.steering_target = 0.0
    cmd.speed = 0.0
    cmd.acceleration = 0.0
    cmd.parking_brake = True
    cmd.gear_location = chassis_pb2.Chassis.GEAR_NEUTRAL
    return cmd


def calculate_response_metrics(start_time, data_stream, target):
    """Simple metric calc. data_stream: [(t, val), ...]"""
    if not data_stream:
        return {
            "response_time_ms": 0,
            "overshoot_percent": 0,
            "steady_state_error_percent": 0,
        }

    # Filter after start
    valid_data = [d for d in data_stream if d[0] >= start_time]
    if not valid_data:
        return {
            "response_time_ms": 0,
            "overshoot_percent": 0,
            "steady_state_error_percent": 0,
        }

    # Steady state (last 0.5s)
    end_time = valid_data[-1][0]
    steady_vals = [v for t, v in valid_data if t > end_time - 0.5]
    avg = sum(steady_vals) / len(steady_vals) if steady_vals else 0

    err_pct = abs(avg - target)

    # Response time (simplistic t90)
    t90 = start_time
    for t, v in valid_data:
        if abs(v) >= abs(target * 0.9):
            t90 = t
            break

    return {
        "response_time_ms": int((t90 - start_time) * 1000),
        "overshoot_percent": 0.0,  # Simplification
        "steady_state_error_percent": round(err_pct, 2),
    }
