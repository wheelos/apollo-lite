#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Optional
from enum import Enum

# Attempt to import the cyber_record library, provide installation instructions on failure.
try:
    from cyber_record.record import Record
except ImportError:
    print("Error: Core dependency 'cyber_record' not found.")
    print("Please install it using: 'pip install cyber-record-parser'")
    exit(1)


# =============================================================================
# MODULE 1: Data Extraction
# =============================================================================


class SortMode(Enum):
    """Enumeration for file sorting methods."""

    NAME = 1


def get_sorted_records(
    input_path: str, sort_mode: SortMode = SortMode.NAME
) -> List[str]:
    """Gets and sorts all record files from the input path."""
    if os.path.isfile(input_path) and "record" in os.path.basename(input_path):
        return [input_path]

    if not os.path.isdir(input_path):
        print(f"Error: Input path '{input_path}' is not a valid file or directory.")
        return []

    record_files = [
        os.path.join(input_path, f)
        for f in os.listdir(input_path)
        if "record" in f and os.path.isfile(os.path.join(input_path, f))
    ]

    if sort_mode == SortMode.NAME:
        record_files.sort()

    return record_files


def extract_trajectory_from_records(
    record_files: List[str], topic: str
) -> Optional[np.ndarray]:
    """Extracts trajectory information from record files into a NumPy array."""
    trajectory_points = []
    print(f"Parsing topic '{topic}' from {len(record_files)} record file(s)...")

    for record_file in record_files:
        try:
            record = Record(record_file)
            for msg_topic, message, _ in record.read_messages_fallback(topics=[topic]):
                trajectory_points.append(
                    [
                        message.pose.position.x,
                        message.pose.position.y,
                        message.pose.heading,
                    ]
                )
        except Exception as e:
            print(f"Warning: An error occurred while reading '{record_file}': {e}")
            continue

    if not trajectory_points:
        return None

    print(f"Successfully extracted {len(trajectory_points)} trajectory points.")
    return np.array(trajectory_points)


# =============================================================================
# MODULE 2: Core Analysis & Computation
# =============================================================================


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    """Normalize an angle or array of angles to the interval [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def moving_average(data: np.ndarray, window_size: int) -> np.ndarray:
    """Computes the moving average of a 1D array."""
    if window_size <= 1:
        return data
    return np.convolve(data, np.ones(window_size), "same") / window_size


def analyze_trajectory(
    trajectory: np.ndarray, stabilization_frames: int, is_reverse: bool
):
    """
    Performs a comprehensive error analysis for both forward and reverse movements.
    """
    if trajectory is None or len(trajectory) < stabilization_frames + 2:
        print(
            f"Error: Not enough trajectory points for analysis (requires at least {stabilization_frames + 2})."
        )
        return

    # 【NEW】Set a multiplier based on the driving direction.
    direction_multiplier = -1 if is_reverse else 1
    mode_string = "Reverse" if is_reverse else "Forward"
    print(f"Analysis Mode: {mode_string}")

    # Stabilize the starting pose by averaging the initial frames.
    if stabilization_frames > 1:
        print(
            f"Stabilizing starting pose using the first {stabilization_frames} frames..."
        )
        stable_segment = trajectory[:stabilization_frames]
        x_start = np.mean(stable_segment[:, 0])
        y_start = np.mean(stable_segment[:, 1])
        avg_sin_h = np.mean(np.sin(stable_segment[:, 2]))
        avg_cos_h = np.mean(np.cos(stable_segment[:, 2]))
        heading_start = np.arctan2(avg_sin_h, avg_cos_h)
        start_pose = np.array([x_start, y_start, heading_start])
        analysis_trajectory = trajectory[stabilization_frames - 1 :]
    else:
        start_pose = trajectory[0]
        x_start, y_start, heading_start = start_pose
        analysis_trajectory = trajectory

    actual_x, actual_y, actual_h = (
        analysis_trajectory[:, 0].copy(),
        analysis_trajectory[:, 1].copy(),
        analysis_trajectory[:, 2].copy(),
    )
    actual_x[0], actual_y[0], actual_h[0] = x_start, y_start, heading_start

    distances = np.sqrt(np.diff(actual_x) ** 2 + np.diff(actual_y) ** 2)
    s = np.insert(np.cumsum(distances), 0, 0)

    # 【MODIFIED】Generate the ideal path using the direction multiplier.
    ideal_x = x_start + direction_multiplier * s * np.cos(heading_start)
    ideal_y = y_start + direction_multiplier * s * np.sin(heading_start)

    # --- Error Calculations ---
    # 【MODIFIED】Standardized Lateral Error: Positive is always to the left of the ideal path.
    lat_err = (actual_y - y_start) * np.cos(heading_start) - (
        actual_x - x_start
    ) * np.sin(heading_start)
    pos_err = np.sqrt((actual_x - ideal_x) ** 2 + (actual_y - ideal_y) ** 2)
    head_err = wrap_angle(actual_h - heading_start)

    delta_s = np.diff(s, prepend=s[0])
    delta_h = wrap_angle(np.diff(actual_h, prepend=actual_h[0]))

    # 【MODIFIED】Curvature sign now reflects the path's geometric turn direction.
    raw_curvature = direction_multiplier * np.divide(
        delta_h, delta_s, out=np.zeros_like(delta_h), where=delta_s > 1e-6
    )
    smoothing_window = max(3, int(len(raw_curvature) * 0.05))
    smoothed_curvature = moving_average(raw_curvature, window_size=smoothing_window)

    # --- Statistical Summary ---
    print("\n--- Comprehensive Trajectory Analysis Report ---")
    print(f"Mode: {mode_string}")
    print(f"Total Trajectory Length: {s[-1]:.2f} m")
    # 【MODIFIED】Corrected interpretation of lateral error.
    print(
        f"Final Lateral Error: {lat_err[-1]:.4f} m ({'Left' if lat_err[-1] > 0 else 'Right'} of ideal path)"
    )
    print(f"Max Lateral Error (absolute): {np.max(np.abs(lat_err)):.4f} m")
    print(f"Final Heading Error: {np.rad2deg(head_err[-1]):.4f} degrees")
    print(
        f"Max Heading Error (absolute): {np.rad2deg(np.max(np.abs(head_err))):.4f} degrees"
    )
    print(
        f"Mean Path Curvature (absolute): {np.mean(np.abs(smoothed_curvature)):.6f} rad/m"
    )
    print("----------------------------------------------\n")

    print("Generating analysis dashboard...")
    plot_analysis_dashboard(
        s,
        ideal_x,
        ideal_y,
        actual_x,
        actual_y,
        start_pose,
        lat_err,
        pos_err,
        head_err,
        smoothed_curvature,
    )


# =============================================================================
# MODULE 3: Results Visualization
# =============================================================================


def plot_analysis_dashboard(
    s,
    ideal_x,
    ideal_y,
    actual_x,
    actual_y,
    start_pose,
    lat_err,
    pos_err,
    head_err,
    curvature,
):
    """Generates a single dashboard figure with four subplots for all results."""
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axs = plt.subplots(2, 2, figsize=(18, 14))
    fig.suptitle("Comprehensive Trajectory Analysis Dashboard", fontsize=20)

    # --- Plot 1: Trajectory Comparison (Top-Left) ---
    ax = axs[0, 0]
    ax.plot(ideal_x, ideal_y, "r--", label="Ideal Straight Path", linewidth=2)
    ax.plot(
        actual_x,
        actual_y,
        "b-",
        label="Actual Vehicle Path",
        marker=".",
        markersize=3,
        alpha=0.8,
    )
    ax.scatter(
        start_pose[0],
        start_pose[1],
        c="g",
        s=120,
        zorder=5,
        label=f"Stabilized Start\n(Heading: {np.rad2deg(start_pose[2]):.2f}°)",
    )
    ax.set_title("Trajectory Comparison", fontsize=14)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.legend()
    ax.axis("equal")
    ax.grid(True)

    # --- Plot 2: Lateral & Positional Error (Top-Right) ---
    ax = axs[0, 1]
    ax.plot(s, lat_err, "g-", label="Lateral Error")
    ax.plot(s, pos_err, color="m", label="Positional Error", linestyle="--")
    ax.axhline(0, color="black", linewidth=0.7, linestyle="--")
    ax.set_title("Lateral & Positional Error", fontsize=14)
    ax.set_xlabel("Distance Traveled (m)")
    ax.set_ylabel("Error (m)")
    ax.legend()
    ax.grid(True)

    # --- Plot 3: Heading Error (Bottom-Left) ---
    ax = axs[1, 0]
    ax.plot(s, np.rad2deg(head_err), color="c", label="Heading Error")
    ax.axhline(0, color="black", linewidth=0.7, linestyle="--")
    ax.set_title("Heading Error", fontsize=14)
    ax.set_xlabel("Distance Traveled (m)")
    ax.set_ylabel("Error (degrees)")
    ax.legend()
    ax.grid(True)

    # --- Plot 4: Smoothed Curvature (Bottom-Right) ---
    ax = axs[1, 1]
    ax.plot(
        s, curvature, color="orange", label="Smoothed Path Curvature"
    )  # Label updated for clarity
    ax.axhline(0, color="black", linewidth=0.7, linestyle="--")
    ax.set_title("Smoothed Path Curvature", fontsize=14)
    ax.set_xlabel("Distance Traveled (m)")
    ax.set_ylabel("Curvature (rad/m)")
    ax.legend()
    ax.grid(True)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


# =============================================================================
# Main Execution Block
# =============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A comprehensive tool to analyze straight-line driving performance from Apollo record files.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "-i",
        "--input",
        required=True,
        help="Path to a record file or a directory containing record files.",
    )
    parser.add_argument(
        "-c",
        "--channel",
        default="/apollo/localization/pose",
        help="The topic name for localization data (default: /apollo/localization/pose).",
    )
    parser.add_argument(
        "-s",
        "--stabilize",
        type=int,
        default=10,
        help="Number of initial frames to average for a stable starting pose (default: 10). Set to 1 to disable.",
    )
    # 【NEW】Argument to specify reverse mode.
    parser.add_argument(
        "-r",
        "--reverse",
        action="store_true",
        help="Enable reverse mode for trajectory analysis.",
    )
    args = parser.parse_args()

    record_files = get_sorted_records(args.input)
    if not record_files:
        print(f"No record files found in the specified path: '{args.input}'")
        exit(1)

    full_trajectory = extract_trajectory_from_records(record_files, args.channel)

    if full_trajectory is not None:
        analyze_trajectory(full_trajectory, args.stabilize, args.reverse)
    else:
        print(
            f"Error: No messages found on topic '{args.channel}' in the provided files."
        )
        exit(1)
