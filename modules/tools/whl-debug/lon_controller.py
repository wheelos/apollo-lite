import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ============ Configuration ============

# Path to your CSV file (replace with your filename)
csv_path = "speed_log__2025-08-27_094217.csv"

# Sampling period in seconds (modify if your log's sample rate is not 0.1s)
DT = 0.1

# Position loop gain (speed compensation = KP_STATION * station_error)
KP_STATION = 0.2

# Significant throttle/brake thresholds (for conflict detection and highlighting)
THR_ON = 0.05
BRK_ON = 0.05

# ============ Column Names (must match the logger's output) ============

COLS = [
    "station_reference",
    "station_error",
    "station_error_limited",
    "preview_station_error",
    "speed_reference",
    "speed_error",
    "speed_error_limited",
    "preview_speed_reference",
    "preview_speed_error",
    "preview_acceleration_reference",
    "acceleration_cmd_closeloop",
    "acceleration_cmd",
    "acceleration_lookup",
    "acceleration_lookup_limit",
    "speed_lookup",
    "calibration_value",
    "throttle_cmd",
    "brake_cmd",
    "is_full_stop",
]

# ============ Data Loading & Cleaning ============


def load_log(path: str, dt: float) -> pd.DataFrame:
    """Loads and cleans the control log from a CSV file."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"CSV not found: {p.resolve()}")

    # Try reading with a header; if key columns are missing, retry without one.
    try:
        df_try = pd.read_csv(p, header=0, on_bad_lines="skip")
        if set(COLS).issubset(df_try.columns):
            df = df_try.copy()
        else:
            raise ValueError("Header not matching")
    except (ValueError, pd.errors.ParserError):
        df = pd.read_csv(p, header=None, on_bad_lines="skip")
        # Keep only the expected number of columns to prevent trailing comma issues
        df = df.iloc[:, : len(COLS)]
        df.columns = COLS

    # Convert all columns to numeric, coercing errors to NaN
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")

    # Drop rows with missing values
    df.dropna(inplace=True)
    df.reset_index(drop=True, inplace=True)

    # Create time axis and derived quantities
    df["time"] = np.arange(len(df)) * dt
    df["speed_offset"] = KP_STATION * df["station_error"]
    df["dynamic_target_speed"] = df["speed_reference"] + df["speed_offset"]

    return df


# ============ Metrics Computation ============


def compute_metrics(df: pd.DataFrame, dt: float) -> dict:
    """Computes a dictionary of key performance indicators (KPIs)."""
    met = {}

    # Speed Error Metrics
    if "speed_error" in df:
        se = df["speed_error"].values
        met["speed_err_rms"] = float(np.sqrt(np.mean(se**2)))
        met["speed_err_max_abs"] = float(np.max(np.abs(se)))
        # Steady-state bias: mean of the last 20% of the data
        tail = se[int(0.8 * len(se)) :]
        met["speed_err_steady_bias"] = float(np.mean(tail)) if len(tail) > 0 else np.nan

    # Speed Tracking Overshoot (relative to target)
    if "speed_lookup" in df and "speed_reference" in df:
        ref = np.maximum(df["speed_reference"].values, 1e-3)  # Avoid division by zero
        overshoot = (df["speed_lookup"].values - df["speed_reference"].values) / ref
        met["speed_overshoot_max_pct"] = float(np.max(overshoot) * 100.0)

    # Acceleration Lag (command vs. actual)
    if "acceleration_cmd" in df and "acceleration_lookup" in df:
        cmd = df["acceleration_cmd"].values - np.mean(df["acceleration_cmd"].values)
        act = df["acceleration_lookup"].values - np.mean(
            df["acceleration_lookup"].values
        )
        # Only compute if signals have variance
        if np.std(cmd) > 1e-9 and np.std(act) > 1e-9:
            corr = np.correlate(cmd, act, mode="full")
            lags = np.arange(-len(cmd) + 1, len(cmd))
            best_lag = lags[np.argmax(corr)]
            met["accel_best_lag_samples"] = int(best_lag)
            met["accel_best_lag_sec"] = float(best_lag * dt)
        else:
            met["accel_best_lag_samples"] = 0
            met["accel_best_lag_sec"] = 0.0

    # Acceleration Saturation Ratio
    if "acceleration_lookup_limit" in df and "acceleration_cmd" in df:
        lim = np.abs(df["acceleration_lookup_limit"].values)
        cmd_abs = np.abs(df["acceleration_cmd"].values)
        saturated = (lim > 1e-6) & (cmd_abs >= 0.98 * lim)
        met["accel_saturation_ratio_pct"] = float(100.0 * np.mean(saturated))

    # Throttle/Brake Conflict Ratio
    if "throttle_cmd" in df and "brake_cmd" in df:
        thr = df["throttle_cmd"].values
        brk = df["brake_cmd"].values
        conflict = (thr > THR_ON) & (brk > BRK_ON)
        met["throttle_brake_conflict_pct"] = float(100.0 * np.mean(conflict))

    # Station Error Metrics
    if "station_error" in df:
        st = df["station_error"].values
        met["station_err_rms"] = float(np.sqrt(np.mean(st**2)))
        met["station_err_max_abs"] = float(np.max(np.abs(st)))

    return met


# ============ Visualization (4x1 Subplots) ============


def plot_dashboard_4x1(df: pd.DataFrame, save_path: str = "longitudinal_dashboard.png"):
    """Generates and saves a 4x1 dashboard plot of the control performance."""
    fig, axs = plt.subplots(4, 1, figsize=(16, 22), sharex=True)
    t = df["time"].values

    # --- Plot 1: Speed Tracking ---
    ax = axs[0]
    ax.plot(t, df["speed_reference"], "g--", label="Speed Reference")
    ax.plot(t, df["preview_speed_reference"], "c:", label="Preview Speed Reference")
    ax.plot(
        t, df["dynamic_target_speed"], "r-", label="Dynamic Target Speed", linewidth=2
    )
    ax.plot(t, df["speed_lookup"], "b-", label="Actual Speed (lookup)", alpha=0.9)
    ax.set_title("Plot 1: Speed Tracking (Reference/Target vs. Actual)")
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, linestyle="--", linewidth=0.5)
    ax.legend(loc="best")

    # --- Plot 2: Acceleration Breakdown ---
    ax = axs[1]
    ax.plot(
        t, df["acceleration_cmd"], "r-", label="Final Acceleration Cmd", linewidth=2
    )
    ax.plot(
        t, df["preview_acceleration_reference"], "g--", label="Preview Accel Reference"
    )
    ax.plot(
        t,
        df["acceleration_cmd_closeloop"],
        "b-",
        label="Closed-Loop Accel Cmd",
        alpha=0.9,
    )
    ax.plot(t, df["acceleration_lookup"], "k:", label="Actual Accel (lookup)")
    ax.plot(
        t,
        df["acceleration_lookup_limit"],
        color="k",
        linestyle="--",
        alpha=0.6,
        label="Accel Lookup Limit",
    )
    ax.axhline(0, color="gray", linestyle=":", linewidth=1)
    ax.set_title("Plot 2: Acceleration Command Breakdown & Response")
    ax.set_ylabel("Acceleration (m/s^2)")
    ax.grid(True, linestyle="--", linewidth=0.5)
    ax.legend(loc="best")

    # --- Plot 3: Error Signals ---
    ax = axs[2]
    ax.plot(t, df["station_error"], color="#FF7F0E", label="Station Error")
    ax.plot(
        t,
        df["station_error_limited"],
        color="#FF7F0E",
        linestyle="--",
        label="Station Error (Limited)",
    )
    ax.plot(
        t,
        df["preview_station_error"],
        color="#FFBB78",
        linestyle=":",
        label="Preview Station Error",
    )
    ax.plot(t, df["speed_error"], color="#9467BD", label="Speed Error")
    ax.plot(
        t,
        df["speed_error_limited"],
        color="#9467BD",
        linestyle="--",
        label="Speed Error (Limited)",
    )
    ax.axhline(0, color="gray", linestyle=":", linewidth=1)
    ax.set_title("Plot 3: Core Error Signals (Station / Speed)")
    ax.set_ylabel("Error (m or m/s)")
    ax.grid(True, linestyle="--", linewidth=0.5)
    ax.legend(loc="best")

    # --- Plot 4: Actuators & State (Dual-Axis) ---
    ax1 = axs[3]
    ax2 = ax1.twinx()  # Create a second y-axis
    lines = []  # For combined legend

    (p1,) = ax1.plot(t, df["throttle_cmd"], "g-", label="Throttle Cmd", alpha=0.9)
    (p2,) = ax1.plot(t, df["brake_cmd"], "r-", label="Brake Cmd", alpha=0.9)
    lines.extend([p1, p2])

    (p3,) = ax2.step(
        t,
        df["is_full_stop"],
        where="post",
        color="k",
        linestyle="-.",
        label="Is Full Stop",
        alpha=0.7,
    )
    lines.append(p3)
    ax2.set_ylabel("Is Full Stop (boolean)")
    ax2.set_yticks([0, 1])

    ax1.set_title("Plot 4: Actuator Commands & Vehicle State")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Throttle / Brake (normalized)")
    ax1.grid(True, linestyle="--", linewidth=0.5)
    ax1.legend(handles=lines, loc="best")

    # Highlight sections where throttle and brake are applied simultaneously
    conflict = (df["throttle_cmd"] > THR_ON) & (df["brake_cmd"] > BRK_ON)
    for i in np.where(np.diff(conflict.astype(int)))[0]:
        if conflict[i]:  # Start of a conflict block
            start_time = t[i]
        else:  # End of a conflict block
            ax1.axvspan(start_time, t[i], color="red", alpha=0.08, zorder=0)

    fig.tight_layout(rect=[0, 0.02, 1, 0.98])
    fig.suptitle("Longitudinal Control Analysis Dashboard", y=0.995, fontsize=18)
    plt.savefig(save_path, dpi=150)
    plt.show()
    print(f"Saved figure to: {Path(save_path).resolve()}")


# ============ Main Execution ============

if __name__ == "__main__":
    try:
        df = load_log(csv_path, DT)
        metrics = compute_metrics(df, DT)

        # Print metrics
        print("\n===== Key Performance Indicators =====")
        for key, value in metrics.items():
            print(f"{key}: {value:.4f}")

        # Generate plot
        plot_dashboard_4x1(df)

    except FileNotFoundError as e:
        print(e)
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
