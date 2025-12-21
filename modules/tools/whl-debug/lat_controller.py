import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import matplotlib.font_manager as fm
from datetime import datetime, timedelta

# --- Data Loading ---
def load_steer_data_from_csv(file_path):
    """
    Load and preprocess steering control data from a CSV file.
    """
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()
    # Timestamp processing: if not present, generate simulated timestamps
    if 'timestamp' not in df.columns:
        base_time = datetime(2025, 1, 1, 0, 0, 0)
        df['timestamp'] = [base_time + timedelta(seconds=i * 0.01) for i in range(len(df))]
    else:
        df['timestamp'] = pd.to_datetime(df['timestamp'])
    df = df.sort_values(by="timestamp").reset_index(drop=True)
    return df

# --- Font and Global Configuration ---
# Note: Make sure this font path exists on your system, or replace it with a valid Chinese font path.
# Windows example: 'C:/Windows/Fonts/msyh.ttc' (Microsoft YaHei)
# Linux example: '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc' (WenQuanYi Zen Hei)
font_path = '/System/Library/Fonts/PingFang.ttc'
font_prop = fm.FontProperties(fname=font_path)
plt.rcParams['axes.unicode_minus'] = False

# --- Optimized 4x1 Professional Visualization ---
def plot_lqr_performance_4x1(df, title="LQR Lateral Control System Performance Analysis"):
    """
    Use a 4x1 vertical layout for in-depth analysis of LQR controller performance.
    Layout logic: Scenario -> Error -> Control -> Attribution
    """
    fig, axs = plt.subplots(4, 1, figsize=(18, 24), sharex=True)
    fig.suptitle(title, fontproperties=font_prop, fontsize=20, y=0.96)

    # 1. Scenario background: speed and desired curvature
    ax = axs[0]
    line1 = ax.plot(df['timestamp'], df['linear_velocity'], label='Speed (m/s)', color='g', linestyle="--")
    ax.set_ylabel("Speed (m/s)", fontproperties=font_prop)
    ax2 = ax.twinx()
    line2 = ax2.plot(df['timestamp'], df['curvature'], label="Desired Curvature (1/m)", color='b', alpha=0.7)
    ax2.set_ylabel("Desired Curvature (1/m)", fontproperties=font_prop)
    # Merge legends
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='upper right', prop=font_prop)

    # 2. Core errors: lateral error and heading error
    ax = axs[1]
    ax.plot(df["timestamp"], df["lateral_error"], label="Lateral Error (m)", color="r")
    # Note: 'heading_error' is a key input for the LQR model, assumed to be present in the data.
    # If not, you need to compute or record it from the raw data.
    if 'heading_error' in df.columns:
        ax.plot(df["timestamp"], df["heading_error"], label="Heading Error (rad)", color="purple", linestyle='-.')
    ax.set_ylabel("Error Value", fontproperties=font_prop)
    ax.axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.5) # Add zero reference line
    ax.legend(prop=font_prop, loc='upper right')

    # 3. Control output: LQR components and actual execution value
    ax = axs[2]
    ax.plot(df['timestamp'], df['steer_angle'], label='Total LQR Output', color='k', linewidth=2.5)
    ax.plot(df['timestamp'], df['steer_angle_feedforward'], label='Feedforward Component', color='c', linestyle='--')
    ax.plot(df['timestamp'], df['steer_angle_feedback'], label='Feedback Component (LQR)', color='m', linestyle='--')
    ax.plot(df['timestamp'], df['steer_angle_feedback_augment'], label='Augmented Component (Lead-Lag)', color='orange', linestyle=':')
    ax.plot(df['timestamp'], df['steering_percentage'], label='Actual Chassis Feedback (%)', color='gray', linestyle='--', alpha=0.7)
    ax.set_ylabel("Steering Angle/Control (%)", fontproperties=font_prop)
    ax.legend(prop=font_prop, loc='upper right')

    # 4. Feedback attribution: internal contribution analysis of LQR feedback terms
    ax = axs[3]
    ax.plot(df['timestamp'], df['steer_angle_lateral_contribution'], label='Lateral Error Contribution')
    ax.plot(df['timestamp'], df['steer_angle_lateral_rate_contribution'], label='Lateral Error Rate Contribution')
    ax.plot(df['timestamp'], df['steer_angle_heading_contribution'], label='Heading Error Contribution')
    ax.plot(df['timestamp'], df['steer_angle_heading_rate_contribution'], label='Heading Error Rate Contribution')
    ax.set_ylabel("State Contribution Value (%)", fontproperties=font_prop)
    ax.axhline(0, color='k', linestyle='--', linewidth=0.8, alpha=0.5) # Add zero reference line
    ax.legend(prop=font_prop, loc='upper right')


    # --- Unified Formatting ---
    # With sharex=True, only need to format the bottom x-axis
    axs[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    plt.xlabel("Time (H:M:S)", fontproperties=font_prop)

    for ax in axs:
        ax.grid(True, linestyle="--", alpha=0.5)
        for label in (ax.get_xticklabels() + ax.get_yticklabels()):
            label.set_fontproperties(font_prop)

    fig.autofmt_xdate(bottom=0.2)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


# --- Main Process ---
# Replace with your data file path
file_path = 'steer_log_simple_optimal_2025-08-19_140000.csv'
df = load_steer_data_from_csv(file_path)

# Call the optimized plotting function
plot_lqr_performance_4x1(df)
