import os
import csv
import time
import threading
import matplotlib.pyplot as plt
from datetime import datetime
from typing import List, Tuple


class DataRecorder:
    """
    Responsible for high-frequency data collection, CSV storage, and plotting.
    Implements decoupling of data and logic.
    """

    def __init__(self, test_name: str, output_dir: str = "test_results"):
        self.test_name = test_name
        self.output_dir = os.path.join(
            output_dir, datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # Data buffer: (timestamp, cmd_val, feedback_val, speed_mps)
        self.data: List[Tuple[float, float, float, float]] = []
        self.is_recording = False
        self._lock = threading.Lock()
        self.start_time = 0.0

    def start(self):
        with self._lock:
            self.data = []
            self.start_time = time.time()
            self.is_recording = True

    def record(self, cmd_val: float, feedback_val: float, speed_mps: float = 0.0):
        """Call this method in the test loop to record data points."""
        if not self.is_recording:
            return
        with self._lock:
            t = time.time() - self.start_time
            self.data.append((t, cmd_val, feedback_val, speed_mps))

    def stop(self):
        with self._lock:
            self.is_recording = False

    def save_and_plot(self, title: str, ylabel: str, save_name: str):
        """Save the CSV file and generate a PNG image."""
        if not self.data:
            return

        # 1. Save CSV
        csv_path = os.path.join(self.output_dir, f"{save_name}.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time(s)", "Command", "Feedback", "Speed(m/s)"])
            writer.writerows(self.data)

        # 2. Plot (Backend Agg for headless environments)
        plt.switch_backend("Agg")
        times = [d[0] for d in self.data]
        cmds = [d[1] for d in self.data]
        fdbks = [d[2] for d in self.data]

        plt.figure(figsize=(10, 6))
        plt.plot(times, cmds, "r--", label="Command", linewidth=1.5)
        plt.plot(times, fdbks, "b-", label="Feedback", linewidth=2.0)

        plt.title(f"{self.test_name}: {title}")
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True)

        png_path = os.path.join(self.output_dir, f"{save_name}.png")
        plt.savefig(png_path)
        plt.close()

        return png_path
