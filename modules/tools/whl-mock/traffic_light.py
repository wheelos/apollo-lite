#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty
import argparse
import logging
import math
import atexit
from pathlib import Path
from typing import List, Optional, Set
from dataclasses import dataclass

# Attempt to import Apollo Cyber RT modules
try:
    from cyber.python.cyber_py3 import cyber
    from cyber.python.cyber_py3 import cyber_timer
    from cyber.python.cyber_py3 import cyber_time

    # Try importing Protobuf messages (compatible with Apollo 8.0+ and older versions)
    try:
        from modules.common_msgs.localization_msgs import localization_pb2
        from modules.common_msgs.perception_msgs import traffic_light_detection_pb2
        from modules.common_msgs.map_msgs.map_pb2 import Map
    except ImportError:
        from modules.localization.proto import localization_pb2
        from modules.perception.proto import traffic_light_detection_pb2
        from modules.map.proto.map_pb2 import Map
except ImportError:
    print(
        "Error: Apollo Cyber python modules not found. Ensure you are in the Apollo Docker environment."
    )
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class UIControl:
    """Constants and methods for terminal UI control using ANSI escape codes."""

    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    CYAN = "\033[96m"
    RESET = "\033[0m"
    BOLD = "\033[1m"

    # Clear from cursor to end of line
    CLEAR_LINE = "\033[K"
    # Cursor visibility controls
    HIDE_CURSOR = "\033[?25l"
    SHOW_CURSOR = "\033[?25h"

    @staticmethod
    def init_terminal():
        """Hide cursor to prevent flickering."""
        sys.stdout.write(UIControl.HIDE_CURSOR)
        sys.stdout.flush()

    @staticmethod
    def restore_terminal():
        """Restore cursor visibility on exit."""
        sys.stdout.write(UIControl.SHOW_CURSOR)
        sys.stdout.flush()


@dataclass
class SignalInfo:
    """Data structure to hold signal information."""

    id: str


class TerminalInputHandler:
    """
    Context manager to handle non-blocking keyboard input.
    Sets the terminal to non-canonical mode.
    """

    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    @staticmethod
    def get_key_non_blocking() -> Optional[str]:
        """Checks for a keypress without blocking execution."""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None


class MapProvider:
    """Handles loading HDMap and querying signals."""

    def __init__(self, map_file: str):
        self.hdmap = Map()
        p = Path(map_file)

        if not p.exists():
            logger.warning(f"Map file not found: {map_file}. Running in non-map mode.")
            return

        logger.info(f"Loading map from: {map_file}")
        try:
            # Must read as binary ('rb') for Protobuf parsing
            with open(p, "rb") as f:
                self.hdmap.ParseFromString(f.read())
            logger.info(f"Map loaded. Total signals: {len(self.hdmap.signal)}")
        except Exception as e:
            logger.error(f"Failed to parse map file: {e}")

    def get_signals(
        self, position, distance: float, all_lights: bool
    ) -> List[SignalInfo]:
        """Returns a list of signals based on position and distance."""
        signals = []

        # Safety check if map is not loaded or empty
        if not self.hdmap or not self.hdmap.signal:
            return signals

        # If position is not yet available or 'all_lights' is forced
        if all_lights or position is None:
            for signal in self.hdmap.signal:
                if signal.id and signal.id.id:
                    signals.append(SignalInfo(signal.id.id))
            return signals

        # Filter by Euclidean distance
        for signal in self.hdmap.signal:
            if not signal.boundary or not signal.boundary.point:
                continue

            # Use the first point of the signal boundary for distance check
            sig_pos = signal.boundary.point[0]
            dx = sig_pos.x - position.x
            dy = sig_pos.y - position.y

            if math.hypot(dx, dy) <= distance:
                signals.append(SignalInfo(signal.id.id))

        return signals


class ManualTrafficLightComponent:
    def __init__(self, node, args, keyboard_handler: TerminalInputHandler):
        self.node = node
        self.args = args
        self.kb_handler = keyboard_handler
        self.map_provider = MapProvider(self.args.map_file)

        # Component State
        self.is_green: bool = False
        self.updated: bool = True
        self.has_localization: bool = False
        self.current_pose = None
        self.prev_signal_ids: Set[str] = set()
        self.msg_seq = 0

        # Cyber Writers and Readers
        self.writer = self.node.create_writer(
            self.args.topic_detection, traffic_light_detection_pb2.TrafficLightDetection
        )
        self.reader = self.node.create_reader(
            self.args.topic_localization,
            localization_pb2.LocalizationEstimate,
            self.on_localization,
        )

        # Initialize Timer (10Hz)
        # Period: 100ms, Callback: self.run_once, Oneshot: 0 (False)
        self.timer = cyber_timer.Timer(100, self.run_once, 0)

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def on_localization(self, msg):
        self.current_pose = msg.pose.position
        self.has_localization = True

    def process_input(self):
        key = self.kb_handler.get_key_non_blocking()
        if key == "c":
            self.is_green = not self.is_green
            self.updated = True

    def run_once(self):
        """Main loop executed by the timer."""
        # 1. Get signals from map
        signals = self.map_provider.get_signals(
            self.current_pose, self.args.distance, self.args.all_lights
        )
        signal_ids = sorted([s.id for s in signals])

        # 2. Check if signal list changed
        current_ids_set = set(signal_ids)
        if current_ids_set != self.prev_signal_ids:
            self.prev_signal_ids = current_ids_set
            self.updated = True

        # 3. Handle keyboard input
        self.process_input()

        # 4. Refresh UI
        # Force refresh every 10 frames to fix potential console artifacts
        self.msg_seq += 1
        if self.updated or self.msg_seq % 10 == 0:
            self._print_status(signal_ids)
            self.updated = False

        # 5. Publish detection message
        self._publish_message(signal_ids)

    def _print_status(self, signal_ids: List[str]):
        """Renders the single-line UI."""

        # Determine status icon and text color
        if self.is_green:
            status_icon = f"{UIControl.GREEN}●{UIControl.RESET}"
            # :^7 centers the text within 7 spaces for stability
            status_text = f"{UIControl.GREEN}{'GREEN':^7}{UIControl.RESET}"
        else:
            status_icon = f"{UIControl.RED}●{UIControl.RESET}"
            status_text = f"{UIControl.RED}{'RED':^7}{UIControl.RESET}"

        # Format Signal IDs
        count = len(signal_ids)
        if count == 0:
            ids_str = f"{UIControl.YELLOW}No Signals{UIControl.RESET}"
        else:
            # Truncate list if too long to keep UI clean
            display_ids = signal_ids[:4]
            ids_str = " ".join(display_ids)
            if count > 4:
                ids_str += f" ...(+{count-4})"

        # Construct output string
        # \r: Return to start of line
        # CLEAR_LINE: Wipe remaining characters at the end
        output = (
            f"\r[{status_icon} {status_text}] "
            f"Found: {count:<3} | "
            f"IDs: {ids_str} "
            f"{UIControl.CLEAR_LINE}"
        )

        sys.stdout.write(output)
        sys.stdout.flush()

    def _publish_message(self, signal_ids: List[str]):
        detection = traffic_light_detection_pb2.TrafficLightDetection()
        detection.header.timestamp_sec = cyber_time.Time.now().to_sec()
        detection.header.module_name = "manual_traffic_light_py"
        detection.header.sequence_num = self.msg_seq

        ts_color = (
            traffic_light_detection_pb2.TrafficLight.GREEN
            if self.is_green
            else traffic_light_detection_pb2.TrafficLight.RED
        )

        for sig_id in signal_ids:
            light = detection.traffic_light.add()
            light.id = sig_id
            light.color = ts_color
            light.confidence = 1.0
            light.tracking_time = 1.0

        self.writer.write(detection)


def parse_args():
    parser = argparse.ArgumentParser(
        description="""
    Apollo Manual Traffic Light Mock

    Example usage:
        python3 modules/tools/whl-mock/traffic_light.py --all_lights --map_file=modules/map/data/demo/base_map.bin
    """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--all_lights", action="store_true", help="Set all lights on the map"
    )
    parser.add_argument(
        "--distance", type=float, default=100.0, help="Detection distance in meters"
    )
    parser.add_argument(
        "--map_file",
        type=str,
        default="/apollo/modules/map/data/borregas_ave/base_map.bin",
        help="Path to the HDMap binary file",
    )
    parser.add_argument(
        "--topic_localization", type=str, default="/apollo/localization/pose"
    )
    parser.add_argument(
        "--topic_detection", type=str, default="/apollo/perception/traffic_light"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cyber.init()

    # Ensure cursor is restored even if the script crashes or exits
    atexit.register(UIControl.restore_terminal)

    node_name = "manual_traffic_light_py"
    node = cyber.Node(node_name)

    # Print Static Header
    print("\n" + "=" * 60)
    print(f"{UIControl.BOLD}Apollo Traffic Light Mocker{UIControl.RESET}")
    print(f"Map: {args.map_file}")
    print(
        f"Control: Press {UIControl.BOLD}'c'{UIControl.RESET} to toggle light. Ctrl+C to exit."
    )
    print("=" * 60 + "\n")

    # Initialize Terminal (Hide Cursor)
    UIControl.init_terminal()

    with TerminalInputHandler() as kb_handler:
        component = ManualTrafficLightComponent(node, args, kb_handler)
        component.start()

        # node.spin() blocks the main thread until shutdown is triggered
        node.spin()

        component.stop()

    cyber.shutdown()


if __name__ == "__main__":
    main()
