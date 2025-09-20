#!/usr/bin/env python

# Copyright 2025 daohu527 <daohu527@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import curses
import datetime
import threading
import time
import logging

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.control_msgs import control_cmd_pb2

CONTROL_TOPIC = "/apollo/control"

SPEED_MIN, SPEED_MAX = -5.0, 5.0
STEERING_MIN, STEERING_MAX = -540.0, 540.0
BRAKE_MIN, BRAKE_MAX = 0.0, 100.0

SPEED_DELTA = 0.1
STEERING_DELTA = 1
BRAKE_DELTA = 1


class KeyboardController:
    """
    Curses-based keyboard controller, no root privileges required, suitable for
    Linux terminal environments.

    Listens for key presses in non-blocking mode:
      - w: Forward (accelerate)
      - s: Backward (decelerate)
      - a: Turn left (increase steering angle)
      - d: Turn right (decrease steering angle)
      - m: Change gear (loop through P, R, N, D)
      - b: Increase brake
      - B: Decrease brake
      - p: Toggle Electronic Parking Brake (EPB)
      - q: Exit the program
    """

    def __init__(
        self,
        screen,
        speed_delta=SPEED_DELTA,
        steering_delta=STEERING_DELTA,
        brake_delta=BRAKE_DELTA,
    ):
        self.screen = screen
        self.running = True
        self.control_cmd_msg = control_cmd_pb2.ControlCommand()
        self.speed = 0
        self.steering = 0.0
        self.speed_delta = speed_delta
        self.steering_delta = steering_delta
        # check definition of in modules/common_msgs/chassis_msgs/chassis.proto
        self.gear_list = [3, 2, 0, 1]
        self.gear_str = ["P", "R", "N", "D"]
        self.gear_index = 0
        self.gear = 3
        self.brake = 0
        self.brake_delta = brake_delta
        self.epb = 0
        self.lock = threading.Lock()
        self.logger = logging.getLogger(__name__)

        self.help_text_lines = [
            "Key instructions:",
            "  w/s: Increase/Decrease speed",
            "  a/d: Turn left/right",
            "  m: Change gear",
            "  b/B: Brake +/-",
            "  p: Toggle Electronic Parking Brake (EPB)",
            "  Space: Emergency stop",
            "  q: Quit program",
        ]

        # Key mapping: map keys using ASCII codes
        # TODO(All): add control mode, throttle, speed, acceleration, etc.
        self.control_map = {
            ord("w"): self.move_forward,
            ord("s"): self.move_backward,
            ord("a"): self.turn_left,
            ord("d"): self.turn_right,
            ord("m"): self.loop_gear,
            ord("b"): self.brake_inc,
            ord("B"): self.brake_dec,
            ord("p"): self.toggle_epb,
            ord(" "): self.emergency_stop,  # space key
        }

    def get_control_cmd(self):
        """Returns the latest control command message."""
        with self.lock:
            return self.control_cmd_msg

    def start(self):
        """Starts keyboard listening, sets curses to non-blocking mode and starts the listening thread."""
        self.screen.nodelay(True)  # Set non-blocking input
        self.screen.keypad(True)
        self.screen.addstr(0, 0, "Keyboard control started, press 'q' to exit.    ")
        for idx, line in enumerate(self.help_text_lines):
            self.screen.addstr(8 + idx, 0, line)
        self.thread = threading.Thread(target=self._listen_keyboard, daemon=True)
        self.thread.start()

    def stop(self):
        """Stops keyboard listening."""
        with self.lock:
            self.running = False
        self.screen.addstr(1, 0, "Keyboard control stopped.                    ")

    def _listen_keyboard(self):
        """Loop reads keyboard input and calls the corresponding control method based on the key pressed."""
        while self.running:
            try:
                key = self.screen.getch()  # Non-blocking call
            except Exception as e:
                print(f"Error reading keyboard input: {e}")
                key = -1

            if key != -1:
                if key == ord("q"):
                    self.stop()
                    break
                elif key in self.control_map:
                    with self.lock:
                        self.control_map[key]()
                self.fill_control_cmd()
            self.fill_control_cmd_header()
            time.sleep(0.05)

    def fill_control_cmd_header(self):
        """Fills the header of the control command message."""
        with self.lock:
            self.control_cmd_msg.header.timestamp_sec = (
                datetime.datetime.now().timestamp()
            )
            self.control_cmd_msg.header.module_name = "can_easy"
            self.control_cmd_msg.header.sequence_num += 1

    def fill_control_cmd(self):
        """Updates the current speed and steering to the protobuf message."""
        with self.lock:
            # TODO(All): start auto-drive via keyboard input
            self.control_cmd_msg.pad_msg.driving_mode = 1
            self.control_cmd_msg.pad_msg.action = 1
            # TODO(All): set control command via control mode
            self.control_cmd_msg.throttle = self.speed
            self.control_cmd_msg.speed = self.speed
            self.control_cmd_msg.steering_target = self.steering
            self.control_cmd_msg.gear_location = self.gear
            self.control_cmd_msg.brake = self.brake
            if self.epb == 1:
                self.control_cmd_msg.parking_brake = True
            else:
                self.control_cmd_msg.parking_brake = False
            # TODO(All): set signal via keyboard input
            if self.steering > 0:
                self.control_cmd_msg.signal.turn_signal = 2
            elif self.steering < 0:
                self.control_cmd_msg.signal.turn_signal = 1
            else:
                self.control_cmd_msg.signal.turn_signal = 0

    def move_forward(self):
        self.speed = min(self.speed + self.speed_delta, SPEED_MAX)
        self.screen.addstr(
            2, 0, f"speed: {self.speed:.2f} [{SPEED_MIN}, {SPEED_MAX}]    "
        )

    def move_backward(self):
        self.speed = max(self.speed - self.speed_delta, SPEED_MIN)
        self.screen.addstr(
            2, 0, f"speed: {self.speed:.2f} [{SPEED_MIN}, {SPEED_MAX}]    "
        )

    def turn_left(self):
        self.steering = min(self.steering + self.steering_delta, STEERING_MAX)
        self.screen.addstr(
            3, 0, f"steer: {self.steering:.2f} [{STEERING_MIN}, {STEERING_MAX}]    "
        )

    def turn_right(self):
        self.steering = max(self.steering - self.steering_delta, STEERING_MIN)
        self.screen.addstr(
            3, 0, f"steer: {self.steering:.2f} [{STEERING_MIN}, {STEERING_MAX}]    "
        )

    def brake_inc(self):
        self.brake = min(self.brake + self.brake_delta, BRAKE_MAX)
        self.screen.addstr(
            5, 0, f"brake: {self.brake:.2f} [{BRAKE_MIN}, {BRAKE_MAX}]    "
        )

    def brake_dec(self):
        self.brake = max(self.brake - self.brake_delta, BRAKE_MIN)
        self.screen.addstr(
            5, 0, f"brake: {self.brake:.2f} [{BRAKE_MIN}, {BRAKE_MAX}]    "
        )

    def loop_gear(self):
        self.gear_index = (self.gear_index + 1) % len(self.gear_list)
        self.gear = self.gear_list[self.gear_index]
        self.screen.addstr(4, 0, f"gear:  {self.gear_str[self.gear_index]}")

    def toggle_epb(self):
        """Toggle Electronic Parking Brake (EPB) state."""
        if self.epb == 0:
            self.epb = 1
        else:
            self.epb = 0
        self.screen.addstr(6, 0, f"epb:   {self.epb}")

    def emergency_stop(self):
        self.speed = 0
        self.brake = BRAKE_MAX
        self.screen.addstr(7, 0, "Emergency Stop activated!       ")


def main(screen):
    # Configure logging at the program entry point
    logging.basicConfig(level=logging.INFO)
    cyber.init()
    node = cyber.Node("can_easy")
    writer = node.create_writer(CONTROL_TOPIC, control_cmd_pb2.ControlCommand)

    # Pre-print the fixed format lines
    screen.addstr(2, 0, "speed: 0.00    ")
    screen.addstr(3, 0, "steer: 0.00    ")
    screen.addstr(4, 0, "gear:  P")
    screen.addstr(5, 0, "brake: 0.00    ")
    screen.addstr(6, 0, "epb:   0       ")

    controller = KeyboardController(screen)
    controller.start()

    try:
        while controller.running:
            cmd = controller.get_control_cmd()
            writer.write(cmd)
            time.sleep(0.1)
    except KeyboardInterrupt:
        controller.stop()
    finally:
        controller.stop()
        cyber.shutdown()

    screen.addstr(6, 0, "Program exited.                        ")


if __name__ == "__main__":
    # Use curses.wrapper to ensure proper initialization and cleanup of the curses environment
    curses.wrapper(main)
