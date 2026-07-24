#!/usr/bin/env python3

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
from wheelos_msgs.control_msgs import control_cmd_pb2

CONTROL_TOPIC = "/apollo/control"

SPEED_MIN, SPEED_MAX = -5.0, 5.0
STEERING_MIN, STEERING_MAX = -100.0, 100.0
THROTTLE_MIN, THROTTLE_MAX = 0.0, 100.0
BRAKE_MIN, BRAKE_MAX = 0.0, 100.0
ACCEL_MIN, ACCEL_MAX = -5.0, 5.0

SPEED_DELTA = 0.1
STEERING_DELTA = 5
THROTTLE_DELTA = 5
BRAKE_DELTA = 1
TURN_SIGNAL_THRESHOLD_DELTA = 1.0
ACCEL_DELTA = 0.1

DEFAULT_STEERING_RATE = 0.0
DEFAULT_TURN_SIGNAL_THRESHOLD = 0.0

CONTROL_MODE_SPEED = "SPEED"
CONTROL_MODE_THROTTLE = "THROTTLE"
CONTROL_MODE_ACCEL = "ACCEL"


class KeyboardController:
    """
    Curses-based keyboard controller for Apollo Cyber RT.
    """

    def __init__(self, screen):
        self.screen = screen
        self.logger = logging.getLogger("KeyboardController")

        self.node = cyber.Node("can_easy")
        self.writer = self.node.create_writer(
            CONTROL_TOPIC, control_cmd_pb2.ControlCommand
        )
        self.control_cmd_msg = control_cmd_pb2.ControlCommand()
        self.sequence_num = 0

        # Start in manual mode for safety; autonomous mode must be explicitly
        # enabled/taken over by the operator (e.g., via the Enter key).
        self.running = True
        self.engage = False
        self.pad_only_pending = True

        self.control_mode = CONTROL_MODE_THROTTLE
        self.speed = 0.0
        self.throttle = 0.0
        self.acceleration = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.steering_rate = DEFAULT_STEERING_RATE
        self.turn_signal_threshold = DEFAULT_TURN_SIGNAL_THRESHOLD

        self.speed_delta = SPEED_DELTA
        self.steering_delta = STEERING_DELTA
        self.brake_delta = BRAKE_DELTA

        self.gear_list = [3, 2, 0, 1]
        self.gear_names = ["P", "R", "N", "D"]
        self.gear_index = 0
        self.gear = 3
        self.epb = 0

        self.turn_signal = 0
        self.horn = False
        self.high_beam = False
        self.low_beam = False
        self.emergency_light = False

        self.show_help = False
        self.msg_log = "System Ready."
        self.msg_time = time.time()
        self.lock = threading.Lock()

        self.control_map = {
            ord("w"): self.move_forward,
            ord("s"): self.move_backward,
            ord("a"): self.turn_left,
            ord("d"): self.turn_right,
            ord("b"): self.brake_inc,
            ord("n"): self.brake_dec,
            ord(" "): self.emergency_stop,
            10: self.toggle_engage,
            27: self.quit_program,
            ord("h"): self.toggle_help,
            ord("g"): self.loop_gear,
            ord("p"): self.toggle_epb,
            ord("o"): self.toggle_horn,
            ord("l"): self.cycle_lights,
            ord("q"): lambda: self.toggle_turn_signal(1, "LEFT"),
            ord("e"): lambda: self.toggle_turn_signal(2, "RIGHT"),
            ord("m"): lambda: self.toggle_turn_signal(3, "HAZARD"),
            ord("1"): self.set_mode_speed,
            ord("2"): self.set_mode_throttle,
            ord("3"): self.set_mode_accel,
        }

    def start(self):
        self.screen.nodelay(True)
        self.screen.keypad(True)
        curses.curs_set(0)
        if curses.has_colors():
            curses.start_color()
            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
            curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
            curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
            curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)

        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        with self.lock:
            self.running = False

    def _loop(self):
        while True:
            with self.lock:
                if not self.running:
                    break
            self._handle_input()
            self._publish_command()
            self._render_ui()
            time.sleep(0.1)

    def _handle_input(self):
        try:
            key = self.screen.getch()
            if key != -1 and key in self.control_map:
                with self.lock:
                    self.control_map[key]()
        except Exception as e:
            self.log_msg(f"Input Error: {e}")

    def _publish_command(self):
        with self.lock:
            # TODO(zero): Add entry to autonomous driving inspection
            if self.pad_only_pending:
                cmd = control_cmd_pb2.ControlCommand()
                cmd.header.timestamp_sec = datetime.datetime.now().timestamp()
                cmd.header.module_name = "can_easy"
                self.sequence_num += 1
                cmd.header.sequence_num = self.sequence_num
                cmd.pad_msg.driving_mode = 1 if self.engage else 0
                cmd.pad_msg.action = 1 if self.engage else 0
                self.pad_only_pending = False
                self.writer.write(cmd)
                return

            cmd = self.control_cmd_msg
            cmd.header.timestamp_sec = datetime.datetime.now().timestamp()
            cmd.header.module_name = "can_easy"
            self.sequence_num += 1
            cmd.header.sequence_num = self.sequence_num

            cmd.ClearField("pad_msg")
            cmd.ClearField("speed")
            cmd.ClearField("throttle")
            cmd.ClearField("acceleration")

            if self.brake > 0:
                cmd.speed = 0.0
                cmd.throttle = 0.0
                cmd.acceleration = 0.0
            elif self.control_mode == CONTROL_MODE_SPEED:
                cmd.speed = self.speed
            elif self.control_mode == CONTROL_MODE_THROTTLE:
                cmd.throttle = self.throttle
            else:
                cmd.acceleration = self.acceleration

            cmd.steering_target = self.steering
            cmd.steering_rate = self.steering_rate
            cmd.gear_location = self.gear
            cmd.brake = self.brake
            cmd.parking_brake = bool(self.epb)

            cmd.signal.horn = self.horn
            cmd.signal.high_beam = self.high_beam
            cmd.signal.low_beam = self.low_beam
            cmd.signal.emergency_light = self.emergency_light

            if self.turn_signal in [1, 2, 3]:
                cmd.signal.turn_signal = self.turn_signal
            elif self.turn_signal_threshold <= 0:
                cmd.signal.ClearField("turn_signal")
            else:
                if self.steering > self.turn_signal_threshold:
                    cmd.signal.turn_signal = 1
                elif self.steering < -self.turn_signal_threshold:
                    cmd.signal.turn_signal = 2
                else:
                    cmd.signal.turn_signal = 0

            self.writer.write(cmd)

    def log_msg(self, msg):
        self.msg_log = msg
        self.msg_time = time.time()

    def quit_program(self):
        self.running = False

    def toggle_help(self):
        self.show_help = not self.show_help

    def move_forward(self):
        if self.control_mode == CONTROL_MODE_SPEED:
            self.speed = min(self.speed + self.speed_delta, SPEED_MAX)
        elif self.control_mode == CONTROL_MODE_THROTTLE:
            self.throttle = min(self.throttle + THROTTLE_DELTA, THROTTLE_MAX)
        else:
            self.acceleration = min(self.acceleration + ACCEL_DELTA, ACCEL_MAX)
        self.update_longitudinal_status()

    def move_backward(self):
        if self.control_mode == CONTROL_MODE_SPEED:
            self.speed = max(self.speed - self.speed_delta, SPEED_MIN)
        elif self.control_mode == CONTROL_MODE_THROTTLE:
            self.throttle = max(self.throttle - THROTTLE_DELTA, THROTTLE_MIN)
        else:
            self.acceleration = max(self.acceleration - ACCEL_DELTA, ACCEL_MIN)
        self.update_longitudinal_status()

    def turn_left(self):
        self.steering = min(self.steering + self.steering_delta, STEERING_MAX)

    def turn_right(self):
        self.steering = max(self.steering - self.steering_delta, STEERING_MIN)

    def brake_inc(self):
        self.brake = min(self.brake + self.brake_delta, BRAKE_MAX)
        self.throttle = 0.0
        self.acceleration = 0.0
        self.speed = 0.0

    def brake_dec(self):
        self.brake = max(self.brake - self.brake_delta, BRAKE_MIN)
        self.throttle = 0.0
        self.acceleration = 0.0
        self.speed = 0.0

    def loop_gear(self):
        if self.control_mode == CONTROL_MODE_SPEED:
            current_val = self.speed
        elif self.control_mode == CONTROL_MODE_THROTTLE:
            current_val = self.throttle
        else:
            current_val = self.acceleration
        if abs(current_val) > 0.1 and self.brake < 10:
            self.log_msg("Speed too high to shift!")
            return
        self.gear_index = (self.gear_index + 1) % len(self.gear_list)
        self.gear = self.gear_list[self.gear_index]
        self.log_msg(f"Gear switched to: {self.gear_names[self.gear_index]}")

    def toggle_epb(self):
        self.epb = 0 if self.epb == 1 else 1
        self.log_msg(f"EPB {'ON' if self.epb else 'OFF'}")

    def toggle_horn(self):
        self.horn = not self.horn
        self.log_msg(f"Horn {'ON' if self.horn else 'OFF'}")

    def toggle_emergency_light(self):
        self.emergency_light = not self.emergency_light
        self.log_msg(f"Emergency Light {'ON' if self.emergency_light else 'OFF'}")

    def cycle_lights(self):
        if self.high_beam:
            self.high_beam = False
            self.low_beam = False
            self.log_msg("Lights: OFF")
        elif self.low_beam:
            self.low_beam = False
            self.high_beam = True
            self.log_msg("Lights: HIGH")
        else:
            self.low_beam = True
            self.high_beam = False
            self.log_msg("Lights: LOW")

    def set_turn_signal(self, signal_val, signal_str):
        self.turn_signal = signal_val
        self.log_msg(f"Turn Signal: {signal_str}")

    def toggle_turn_signal(self, signal_val, signal_str):
        if self.turn_signal == signal_val:
            self.set_turn_signal(0, "NONE")
        else:
            self.set_turn_signal(signal_val, signal_str)

    def toggle_engage(self):
        self.engage = not self.engage
        self.pad_only_pending = True
        state = "ENGAGED" if self.engage else "DISENGAGED"
        self.log_msg(f"Auto-drive {state}")

    def emergency_stop(self):
        self.speed = 0.0
        self.throttle = 0.0
        self.acceleration = 0.0
        self.brake = BRAKE_MAX
        self.log_msg("!!! EMERGENCY STOP ACTIVATED !!!")

    def set_mode_speed(self):
        self.control_mode = CONTROL_MODE_SPEED
        self.update_longitudinal_status()
        self.log_msg("Mode set to: SPEED")

    def set_mode_throttle(self):
        self.control_mode = CONTROL_MODE_THROTTLE
        self.update_longitudinal_status()
        self.log_msg("Mode set to: THROTTLE")

    def set_mode_accel(self):
        self.control_mode = CONTROL_MODE_ACCEL
        self.update_longitudinal_status()
        self.log_msg("Mode set to: ACCEL")

    def update_longitudinal_status(self):
        pass

    def _draw_bar(self, val, max_val, width=12):
        percent = max(0.0, min(1.0, val / max_val)) if max_val != 0 else 0
        filled = int(width * percent)
        return "[" + "|" * filled + "." * (width - filled) + "]"

    def _render_ui(self):
        scr = self.screen
        h, w = scr.getmaxyx()
        scr.erase()

        min_h, min_w = 24, 80
        if h < min_h or w < min_w:
            max_chars = max(0, w - 1)
            if h > 0 and max_chars > 0:
                scr.addnstr(0, 0, "Terminal window is too small.", max_chars)
            if h > 1 and max_chars > 0:
                scr.addnstr(1, 0, "Resize to at least 80x24.", max_chars)
            scr.refresh()
            return

        try:
            style_header = curses.A_BOLD | (
                curses.color_pair(1) if self.engage else curses.color_pair(3)
            )
            style_brake = curses.color_pair(2) if self.brake > 0 else 0

            status_text = " AUTONOMOUS " if self.engage else "   MANUAL   "
            scr.addstr(0, 0, "=" * w)
            scr.addstr(1, 2, f"CONTROL MODE: {status_text}", style_header)
            scr.addstr(1, w - 20, datetime.datetime.now().strftime("%H:%M:%S"))
            scr.addstr(2, 0, "-" * w)

            c1 = 2
            scr.addstr(
                3, c1, f"GEAR:    [{self.gear_names[self.gear_index]}]", curses.A_BOLD
            )
            scr.addstr(4, c1, f"EPB:     [{'ON' if self.epb else ' '}]")
            scr.addstr(5, c1, f"SIGNAL:  {self._signal_text()}")

            c2 = 20
            bar_speed = self._draw_bar(abs(self.speed), SPEED_MAX, 12)
            scr.addstr(3, c2, f"SPD: {self.speed:5.2f} {bar_speed}")
            bar_throttle = self._draw_bar(self.throttle, THROTTLE_MAX, 12)
            scr.addstr(4, c2, f"THR: {self.throttle:5.2f} {bar_throttle}")
            bar_accel = self._draw_bar(abs(self.acceleration), ACCEL_MAX, 12)
            scr.addstr(5, c2, f"ACC: {self.acceleration:5.2f} {bar_accel}")
            bar_brake = self._draw_bar(self.brake, BRAKE_MAX, 12)
            scr.addstr(6, c2, f"BRK: {self.brake:5.2f} {bar_brake}", style_brake)

            c3 = 55
            steer_steps = 8
            steer_visual = "." * 4 + "|" + "." * 4
            idx = int(
                round(
                    (self.steering - STEERING_MIN)
                    * steer_steps
                    / (STEERING_MAX - STEERING_MIN)
                )
            )
            # reverse to match intuitive display, `+` for left, `-` for right
            idx = max(0, min(steer_steps, steer_steps - idx))
            s_list = list(steer_visual)
            if 0 <= idx < len(s_list):
                s_list[idx] = "O"
            scr.addstr(3, c3, f"STR:[L{''.join(s_list)}R] {self.steering:+.1f}")
            scr.addstr(4, c3, f"LOW: {'ON' if self.low_beam else 'OFF'}")
            scr.addstr(
                5,
                c3,
                f"HIGH:{'ON' if self.high_beam else 'OFF'}",
            )

            scr.addstr(11, 0, "-" * w)
            if time.time() - self.msg_time < 3.0:
                scr.addstr(9, 2, f">> {self.msg_log}", curses.A_BOLD)
            else:
                scr.addstr(9, 2, ">>")

            scr.addstr(11, 0, "=" * w)
            if not self.show_help:
                scr.addstr(
                    12, 2, "[H] Help | [Space] E-Stop | [Enter] Auto | [Esc] Quit"
                )
            else:
                scr.addstr(
                    12, 2, "W/S: Drive | A/D: Steer | B/N: Brake +/- | Space: E-Stop"
                )
                scr.addstr(13, 2, "G: Gear | P: EPB | 1/2/3: Mode | Enter: Auto-Drive")
                scr.addstr(14, 2, "q/e/m: Left/Right/Hazard (toggle) | o: Horn")
                scr.addstr(15, 2, "l: Lights (Off/Low/High)")

            scr.refresh()
        except curses.error:
            scr.erase()
            h, w = scr.getmaxyx()
            max_chars = max(0, w - 1)
            if h > 0 and max_chars > 0:
                scr.addnstr(0, 0, "UI render error, try resizing terminal.", max_chars)
            try:
                scr.refresh()
            except curses.error:
                pass

    def _signal_text(self):
        if self.turn_signal == 1:
            return "LEFT"
        if self.turn_signal == 2:
            return "RIGHT"
        if self.turn_signal == 3:
            return "HAZARD"
        return "NONE"


def main(screen):
    logging.basicConfig(level=logging.INFO)
    cyber.init()
    max_y, max_x = screen.getmaxyx()
    if max_y < 24:
        screen.addstr(0, 0, "Error: Terminal window is too small.")
        screen.addstr(
            1, 0, "Please resize your terminal to at least 24 rows and try again."
        )
        screen.refresh()
        time.sleep(3)
        return
    controller = KeyboardController(screen)
    controller.start()

    try:
        while controller.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        controller.stop()
    finally:
        controller.stop()
        cyber.shutdown()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except curses.error as e:
        print(f"Error initializing curses: {e}")
        print(
            "Please ensure your terminal is large enough (e.g., 80x25) and try again."
        )
