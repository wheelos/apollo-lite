#!/usr/bin/env python3

# Copyright 2026 The WheelOS Team. All Rights Reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Created Date: 2026-02-09
# Author: daohu527


import serial
import threading
import time
from datetime import datetime

SERIAL_PORT = "/dev/ttyUSB0"  # Replace with your device name /dev/ttyUSB0
BAUDRATE = 460800
READ_TIMEOUT = 1  # seconds

# Command list (modify as needed)
COMMANDS = [
    "unlogall",
    "com com1 460800",
    "log com1 gpchcx ontime 0.01",
    "log com1 gpgga ontime 1",
    "log com3 gprmc gprmc ontime 1",
    "insangle 0 0 0 5 5 5",
    "headingoffset 0 0 19.13 5.00 5.00 5.00",
    "ant2bodyoffset -0.855 0.14 -0.42 1.0 1.0 1.0",
    "ins2antoffset 0.56 -3.08 0.28 1.0 1.0 1.0",
    "ant2outposoffset 2 -0.855 2.465 -0.28",
    "wheeltread 1.82 4.3",
    "bodytype 2 0 0",
    "saveconfig",
]

stop_event = threading.Event()


def reader_thread(ser):
    """Continuously read from the serial port and print with timestamp."""
    while not stop_event.is_set():
        try:
            # readline() returns when it encounters a newline or times out
            raw = ser.readline()
        except Exception as e:
            print(f"[reader] read error: {e}")
            break

        if not raw:
            continue
        try:
            text = raw.decode(errors="replace").rstrip("\r\n")
        except Exception:
            text = repr(raw)
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{ts}] <<< {text}")


def send_command_and_wait_echo(ser, cmd, wait_for=None, timeout=2.0):
    """
    Send the command and wait for an echo, returning `None` if it times out.
    If `wait_for` is `None`, wait for the next non-empty line from the device.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    line_ending = "\r\n"
    raw_cmd = (cmd + line_ending).encode()
    ser.write(raw_cmd)
    ser.flush()
    print(f">>> Sent: {cmd}")

    deadline = time.time() + timeout
    buf = ""
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        try:
            text = raw.decode(errors="replace").rstrip("\r\n")
        except Exception:
            text = repr(raw)
        # Directly return the first non-empty line, or return when it contains wait_for
        if wait_for:
            if wait_for in text:
                return text
        else:
            if text.strip():
                return text
    return None


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=READ_TIMEOUT)
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    try:
        for cmd in COMMANDS:
            res = send_command_and_wait_echo(ser, cmd, wait_for=cmd, timeout=1.0)
            if res is None:
                print(
                    f"[WARN] Did not detect an echo containing `{cmd}` within the timeout period (the device may only respond with different text)."
                )
            else:
                print(f"[ACK] Received echo containing command: {res}")
            # Adjust the interval as needed for the device
            time.sleep(0.05)
    finally:
        # Stop the reader thread and close the serial port
        stop_event.set()
        time.sleep(0.1)
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
