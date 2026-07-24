#!/usr/bin/env python3
# Copyright 2025 The WheelOS Team. All Rights Reserved.
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

# Created Date: 2026-01-20
# Author: lykling
"""
send_routing.py - Cyber RT Routing Request Sender Tool
"""

import sys
import time
import logging
import uuid
from typing import Optional
from select import select

import click
from google.protobuf import text_format

from cyber.python.cyber_py3 import cyber

from wheelos_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from wheelos_msgs.routing_msgs.routing_pb2 import RoutingRequest

# Defaults
DEFAULT_FILE = "RoutingRequest.txt"
DEFAULT_CHANNEL = "/apollo/routing_request"
DEFAULT_LOCALIZATION_CHANNEL = "/apollo/localization/pose"
DEFAULT_POSE_TIMEOUT = 10.0

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


class LocalizationTimeoutError(Exception):
    pass


class LocalizationSubscriber:

    def __init__(self,
                 node: cyber.Node,
                 channel: str = DEFAULT_LOCALIZATION_CHANNEL):
        self.latest_pose: Optional[LocalizationEstimate] = None
        self.node = node
        self.channel = channel

    def callback(self, msg: LocalizationEstimate):
        self.latest_pose = msg

    def subscribe(self):
        self.node.create_reader(self.channel, LocalizationEstimate,
                                self.callback)
        logger.info(f"Subscribed to: {self.channel}")

    def wait_for_pose(
            self,
            timeout: float = DEFAULT_POSE_TIMEOUT) -> LocalizationEstimate:
        start = time.time()
        while self.latest_pose is None:
            if time.time() - start >= timeout:
                raise LocalizationTimeoutError(
                    f"Timeout waiting for pose ({timeout}s)")
            time.sleep(0.05)
        return self.latest_pose


def load_request(filepath: str) -> Optional[RoutingRequest]:
    try:
        with open(filepath, "r") as f:
            content = f.read()
        request = RoutingRequest()
        text_format.Parse(content, request)
        logger.info(
            f"Loaded {len(request.waypoint)} waypoints from {filepath}")
        return request
    except Exception as e:
        logger.error(f"Failed to load {filepath}: {e}")
        return None


def add_current_pose(request: RoutingRequest,
                     pose: LocalizationEstimate) -> RoutingRequest:
    """Add current pose as first waypoint by copying and inserting"""
    new_request = RoutingRequest()
    new_request.CopyFrom(request)

    # Insert current pose at the beginning
    first_wp = new_request.waypoint.add()
    first_wp.pose.x = pose.pose.position.x
    first_wp.pose.y = pose.pose.position.y
    first_wp.pose.z = pose.pose.position.z
    first_wp.heading = pose.pose.heading

    # Move all original waypoints after the first one
    original_waypoints = list(request.waypoint)
    new_request.ClearField("waypoint")
    new_request.waypoint.add().CopyFrom(first_wp)
    for wp in original_waypoints:
        new_request.waypoint.add().CopyFrom(wp)

    logger.info(
        f"Added current pose, total waypoints: {len(new_request.waypoint)}")
    return new_request


def fill_header(request: RoutingRequest, seq: int):
    from wheelos_msgs.basic_msgs.header_pb2 import Header
    if not request.HasField("header"):
        request.header.CopyFrom(Header())
    request.header.timestamp_sec = time.time()
    request.header.sequence_num = seq
    request.header.module_name = "send_routing"


def send_request(writer, request: RoutingRequest, seq: int):
    fill_header(request, seq)
    ret = writer.write(request)

    # In Cyber, Write() returns bool: true=1 (success), false=0 (failure)
    if ret == 0:
        logger.error(f"Writer.write() FAILED (returned {ret})")
    elif ret == 1:
        logger.debug(f"Writer.write() succeeded (returned {ret})")
    else:
        logger.warning(f"Writer.write() returned unexpected value: {ret}")

    logger.info(
        f"Sent RoutingRequest (seq={seq}): {len(request.waypoint)} waypoints")


def interactive_loop(writer, request, seq_gen, pose_sub=None, pose_mode=None):
    click.echo("\n=== Interactive Mode ===\nc - send, q - quit\n")
    while not cyber.is_shutdown():
        click.echo("> ", nl=False)
        rlist, _, _ = select([sys.stdin], [], [], None)
        if rlist:
            line = sys.stdin.readline().strip().lower()
            if line == 'q':
                break
            elif line == 'c':
                if pose_sub and pose_mode == "fresh":
                    try:
                        pose = pose_sub.wait_for_pose(timeout=2.0)
                        req = add_current_pose(request, pose)
                        send_request(writer, req, next(seq_gen))
                    except LocalizationTimeoutError:
                        click.echo("No pose available", err=True)
                else:
                    send_request(writer, request, next(seq_gen))


def timed_loop(writer,
               request,
               interval,
               seq_gen,
               pose_sub=None,
               pose_mode=None,
               count=0,
               initial_wait=0):
    click.echo(f"Sending every {interval}s (Ctrl+C to stop)" + (f", max {count} times" if count > 0 else ""))

    # Wait before first send to allow service discovery
    if initial_wait > 0:
        logger.info(f"Waiting {initial_wait}s for service discovery before first send...")
        time.sleep(initial_wait)

    sent_count = 0
    while not cyber.is_shutdown():
        if count > 0 and sent_count >= count:
            click.echo(f"Sent {sent_count} times, exiting.")
            break

        if pose_sub and pose_mode == "fresh":
            try:
                pose = pose_sub.wait_for_pose(timeout=2.0)
                req = add_current_pose(request, pose)
                send_request(writer, req, next(seq_gen))
                sent_count += 1
            except LocalizationTimeoutError:
                logger.warning("No pose available, skipping")
        else:
            send_request(writer, request, next(seq_gen))
            sent_count += 1
        time.sleep(interval)


def seq_generator():
    i = 0
    while True:
        i += 1
        yield i


@click.command()
@click.option("-i",
              "--input",
              type=click.Path(exists=True),
              default=DEFAULT_FILE,
              help="Input file")
@click.option("-c",
              "--channel",
              type=str,
              default=DEFAULT_CHANNEL,
              help="Target channel")
@click.option("--add-pose",
              is_flag=True,
              help="Add current pose as first waypoint")
@click.option("--localization-channel",
              type=str,
              default=DEFAULT_LOCALIZATION_CHANNEL,
              help="Localization channel")
@click.option("--pose-timeout",
              type=float,
              default=DEFAULT_POSE_TIMEOUT,
              help="Pose timeout (seconds)")
@click.option("--loop", is_flag=True, help="Loop mode")
@click.option("--interval",
              type=float,
              default=1.0,
              help="Send interval (seconds)")
@click.option("--interactive",
              is_flag=True,
              help="Interactive mode (send on keypress)")
@click.option("--pose-mode",
              type=click.Choice(["once", "fresh"], case_sensitive=False),
              default="fresh",
              help="Pose refresh mode: once/fresh")
@click.option("--log-level",
              type=click.Choice(["DEBUG", "INFO", "WARNING", "ERROR"]),
              default="INFO",
              help="Log level")
@click.option("--wait",
              type=float,
              default=1.0,
              help="Wait time before sending for service discovery (seconds), default 1.0")
@click.option("--count",
              type=int,
              default=0,
              help="Number of times to send (0=infinite, loop mode only)")
def main(input, channel, add_pose, localization_channel, pose_timeout, loop,
         interval, interactive, pose_mode, log_level, wait, count):
    """Send RoutingRequest via Cyber RT.

    \b
    Examples:
        # Single send (default 1s wait for service discovery)
        send_routing -i routing.txt

        # Single send with custom wait time
        send_routing.py -i routing.txt --wait 2.0

        # Loop mode, send 3 times then exit
        send_routing.py -i routing.txt --loop --count 3

        # Infinite loop mode
        send_routing.py -i routing.txt --loop
    """

    logging.getLogger().setLevel(getattr(logging, log_level))

    if interactive and not loop:
        loop = True

    node = None
    try:
        cyber.init()
        node_name = f"send_routing_{uuid.uuid4().hex[:8]}"
        node = cyber.Node(node_name)

        writer = node.create_writer(channel, RoutingRequest)

        # Verify writer was actually created
        if writer is None:
            logger.error("Failed to create writer!")
            return 1
        if writer.writer is None:
            logger.error("Writer.writer is None! Message type may not be registered.")
            return 1

        request = load_request(input)
        if not request:
            return 1

        logger.debug(f"Request has {len(request.waypoint)} waypoints")
        if logger.level <= logging.DEBUG:
            for i, wp in enumerate(request.waypoint):
                logger.debug(f"  Waypoint {i}: x={wp.pose.x}, y={wp.pose.y}, heading={wp.heading}")
            if request.HasField("parking_info"):
                logger.debug(f"Request has parking_info: {request.parking_info.parking_space_id}")

        pose_sub = None
        if add_pose:
            pose_sub = LocalizationSubscriber(node, localization_channel)
            pose_sub.subscribe()
            click.echo(f"Waiting for pose (timeout={pose_timeout}s)...")
            try:
                pose = pose_sub.wait_for_pose(timeout=pose_timeout)
                click.echo(
                    f"Pose: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
                )
            except LocalizationTimeoutError as e:
                click.echo(f"Error: {e}", err=True)
                click.echo("Ensure localization is running", err=True)
                return 1

        if not loop:
            if add_pose:
                pose = pose_sub.latest_pose
                request = add_current_pose(request, pose)

            # Wait before sending to allow service discovery to register the writer
            # This is critical: the first send often fails because the writer hasn't been
            # discovered by subscribers yet. Waiting here gives service discovery time.
            if wait > 0:
                logger.info(f"Waiting {wait}s for service discovery to register writer...")
                time.sleep(wait)

            send_request(writer, request, 1)
            return 0

        seq_gen = seq_generator()

        if add_pose and pose_mode == "once":
            pose = pose_sub.latest_pose
            request = add_current_pose(request, pose)
            click.echo("Using cached pose for all iterations")
            if interactive:
                interactive_loop(writer, request, seq_gen)
            else:
                timed_loop(writer, request, interval, seq_gen, count=count, initial_wait=wait)
        elif add_pose and pose_mode == "fresh":
            click.echo("Getting fresh pose each iteration")
            if interactive:
                interactive_loop(writer, request, seq_gen, pose_sub, pose_mode)
            else:
                timed_loop(writer, request, interval, seq_gen, pose_sub, pose_mode, count, initial_wait=wait)
        else:
            if interactive:
                interactive_loop(writer, request, seq_gen)
            else:
                timed_loop(writer, request, interval, seq_gen, count=count, initial_wait=wait)

        return 0

    except KeyboardInterrupt:
        click.echo("\nInterrupted")
        return 0
    except Exception as e:
        logger.exception(f"Error: {e}")
        return 1
    finally:
        # IMPORTANT: Shutdown in correct order to ensure messages are sent
        # 1. Delete all writers first to flush pending messages
        # 2. Then shutdown Cyber
        if node:
            logger.debug("Cleaning up writers...")
            for writer in node.list_writer:
                cyber._CYBER.delete_PyWriter(writer)
            node.list_writer.clear()
            logger.debug("Shutting down Cyber...")
            cyber.shutdown()


if __name__ == "__main__":
    sys.exit(main())
