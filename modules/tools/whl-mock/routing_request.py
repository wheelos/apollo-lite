# Copyright 2025 The WheelOS Team. All Rights Reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Created Date: 2025-10-17
# Author: daohu527


import time
import logging
import math
from typing import Optional

from google.protobuf.message import Message

from cyber.python.cyber_py3 import cyber

from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.routing_msgs.routing_pb2 import RoutingRequest

# ================= USER CONFIGURATION =================

ORIGINAL_END_X = -3.886315019772555
ORIGINAL_END_Y = 37.61260329902406
ORIGINAL_END_HEADING = 1.485286644

EXTEND_LENGTH = 5.23 / 2


# Topics
LOCALIZATION_TOPIC = "/apollo/localization/pose"
ROUTING_REQUEST_TOPIC = "/apollo/routing_request"

# Publish frequency (Hz)
PUBLISH_FREQUENCY = 1.0

# ================= END USER CONFIGURATION =================


# Global state variables
latest_localization_msg: Optional[LocalizationEstimate] = None
global_sequence_num = 0
routing_published = False


def move_by_heading(x, y, heading, length):
    """Calculate new coordinates based on heading and length"""
    dx = length * math.cos(heading)
    dy = length * math.sin(heading)
    new_x = x + dx
    new_y = y + dy
    return new_x, new_y


# Calculate the actual end point coordinates used
END_X, END_Y = move_by_heading(
    ORIGINAL_END_X, ORIGINAL_END_Y, ORIGINAL_END_HEADING, EXTEND_LENGTH
)

# Print the actual end point coordinates used
print(
    f"Calculated End Point: X={END_X}, Y={END_Y}",
    f"END_HEADING={ORIGINAL_END_HEADING}",
)


# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)


def localization_callback(msg: LocalizationEstimate):
    """
    Localization message callback function.
    """
    global latest_localization_msg, routing_published

    if routing_published:
        return

    latest_localization_msg = msg
    logging.debug(
        f"Received Localization: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}"
    )


def fill_header(msg: Message):
    """
    Fill the message header (Header) fields and increment sequence number.
    """
    global global_sequence_num
    if hasattr(msg, "header"):
        msg.header.timestamp_sec = time.time()
        msg.header.module_name = "routing_publisher"

        global_sequence_num += 1
        msg.header.sequence_num = global_sequence_num


def create_and_publish_routing_request(
    writer: cyber.Writer,
    end_x: float,
    end_y: float,
    end_heading: float,
) -> bool:
    """
    Create and publish a RoutingRequest message. Returns True if successful.
    """
    global latest_localization_msg

    if latest_localization_msg is None:
        logging.warning("Waiting for the first LocalizationEstimate message...")
        return False

    # 1. Get the localization message as the starting point (waypoint[0])
    start_pose = latest_localization_msg.pose
    start_position = start_pose.position

    # Construct RoutingRequest
    routing_request = RoutingRequest()
    fill_header(routing_request)

    # Construct start waypoint [0]
    start_waypoint = routing_request.waypoint.add()
    start_waypoint.pose.x = start_position.x
    start_waypoint.pose.y = start_position.y

    # Construct end waypoint [1]
    end_waypoint = routing_request.waypoint.add()
    end_waypoint.pose.x = end_x
    end_waypoint.pose.y = end_y
    end_waypoint.heading = end_heading

    # Publish message
    writer.write(routing_request)
    logging.info(
        f"✅ Published RoutingRequest (Seq: {routing_request.header.sequence_num}): "
        f"Start=({start_position.x:.2f}, {start_position.y:.2f}), "
        f"End=({end_x:.2f}, {end_y:.2f})"
    )
    return True


def main():
    """
    Main program entry point.
    """
    global routing_published

    try:
        cyber.init()
        node = cyber.Node("routing_request_publisher_node")

        # 1. Create a positioning message subscriber
        node.create_reader(
            LOCALIZATION_TOPIC, LocalizationEstimate, localization_callback
        )
        logging.info(f"Subscribed to topic: {LOCALIZATION_TOPIC}")

        # 2. Create a RoutingRequest message publisher
        writer = node.create_writer(ROUTING_REQUEST_TOPIC, RoutingRequest)
        logging.info(f"Ready to publish to topic: {ROUTING_REQUEST_TOPIC}")

        # 3. Core loop to wait for localization messages and publish requests
        sleep_time = 1.0 / PUBLISH_FREQUENCY

        while not cyber.is_shutdown():
            if not routing_published:
                routing_published = create_and_publish_routing_request(
                    writer, END_X, END_Y, ORIGINAL_END_HEADING
                )

            # If published successfully, gracefully shut down Cyber
            if routing_published:
                logging.info(
                    "RoutingRequest published. Initiating graceful shutdown..."
                )
                cyber.shutdown()

            time.sleep(sleep_time)

    except KeyboardInterrupt:
        logging.info("Publishing stopped by user.")
    except Exception:
        logging.exception("An unexpected error occurred during publishing.")
    finally:
        if not cyber.is_shutdown():
            cyber.shutdown()
        logging.info("Cyber RT shutdown complete.")


if __name__ == "__main__":
    main()
