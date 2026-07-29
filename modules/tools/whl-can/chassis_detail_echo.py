#!/usr/bin/env python3

import argparse
import importlib
import logging
import signal
import time

from cyber.python.cyber_py3 import cyber
from wheelos_msgs.chassis_msgs.chassis_detail_pb2 import ChassisDetail

# Override module path for specific message classes if needed.
MODULE_OVERRIDES = {
    # "LincolnMKZ": "modules.canbus.vehicle.lincoln.proto.lincoln_pb2",
}


logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s",
)

_logger = logging.getLogger(__name__)

_message_class_cache = {}
_failed_classes = set()
_stop = False


def infer_module_name(class_name: str) -> str:
    """Infer protobuf module path from message name."""

    package = class_name.lower()

    return (
        f"modules.canbus.vehicle."
        f"{package}.proto."
        f"{package}_pb2"
    )


def load_message_class(class_name: str):
    """Load protobuf message class."""

    if class_name in _message_class_cache:
        return _message_class_cache[class_name]

    if class_name in _failed_classes:
        return None

    module_name = MODULE_OVERRIDES.get(
        class_name,
        infer_module_name(class_name),
    )

    try:
        module = importlib.import_module(module_name)
        cls = getattr(module, class_name)

    except ModuleNotFoundError:
        _logger.error(
            "Cannot import protobuf module.\n"
            "Message: %s\n"
            "Expected module:\n"
            "    %s\n\n"
            "Add real entry to MODULE_OVERRIDES, then re-run this script.",
            class_name,
            module_name,
        )
        _failed_classes.add(class_name)
        return None

    except AttributeError:
        _logger.error(
            "Message class '%s' not found in module '%s'.",
            class_name,
            module_name,
        )
        _failed_classes.add(class_name)
        return None

    _message_class_cache[class_name] = cls
    return cls


def extract_class_name(type_url: str) -> str:
    """Extract message name from protobuf Any.type_url."""

    if not type_url:
        return ""

    # Supports:
    #   apollo.canbus.Devkit
    #   type.googleapis.com/apollo.canbus.Devkit
    if "/" in type_url:
        type_url = type_url.rsplit("/", 1)[-1]

    return type_url.rsplit(".", 1)[-1]


def callback(chassis_detail: ChassisDetail):

    if not chassis_detail.HasField("chassis_extension"):
        return

    extension = chassis_detail.chassis_extension

    class_name = extract_class_name(extension.type_url)

    cls = load_message_class(class_name)

    if cls is None:
        return

    msg = cls()

    if not extension.Unpack(msg):
        _logger.warning(
            "Failed to unpack message: %s",
            class_name,
        )
        return

    _logger.info(
        "\n%s\n%s",
        "=" * 80,
        msg,
    )


def shutdown_handler(signum, frame):
    global _stop
    _stop = True


def main():

    parser = argparse.ArgumentParser(
        description="Echo ChassisDetail with automatic protobuf loading."
    )

    parser.add_argument(
        "--topic",
        default="/apollo/canbus/chassis_detail",
        help="Cyber topic.",
    )

    args = parser.parse_args()

    cyber.init("chassis_detail_echo")

    node = cyber.Node("chassis_detail_echo")

    reader = node.create_reader(
        args.topic,
        ChassisDetail,
        callback,
    )

    signal.signal(
        signal.SIGINT,
        shutdown_handler,
    )

    _logger.info(
        "Listening on %s",
        args.topic,
    )

    try:
        while not _stop and cyber.ok():
            time.sleep(0.2)

    finally:
        del reader
        cyber.shutdown()


if __name__ == "__main__":
    main()
