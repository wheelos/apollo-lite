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

import sys
import select
import argparse
import time
import logging
from typing import Optional, Any, Type, Dict
import uuid

from google.protobuf import text_format
from google.protobuf.message import Message

from cyber.python.cyber_py3 import cyber

# ================= CHANNEL TO MESSAGE TYPE MAPPING ====================
# Common channel to message type mappings for Apollo Cyber RT
# Format: "channel_name": ("import_path", "MessageClassName")
CHANNEL_MESSAGE_TYPE_MAP: Dict[str, tuple] = {
    # Planning
    "/apollo/planning": (
        "wheelos_msgs.planning_msgs.planning_pb2",
        "ADCTrajectory"
    ),
    "/apollo/planning/pad": (
        "wheelos_msgs.planning_msgs.planning_pb2",
        "PlanningPadMsg"
    ),

    # Prediction
    "/apollo/prediction": (
        "wheelos_msgs.prediction_msgs.prediction_obstacle_pb2",
        "PredictionObstacles"
    ),

    # Routing
    "/apollo/routing_request": (
        "wheelos_msgs.routing_msgs.routing_pb2",
        "RoutingRequest"
    ),
    "/apollo/routing_response": (
        "wheelos_msgs.routing_msgs.routing_pb2",
        "RoutingResponse"
    ),

    # Control
    "/apollo/control": (
        "wheelos_msgs.control_msgs.control_cmd_pb2",
        "ControlCommand"
    ),
    "/apollo/control/pad": (
        "wheelos_msgs.control_msgs.pad_msg_pb2",
        "PadMessage"
    ),

    # Canbus
    "/apollo/canbus/chassis": (
        "wheelos_msgs.chassis_msgs.chassis_pb2",
        "Chassis"
    ),
    "/apollo/canbus/chassis_detail": (
        "wheelos_msgs.chassis_msgs.chassis_detail_pb2",
        "ChassisDetail"
    ),

    # Localization
    "/apollo/localization/pose": (
        "wheelos_msgs.localization_msgs.localization_pb2",
        "LocalizationEstimate"
    ),
    "/apollo/localization/msf_gnss": (
        "wheelos_msgs.localization_msgs.localization_pb2",
        "LocalizationEstimate"
    ),
    "/apollo/localization/msf_lidar": (
        "wheelos_msgs.localization_msgs.localization_pb2",
        "LocalizationEstimate"
    ),
    "/apollo/localization/ndt_lidar": (
        "wheelos_msgs.localization_msgs.localization_pb2",
        "LocalizationEstimate"
    ),

    # Perception
    "/apollo/perception/obstacles": (
        "wheelos_msgs.perception_msgs.perception_obstacle_pb2",
        "PerceptionObstacles"
    ),
    "/apollo/perception/traffic_light": (
        "wheelos_msgs.perception_msgs.traffic_light_detection_pb2",
        "TrafficLightDetection"
    ),

    # Dreamview
    "/apollo/dreamview": (
        "wheelos_msgs.dreamview_msgs.chart_pb2",
        "Chart"
    ),

    # Storytelling
    "/apollo/storytelling": (
        "wheelos_msgs.storytelling_msgs.storytelling_pb2",
        "Storytelling"
    ),
}

# Default message type (used when no topic is specified or for backward compatibility)
# ================= USER CONFIGURATION ====================
# Please replace the import below with your own Protobuf message type.
# For example:
# from your_package.your_proto_pb2 import YourMessage as MessageType
#
# This affects the generated template and the message type that will be published.
# Make sure it matches the actual message type you want to use.

# from wheelos_msgs.planning_msgs.planning_pb2 import ADCTrajectory as MessageType

from wheelos_msgs.prediction_msgs.prediction_obstacle_pb2 import (
    PredictionObstacles as MessageType, )
# from wheelos_msgs.routing_msgs.routing_pb2 import (
#     RoutingRequest as MessageType,
# )
# from wheelos_msgs.planning_msgs.planning_pb2 import ADCTrajectory as MessageType

# ========================================================

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")


def import_message_type(import_path: str, class_name: str) -> Type[Message]:
    """
    Dynamically import a message type by import path and class name.

    Args:
        import_path: Module import path (e.g., "wheelos_msgs.planning_msgs.planning_pb2")
        class_name: Name of the message class (e.g., "ADCTrajectory")

    Returns:
        The imported message type class
    """
    try:
        # Dynamically import the module
        parts = import_path.split('.')
        module = __import__(import_path)

        # Navigate to the actual module
        for part in parts[1:]:
            module = getattr(module, part)

        # Get the message class
        message_type = getattr(module, class_name)
        return message_type
    except (ImportError, AttributeError) as e:
        logging.error(
            f"Failed to import message type '{class_name}' from '{import_path}': {e}"
        )
        raise


def fill_header(msg: Message):
    """
    Fill the header fields of the message with default values.
    This is a placeholder function and should be customized based on your message type.
    """
    if hasattr(msg, "header"):
        msg.header.timestamp_sec = time.time()
        msg.header.sequence_num += 1


class ProtoTemplateGenerator:

    def __init__(self, message_type: Type[Message]):
        self.message_type = message_type

    def generate_template(self, output_filepath: str):
        try:
            msg_instance = self.message_type()
            logging.info(
                f"Starting template generation for: {self.message_type.DESCRIPTOR.full_name}"
            )
            # Start recursive fill with the full name of the root message as path
            self._fill_template_recursive(
                msg_instance, path=self.message_type.DESCRIPTOR.full_name)

            template_str = text_format.MessageToString(msg_instance,
                                                       as_utf8=True,
                                                       indent=0,
                                                       as_one_line=False)

            preamble = f"""# Protobuf Text Format Template for message: {self.message_type.DESCRIPTOR.full_name}
#
# Instructions:
# 1. Fill in the values for each field.
# 2. For repeated fields, add multiple entries by repeating the field name.
# 3. For enum fields, use defined enum names.
# 4. Remove comments (#) and placeholder values before publishing.
# 5. Fields not specified will use their default Protobuf values.
#
"""

            full_template = preamble + template_str

            with open(output_filepath, "w", encoding="utf-8") as f:
                f.write(full_template)

            logging.info(
                f"Successfully generated Text Format template "
                f"for '{self.message_type.DESCRIPTOR.full_name}' at '{output_filepath}'"
            )
        except Exception:
            logging.exception("Failed to generate template")
            raise

    def _fill_template_recursive(self, msg_instance: Message, path: str = ""):
        from google.protobuf import descriptor

        logging.debug(
            f"[{path}] Entering _fill_template_recursive for {msg_instance.DESCRIPTOR.full_name}"
        )

        for field in msg_instance.DESCRIPTOR.fields:
            current_path = f"{path}.{field.name}"

            # Use TypeName and LabelName static methods for robustness across protobuf versions.
            # If these still fail, fallback to direct integer values in logs.
            try:
                field_type_str = descriptor.FieldDescriptor.TypeName(
                    field.type)
                field_label_str = descriptor.FieldDescriptor.LabelName(
                    field.label)
            except AttributeError:
                # Fallback if TypeName/LabelName static methods are not available in a specific protobuf version
                field_type_str = f"TYPE_ID:{field.type}"
                field_label_str = f"LABEL_ID:{field.label}"

            # --- Map field detection and handling ---
            # A map field is internally represented as a 'repeated message' with a special option `map_entry`.
            # We must handle it before the general LABEL_REPEATED check.
            if (field.type == descriptor.FieldDescriptor.TYPE_MESSAGE
                    and field.message_type.has_options
                    and field.message_type.GetOptions().map_entry):

                logging.debug(
                    f"[{current_path}] Detected as MAP field (type: {field_type_str}, label: {field_label_str})"
                )

                # Get the map object from the message instance
                map_field = getattr(msg_instance, field.name)
                # Get the descriptors for key and value types of the map
                key_field_desc = field.message_type.fields_by_name["key"]
                value_field_desc = field.message_type.fields_by_name["value"]

                # Add two example map entries for the template
                for i in range(2):
                    key_placeholder = self._get_placeholder_for_primitive(
                        key_field_desc.type)
                    # For string keys, make them unique for the template
                    if key_field_desc.type == descriptor.FieldDescriptor.TYPE_STRING:
                        key_placeholder = f"PLACEHOLDER_KEY_{i}"

                    if value_field_desc.type == descriptor.FieldDescriptor.TYPE_MESSAGE:
                        # For map values that are messages, we must get the message object
                        # from the map first (which creates it if not present), then fill its fields.
                        nested_value_instance_in_map = map_field[
                            key_placeholder]
                        logging.debug(
                            f"[{current_path}[{key_placeholder}]] Recursing into map value message: {nested_value_instance_in_map.DESCRIPTOR.full_name}"
                        )
                        self._fill_template_recursive(
                            nested_value_instance_in_map,
                            path=f"{current_path}[{key_placeholder}]",
                        )
                        # No direct assignment needed here; nested_value_instance_in_map is already a reference to the object within the map.
                    elif value_field_desc.type == descriptor.FieldDescriptor.TYPE_ENUM:
                        enum_desc = value_field_desc.enum_type
                        first_enum_value = (enum_desc.values[0].number
                                            if enum_desc.values else 0)
                        map_field[key_placeholder] = (
                            first_enum_value  # Use dictionary assignment for map primitive/enum values
                        )
                        logging.debug(
                            f"[{current_path}[{key_placeholder}]] Set map enum value: {first_enum_value}"
                        )
                    else:  # Primitive value type for map
                        placeholder_val = self._get_placeholder_for_primitive(
                            value_field_desc.type)
                        map_field[key_placeholder] = (
                            placeholder_val  # Use dictionary assignment for map primitive values
                        )
                        logging.debug(
                            f"[{current_path}[{key_placeholder}]] Set map primitive value: {placeholder_val}"
                        )

            elif field.label == descriptor.FieldDescriptor.LABEL_REPEATED:
                logging.debug(
                    f"[{current_path}] Detected as REPEATED field (type: {field_type_str}, label: {field_label_str})"
                )

                # Get the actual list-like object for the repeated field
                target_container = getattr(msg_instance, field.name)

                # If a field is actually a map but was not caught by the `map_entry` check above
                # (e.g., due to an unexpected Protobuf version behavior or descriptor structure),
                # it will be a MessageMap object, which does NOT have 'append'.
                if not hasattr(target_container, "append"):
                    logging.critical(
                        f"[{current_path}] CRITICAL ERROR: Field '{field.name}' "
                        f"(in message {msg_instance.DESCRIPTOR.full_name}, Protobuf Type: {field_type_str}, Label: {field_label_str}) "
                        f"was identified as a REPEATED field, but its instance of type '{type(target_container).__name__}' "
                        f"has no 'append' method. This most likely indicates that it is actually a MAP field "
                        f"that was not correctly identified by the `map_entry` check. "
                        f"Please check your Protobuf definition for field '{field.name}'."
                    )
                    # Re-raise the error to keep the original traceback, but with our helpful log above.
                    raise AttributeError(
                        f"'{type(target_container).__name__}' object has no attribute 'append' for field '{current_path}'"
                    )

                if field.type == descriptor.FieldDescriptor.TYPE_MESSAGE:
                    for _ in range(2):  # Add two repeated message instances
                        nested_msg = (
                            target_container.add()
                        )  # Use .add() for repeated messages instead of _concrete_class() then append
                        logging.debug(
                            f"[{current_path}] Recursing into repeated message instance: {nested_msg.DESCRIPTOR.full_name}"
                        )
                        self._fill_template_recursive(nested_msg,
                                                      path=f"{current_path}[]")
                elif field.type == descriptor.FieldDescriptor.TYPE_ENUM:
                    enum_desc = field.enum_type
                    first_enum_value = (enum_desc.values[0].number
                                        if enum_desc.values else 0)
                    second_enum_value = (enum_desc.values[1].number if len(
                        enum_desc.values) > 1 else first_enum_value)
                    target_container.append(
                        first_enum_value)  # append for repeated enums
                    target_container.append(second_enum_value)
                    logging.debug(
                        f"[{current_path}] Appended repeated enum values: {first_enum_value}, {second_enum_value}"
                    )
                else:  # Repeated primitive types
                    for _ in range(2):
                        placeholder_val = self._get_placeholder_for_primitive(
                            field.type)
                        target_container.append(
                            placeholder_val)  # append for repeated primitives
                        logging.debug(
                            f"[{current_path}] Appended repeated primitive value: {placeholder_val}"
                        )

            # --- Single field handling ---
            elif field.type == descriptor.FieldDescriptor.TYPE_MESSAGE:
                logging.debug(
                    f"[{current_path}] Detected as SINGLE MESSAGE field (type: {field_type_str}, label: {field_label_str})"
                )
                nested_instance = getattr(msg_instance, field.name)
                logging.debug(
                    f"[{current_path}] Recursing into single message instance: {nested_instance.DESCRIPTOR.full_name}"
                )
                self._fill_template_recursive(nested_instance,
                                              path=current_path)
            elif field.type == descriptor.FieldDescriptor.TYPE_ENUM:
                enum_desc = field.enum_type
                first_enum_value = enum_desc.values[
                    0].number if enum_desc.values else 0
                setattr(msg_instance, field.name, first_enum_value)
                logging.debug(
                    f"[{current_path}] Set enum value: {first_enum_value}")
            else:  # Single primitive types
                placeholder_val = self._get_placeholder_for_primitive(
                    field.type)
                setattr(
                    msg_instance,
                    field.name,
                    placeholder_val,
                )
                logging.debug(
                    f"[{current_path}] Set primitive value: {placeholder_val}")

    def _get_placeholder_for_primitive(self, field_type: int) -> Any:
        from google.protobuf import descriptor

        if field_type == descriptor.FieldDescriptor.TYPE_STRING:
            return "PLACEHOLDER_STRING"
        elif field_type == descriptor.FieldDescriptor.TYPE_BYTES:
            return b""
        elif field_type in (
                descriptor.FieldDescriptor.TYPE_INT32,
                descriptor.FieldDescriptor.TYPE_INT64,
                descriptor.FieldDescriptor.TYPE_UINT32,
                descriptor.FieldDescriptor.TYPE_UINT64,
        ):
            return 0
        elif field_type in (
                descriptor.FieldDescriptor.TYPE_FLOAT,
                descriptor.FieldDescriptor.TYPE_DOUBLE,
        ):
            return 0.0
        elif field_type == descriptor.FieldDescriptor.TYPE_BOOL:
            return False
        else:
            return None


class ProtoMessagePublisher:

    def __init__(self, message_type: Type[Message]):
        self.message_type = message_type
        # Generate unique instance ID for this publisher
        self.instance_id = uuid.uuid4().hex[:8]
        self.node_name = f"proto_mock_publisher_{self.instance_id}"

    def load_message_from_text_file(self, filepath: str) -> Optional[Message]:
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                text_content = f.read()

            msg_instance = self.message_type()
            text_format.Parse(text_content, msg_instance)
            logging.info(
                f"[{self.instance_id}] Loaded message from {filepath} for type {msg_instance.DESCRIPTOR.full_name}"
            )
            return msg_instance
        except FileNotFoundError:
            logging.error(f"[{self.instance_id}] File not found: {filepath}")
            return None
        except text_format.ParseError as e:
            logging.error(f"[{self.instance_id}] Error parsing Text Format file {filepath}: {e}")
            return None
        except Exception:
            logging.exception(f"[{self.instance_id}] Unexpected error loading message")
            return None

    def publish_message(
        self,
        filepath: str,
        topic_name: str,
        period: float = 0.1,
        step_by_step: bool = False,
    ):
        if not topic_name:
            raise ValueError("Topic name is required for publishing.")

        msg_to_publish = self.load_message_from_text_file(filepath)
        if not msg_to_publish:
            raise ValueError("Invalid message data. Cannot publish.")

        try:
            cyber.init()
            node = cyber.Node(self.node_name)
            writer = node.create_writer(topic_name, self.message_type)

            logging.info(
                f"[{self.instance_id}] Node '{self.node_name}' created"
            )
            logging.info(
                f"[{self.instance_id}] Ready to publish '{msg_to_publish.DESCRIPTOR.full_name}' "
                f"to topic '{topic_name}' with "
                f"{'step-by-step mode' if step_by_step else f'period {period}s'}."
            )

            if step_by_step:
                logging.info(
                    f"[{self.instance_id}] Press Enter to publish one message, Ctrl+C to quit.")
                while not cyber.is_shutdown():
                    logging.info(f"[{self.instance_id}] Waiting for Enter key to publish...")
                    i, _, _ = select.select([sys.stdin], [], [], None)
                    if i:
                        input_line = sys.stdin.readline()
                        fill_header(msg_to_publish)
                        writer.write(msg_to_publish)
                        logging.info(
                            f"[{self.instance_id}] Topic '{topic_name}' message published.")
            else:
                while not cyber.is_shutdown():
                    fill_header(msg_to_publish)
                    writer.write(msg_to_publish)
                    if period > 0:
                        time.sleep(period)

        except KeyboardInterrupt:
            logging.info(f"[{self.instance_id}] Publishing stopped by user.")
        except Exception:
            logging.exception(f"[{self.instance_id}] Error during publishing")
            raise
        finally:
            cyber.shutdown()


def main():
    """
    Usage:
      1. Use --topic to specify a channel. The message type will be looked up from
         the predefined mapping. Use --list-topics to see all available channels.

      2. Use --msg-type to override the message type. Format: "import_path:ClassName"
         Example: --msg-type "wheelos_msgs.planning_msgs.planning_pb2:ADCTrajectory"

      3. For backward compatibility, you can still modify the # USER CONFIGURATION
         section to set the default MessageType.

    Examples:
      # Use predefined channel (looks up message type automatically)
      $ python publisher.py --gen -t /apollo/planning -o planning_template.txt
      $ python publisher.py --publish -i planning_template.txt -t /apollo/planning -p 0.1

      # Use custom message type for a channel
      $ python publisher.py --gen -t /custom/channel --msg-type "wheelos_msgs.routing_msgs.routing_pb2:RoutingRequest"

      # List all available predefined channels
      $ python publisher.py --list-topics
    """
    parser = argparse.ArgumentParser(
        description=(
            "Generate Text Format template or publish messages via Cyber RT.\n"
            "Message type can be specified via --msg-type or looked up from --topic."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    action_group = parser.add_mutually_exclusive_group(required=True)
    action_group.add_argument(
        "--gen",
        "-g",
        action="store_true",
        help="Generate a Text Format template for the message type.",
    )
    action_group.add_argument(
        "--publish",
        action="store_true",
        help="Publish messages from a Text Format file.",
    )
    action_group.add_argument(
        "--list-topics",
        "-l",
        action="store_true",
        help="List all predefined channel to message type mappings.",
    )

    parser.add_argument(
        "--input",
        "-i",
        type=str,
        help="Path to the Text Format file containing the message (for --publish).",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        help="Output file path for the generated template (for --gen).",
    )
    parser.add_argument(
        "--topic",
        "-t",
        type=str,
        help="The Cyber RT topic/channel name.",
    )
    parser.add_argument(
        "--msg-type",
        "-m",
        type=str,
        metavar="IMPORT_PATH:CLASS_NAME",
        help=(
            "Message type in format 'import_path:ClassName'. "
            "If not specified, will be looked up from --topic if provided. "
            "Example: 'wheelos_msgs.planning_msgs.planning_pb2:ADCTrajectory'"
        ),
    )
    parser.add_argument(
        "--period",
        "-p",
        type=float,
        default=0.1,
        help=
        "Publishing period in seconds. Set <= 0 for step-by-step mode (for --publish).",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Set the logging level.",
    )

    args = parser.parse_args()
    # Set log level dynamically based on argument
    logging.getLogger().setLevel(getattr(logging, args.log_level.upper()))

    # Handle --list-topics
    if args.list_topics:
        print("Available channel to message type mappings:")
        print("-" * 80)
        for channel, (import_path, class_name) in sorted(CHANNEL_MESSAGE_TYPE_MAP.items()):
            print(f"  {channel:50s} -> {class_name}")
            print(f"  {'':50s}    ({import_path})")
        print("-" * 80)
        print(f"Total: {len(CHANNEL_MESSAGE_TYPE_MAP)} channels")
        return

    # Determine the message type to use
    message_type: Type[Message] = MessageType  # Default to configured type
    output_filename = None

    if args.msg_type:
        # User specified explicit message type
        try:
            import_path, class_name = args.msg_type.split(":", 1)
            message_type = import_message_type(import_path, class_name)
            logging.info(f"Using specified message type: {import_path}.{class_name}")
        except ValueError:
            parser.error(
                f"--msg-type must be in format 'import_path:ClassName', got: {args.msg_type}"
            )
        except Exception as e:
            parser.error(f"Failed to import message type: {e}")

        output_filename = f"{class_name}_template.txt"

    elif args.topic and args.topic in CHANNEL_MESSAGE_TYPE_MAP:
        # Look up message type from channel mapping
        import_path, class_name = CHANNEL_MESSAGE_TYPE_MAP[args.topic]
        message_type = import_message_type(import_path, class_name)
        logging.info(
            f"Looked up message type for channel '{args.topic}': {class_name}"
        )
        output_filename = f"{class_name}_template.txt"

    elif args.topic:
        # Channel specified but not in mapping, use default MessageType
        logging.warning(
            f"Channel '{args.topic}' not found in predefined mappings. "
            f"Using default MessageType: {MessageType.DESCRIPTOR.full_name}"
        )
        output_filename = f"{MessageType.DESCRIPTOR.name}_template.txt"
    else:
        # No channel specified, use default MessageType
        logging.info(
            f"No channel specified, using default MessageType: {MessageType.DESCRIPTOR.full_name}"
        )
        output_filename = f"{MessageType.DESCRIPTOR.name}_template.txt"

    # Set default input/output filenames if not specified
    if args.gen and not args.output:
        args.output = output_filename
    if args.publish and not args.input:
        args.input = output_filename
        logging.info(f"No input file specified, using default: {args.input}")

    if args.publish and not args.topic:
        parser.error("--publish requires --topic argument.")

    try:
        if args.gen:
            generator = ProtoTemplateGenerator(message_type)
            generator.generate_template(args.output)
        elif args.publish:
            step_mode = args.period <= 0.0
            publisher = ProtoMessagePublisher(message_type)
            publisher.publish_message(
                filepath=args.input,
                topic_name=args.topic,
                period=args.period,
                step_by_step=step_mode,
            )
    except Exception as e:
        logging.error(f"An error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
