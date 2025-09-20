import sys
import select
import argparse
import time
import logging
from typing import Optional, Any, Type

from google.protobuf import text_format
from google.protobuf.message import Message

from cyber.python.cyber_py3 import cyber

# ================= USER CONFIGURATION ====================
# Please replace the import below with your own Protobuf message type.
# For example:
# from your_package.your_proto_pb2 import YourMessage as MessageType
#
# This affects the generated template and the message type that will be published.
# Make sure it matches the actual message type you want to use.

from cyber.proto.unit_test_pb2 import ChatterBenchmark as MessageType

# ========================================================


# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)


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
            self._fill_template_recursive(msg_instance)

            template_str = text_format.MessageToString(
                msg_instance, as_utf8=True, indent=0, as_one_line=False
            )

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

    def _fill_template_recursive(self, msg_instance: Message):
        from google.protobuf import descriptor

        for field in msg_instance.DESCRIPTOR.fields:
            if field.label == descriptor.FieldDescriptor.LABEL_REPEATED:
                if field.type == descriptor.FieldDescriptor.TYPE_MESSAGE:
                    for _ in range(2):
                        nested_msg = field.message_type._concrete_class()
                        self._fill_template_recursive(nested_msg)
                        getattr(msg_instance, field.name).append(nested_msg)
                elif field.type == descriptor.FieldDescriptor.TYPE_ENUM:
                    enum_desc = field.enum_type
                    first_enum_value = (
                        enum_desc.values[0].number if enum_desc.values else 0
                    )
                    second_enum_value = (
                        enum_desc.values[1].number
                        if len(enum_desc.values) > 1
                        else first_enum_value
                    )
                    getattr(msg_instance, field.name).append(first_enum_value)
                    getattr(msg_instance, field.name).append(second_enum_value)
                else:
                    for _ in range(2):
                        getattr(msg_instance, field.name).append(
                            self._get_placeholder_for_primitive(field.type)
                        )
            elif field.type == descriptor.FieldDescriptor.TYPE_MESSAGE:
                nested_instance = getattr(msg_instance, field.name)
                self._fill_template_recursive(nested_instance)
            elif field.type == descriptor.FieldDescriptor.TYPE_ENUM:
                enum_desc = field.enum_type
                first_enum_value = enum_desc.values[0].number if enum_desc.values else 0
                setattr(msg_instance, field.name, first_enum_value)
            else:
                setattr(
                    msg_instance,
                    field.name,
                    self._get_placeholder_for_primitive(field.type),
                )

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

    def load_message_from_text_file(self, filepath: str) -> Optional[Message]:
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                text_content = f.read()

            msg_instance = self.message_type()
            text_format.Parse(text_content, msg_instance)
            logging.info(
                f"Loaded message from {filepath} for type {msg_instance.DESCRIPTOR.full_name}"
            )
            return msg_instance
        except FileNotFoundError:
            logging.error(f"File not found: {filepath}")
            return None
        except text_format.ParseError as e:
            logging.error(f"Error parsing Text Format file {filepath}: {e}")
            return None
        except Exception:
            logging.exception("Unexpected error loading message")
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
            node = cyber.Node("proto_mock_publisher")
            writer = node.create_writer(topic_name, self.message_type)

            logging.info(
                f"Ready to publish '{msg_to_publish.DESCRIPTOR.full_name}' "
                f"to topic '{topic_name}' with "
                f"{'step-by-step mode' if step_by_step else f'period {period}s'}."
            )

            if step_by_step:
                logging.info("Press Enter to publish one message, Ctrl+C to quit.")
                while not cyber.is_shutdown():
                    logging.info("Waiting for Enter key to publish...")
                    i, _, _ = select.select([sys.stdin], [], [], None)
                    if i:
                        input_line = sys.stdin.readline()
                        fill_header(msg_to_publish)
                        writer.write(msg_to_publish)
                        logging.info(f"Topic '{topic_name}' message published.")
            else:
                while not cyber.is_shutdown():
                    fill_header(msg_to_publish)
                    writer.write(msg_to_publish)
                    if period > 0:
                        time.sleep(period)

        except KeyboardInterrupt:
            logging.info("Publishing stopped by user.")
        except Exception:
            logging.exception("Error during publishing")
            raise
        finally:
            cyber.shutdown()


def main():
    """
    Usage:
      Modify the # USER CONFIGURATION section in this file to replace `MessageType`
      with your own Protobuf message type.

      Then you can generate the corresponding Text Format template with --gen,
      or publish messages using --publish with the template file.

    Examples:
      $ python publisher.py --gen -o your_message_template.txt
      $ python publisher.py --publish -i your_message_template.txt -t /your_topic -p 0.1
    """
    parser = argparse.ArgumentParser(
        description=(
            "Utility for a specific Protobuf MessageType: "
            "Generate Text Format template or publish messages via Cyber RT."
        )
    )

    action_group = parser.add_mutually_exclusive_group(required=True)
    action_group.add_argument(
        "--gen",
        "-g",
        action="store_true",
        help="Generate a Text Format template for the configured MessageType.",
    )
    action_group.add_argument(
        "--publish",
        action="store_true",
        help="Publish messages from a Text Format file.",
    )

    parser.add_argument(
        "--input",
        "-i",
        type=str,
        default=f"{MessageType.DESCRIPTOR.name}_template.txt",
        help="Path to the Text Format file containing the message (required for --publish).",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default=f"{MessageType.DESCRIPTOR.name}_template.txt",
        help="Output file path for the generated template (used with --gen).",
    )
    parser.add_argument(
        "--topic",
        "-t",
        type=str,
        help="The Cyber RT topic name to publish to (required for --publish).",
    )
    parser.add_argument(
        "--period",
        "-p",
        type=float,
        default=0.1,
        help="Publishing period in seconds. Set <= 0 for step-by-step mode (used with --publish).",
    )

    args = parser.parse_args()

    if args.publish and not args.topic:
        parser.error("--publish requires --topic argument.")

    try:
        if args.gen:
            generator = ProtoTemplateGenerator(MessageType)
            generator.generate_template(args.output)
        elif args.publish:
            step_mode = args.period <= 0.0
            publisher = ProtoMessagePublisher(MessageType)
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
