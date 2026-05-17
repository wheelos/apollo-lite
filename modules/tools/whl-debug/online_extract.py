#!/usr/bin/env python3
"""
Online Data Extractor for Apollo Cyber RT

Usage:
    python online_extract.py -c /apollo/canbus/chassis -f speed_ms -n 10
    python online_extract.py -c /apollo/planning -f decision
    python online_extract.py -c /apollo/localization/pose -f pose.position.x pose.position.y
    python online_extract.py -c /apollo/canbus/chassis -c /apollo/planning -f speed_ms -f decision
    python online_extract.py -c /apollo/canbus/chassis -c /apollo/planning  # output full message for both
"""

import sys
import logging
import json
import uuid
from typing import Optional, List, Any, Tuple
import click
from google.protobuf.json_format import MessageToDict
from google.protobuf.message import Message

from cyber.python.cyber_py3 import cyber

# Import common protobuf message types
from modules.common_msgs.chassis_msgs.chassis_pb2 import Chassis
from modules.common_msgs.chassis_msgs.chassis_detail_pb2 import ChassisDetail
from modules.common_msgs.control_msgs.control_cmd_pb2 import ControlCommand
from modules.common_msgs.dreamview_msgs.hmi_status_pb2 import HMIStatus
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles
from modules.common_msgs.prediction_msgs.prediction_obstacle_pb2 import PredictionObstacles
from modules.common_msgs.routing_msgs.routing_pb2 import (RoutingRequest,
                                                          RoutingResponse)

# Channel to message type mapping
CHANNEL_MESSAGE_TYPE_MAP = {
    "/apollo/canbus/chassis": Chassis,
    "/apollo/canbus/chassis_detail": ChassisDetail,
    "/apollo/localization/pose": LocalizationEstimate,
    "/apollo/planning": ADCTrajectory,
    "/apollo/hmi/status": HMIStatus,
    "/apollo/control": ControlCommand,
    "/apollo/prediction": PredictionObstacles,
    "/apollo/perception/obstacles": PerceptionObstacles,
    "/apollo/routing_request": RoutingRequest,
    "/apollo/routing_response": RoutingResponse,
}

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


def get_nested_field(msg: Message, field_path: str) -> Optional[Any]:
    """
    Get a nested field from a protobuf message using dot notation.

    Args:
        msg: The protobuf message
        field_path: Dot-separated field path (e.g., "position.x", "header.sequence_num")

    Returns:
        The field value or None if not found
    """
    data = MessageToDict(msg,
                         preserving_proto_field_name=True,
                         use_integers_for_enums=True)
    parts = field_path.split(".")
    for part in parts:
        if isinstance(data, dict) and part in data:
            data = data[part]
        else:
            return None
    return data


def format_value(value: Any) -> str:
    """
    Format a value for printing.

    Args:
        value: The value to format

    Returns:
        Formatted string representation
    """
    if isinstance(value, (list, dict)):
        return json.dumps(value, separators=(",", ":"))
    return str(value)


class ChannelExtractor:
    """Extractor for a single channel."""

    def __init__(
        self,
        channel: str,
        fields: List[str],
        output_format: str = "text",
        separator: str = ",",
        porcelain: bool = False,
        all_csv_fields: Optional[List[str]] = None,
        msg_type_name: str = "Unknown",
    ):
        """
        Initialize the channel extractor.

        Args:
            channel: Cyber RT channel name to subscribe
            fields: List of field paths to extract (dot notation for nested fields)
            output_format: Output format ('text', 'csv', or 'json')
            separator: Separator for CSV output
            porcelain: If True, suppress all log output (only data is printed)
            all_csv_fields: All fields to use in CSV header (for unified output)
            msg_type_name: Message type name for this channel
        """
        self.channel = channel
        self.fields = fields
        self.output_format = output_format
        self.separator = separator
        self.porcelain = porcelain
        self.all_csv_fields = all_csv_fields
        self.msg_type_name = msg_type_name
        self.message_count = 0
        self.reader = None

    def _log(self, msg: str):
        """Log message to stderr if not in porcelain mode."""
        if not self.porcelain:
            click.echo(msg, err=True)

    def callback(self, data):
        """
        Callback function for received messages.

        Args:
            data: The received protobuf message
        """
        self.message_count += 1

        # Get timestamp
        timestamp = getattr(data, "header", None)
        ts_value = (timestamp.timestamp_sec
                    if timestamp and hasattr(timestamp, "timestamp_sec") else
                    self.message_count)

        # Convert message to dict once
        msg_dict = MessageToDict(data,
                                 preserving_proto_field_name=True,
                                 use_integers_for_enums=True)

        # Output based on format
        if self.output_format == "json":
            if self._should_output_root():
                json_obj = {
                    "channel": self.channel,
                    "ts": ts_value,
                    "data": msg_dict
                }
            else:
                json_obj = {"channel": self.channel, "ts": ts_value}
                for field_path in self.fields:
                    value = get_nested_field(data, field_path)
                    json_obj[field_path] = value
            click.echo(json.dumps(json_obj, separators=(",", ":")))
        elif self.output_format == "csv":
            out_values = [self.channel, str(ts_value)]
            # Build a mapping from display name to original field path
            field_map = {}
            for f in self.fields:
                if f in ("", "."):
                    field_map[
                        self.msg_type_name] = f  # Map type name to '.' or ''
                else:
                    field_map[f] = f
            # Use unified CSV fields if available, otherwise use own fields
            fields_to_output = (self.all_csv_fields if self.all_csv_fields else
                                list(field_map.keys()))
            for display_name in fields_to_output:
                if display_name in field_map:
                    original_field = field_map[display_name]
                    if original_field in ("", "."):
                        # Output full message for '.' or ''
                        out_values.append(
                            json.dumps(msg_dict, separators=(",", ":")))
                    else:
                        # Get nested field value
                        value = get_nested_field(data, original_field)
                        if value is None:
                            out_values.append("")
                        elif isinstance(value, (list, dict)):
                            out_values.append(
                                json.dumps(value, separators=(",", ":")))
                        else:
                            out_values.append(str(value))
                else:
                    # This field belongs to another channel, output empty
                    out_values.append("")
            click.echo(self.separator.join(out_values))
        else:  # text format
            if self._should_output_root():
                click.echo(f"[{self.channel}][{ts_value}] " +
                           json.dumps(msg_dict, separators=(",", ":")))
            else:
                formatted_values = []
                for field_path in self.fields:
                    value = get_nested_field(data, field_path)
                    formatted_values.append(
                        format_value(value) if value is not None else "N/A")
                click.echo(f"[{self.channel}][{ts_value}] " +
                           " | ".join(formatted_values))

    def _should_output_root(self) -> bool:
        """Check if we should output the entire message root."""
        return all(f in ("", ".") for f in self.fields)

    def get_display_fields(self) -> str:
        """Get display string for fields."""
        if self._should_output_root():
            return "(full message)"
        return ", ".join(self.fields)


class OnlineExtractor:
    """Online data extractor for multiple Cyber RT channels."""

    def __init__(
        self,
        channel_configs: List[Tuple[str, List[str]]],
        count: int = -1,
        output_format: str = "text",
        separator: str = ",",
        porcelain: bool = False,
    ):
        """
        Initialize the online extractor.

        Args:
            channel_configs: List of (channel, fields) tuples
            count: Number of messages to process (-1 for infinite)
            output_format: Output format ('text', 'csv', or 'json')
            separator: Separator for CSV output
            porcelain: If True, suppress all log output (only data is printed)
        """
        self.channel_configs = channel_configs
        self.target_count = count
        self.output_format = output_format
        self.separator = separator
        self.porcelain = porcelain
        self.total_processed = 0
        self.should_stop = False
        self.node = None
        self.extractors: List[ChannelExtractor] = []

        # Generate unique instance ID for this extractor
        self.instance_id = uuid.uuid4().hex[:8]
        self.node_name = f"online_extractor_{self.instance_id}"

        # Calculate all unique fields for unified CSV header
        self.all_csv_fields = self._calculate_all_fields()

    def _calculate_all_fields(self) -> Optional[List[str]]:
        """Calculate all unique fields from all channels for CSV."""
        if self.output_format != "csv":
            return None
        # Collect all unique fields
        # For '.' or '', use message type name as the field name
        all_fields = []
        for channel, fields in self.channel_configs:
            msg_type = CHANNEL_MESSAGE_TYPE_MAP.get(channel)
            type_name = msg_type.__name__ if msg_type else "Unknown"
            for f in fields:
                if f in ("", "."):
                    # Use message type name as the field name
                    field_name = type_name
                else:
                    field_name = f
                if field_name not in all_fields:
                    all_fields.append(field_name)
        return all_fields

    def _log(self, msg: str):
        """Log message to stderr if not in porcelain mode."""
        if not self.porcelain:
            click.echo(f"[{self.instance_id}] {msg}", err=True)

    def _print_header(self):
        """Print header for CSV format."""
        if self.output_format == "csv":
            header = ["channel", "ts"]
            if self.all_csv_fields:
                header.extend(self.all_csv_fields)
            click.echo(self.separator.join(header))

    def _on_message(self, channel_extractor: ChannelExtractor, data):
        """Wrapper callback for counting messages."""
        channel_extractor.callback(data)
        self.total_processed += 1

        # Check if we've processed enough messages
        if self.target_count > 0 and self.total_processed >= self.target_count:
            self._log(
                f"Processed {self.total_processed} messages. Stopping...")
            self.should_stop = True

    def start(self):
        """
        Start subscribing to all channels.

        Returns:
            True if successful, False otherwise
        """
        try:
            if not cyber.ok():
                cyber.init()

            self.node = cyber.Node(self.node_name)

            # Create extractors and readers for each channel
            for channel, fields in self.channel_configs:
                message_type = CHANNEL_MESSAGE_TYPE_MAP.get(channel)
                if message_type is None:
                    self._log(
                        f"Error: Channel '{channel}' is not in the predefined mapping. "
                        f"Supported channels: {', '.join(CHANNEL_MESSAGE_TYPE_MAP.keys())}"
                    )
                    return False

                msg_type_name = message_type.__name__
                extractor = ChannelExtractor(
                    channel,
                    fields,
                    self.output_format,
                    self.separator,
                    self.porcelain,
                    self.all_csv_fields,
                    msg_type_name,
                )

                # Create reader with wrapped callback
                self.node.create_reader(
                    channel,
                    message_type,
                    lambda data, e=extractor: self._on_message(e, data),
                )

                self.extractors.append(extractor)

            # Print header for CSV format
            self._print_header()

            # Log subscription info
            count_str = ("infinite" if self.target_count == -1 else str(
                self.target_count))
            self._log(
                f"Subscribed to {len(self.channel_configs)} channel(s) for {count_str} messages:"
            )
            for extractor in self.extractors:
                self._log(
                    f"  - {extractor.channel}: {extractor.get_display_fields()}"
                )
            self._log("Press Ctrl+C to stop...")

            # Spin loop
            import time as _time

            while not cyber.is_shutdown() and not self.should_stop:
                _time.sleep(0.002)
            return True

        except Exception as e:
            self._log(f"Error subscribing to channel: {e}")
            return False

    def stop(self):
        """Stop the extractor and cleanup resources."""
        if self.node:
            self.node = None
        if cyber.ok():
            cyber.shutdown()


@click.command()
@click.option(
    "-c",
    "--channel",
    "channels",
    multiple=True,
    help=
    "Cyber RT channel name to subscribe to (can be specified multiple times)",
)
@click.option(
    "-f",
    "--field",
    "fields",
    multiple=True,
    help=
    "Field paths to extract for the most recent -c (can be specified multiple times, "
    'use "." or "" for root level, default: ".")',
)
@click.option(
    "-n",
    "--count",
    default=-1,
    type=int,
    help="Number of messages to process (default: -1 for infinite)",
)
@click.option("--list-channels",
              is_flag=True,
              help="List all supported channels")
@click.option(
    "--output-format",
    type=click.Choice(["text", "csv", "json"]),
    default="text",
    help="Output format (default: text)",
)
@click.option("--separator",
              default=",",
              help='Separator for CSV output (default: ",")')
@click.option(
    "--porcelain",
    is_flag=True,
    help="Porcelain mode: suppress all log output, only print data",
)
@click.option(
    "--log-level",
    type=click.Choice(["DEBUG", "INFO", "WARNING", "ERROR"]),
    default="INFO",
    help="Set the logging level",
)
def main(
    channels,
    fields,
    count,
    list_channels,
    output_format,
    separator,
    porcelain,
    log_level,
):
    """
    Online data extractor for Apollo Cyber RT.

    Subscribe to channels and extract specified fields.

    \b
    Examples:
      # Extract single field from one channel
      python online_extract.py -c /apollo/canbus/chassis -f speed_ms

      # Extract multiple fields from one channel (CSV format)
      python online_extract.py -c /apollo/canbus/chassis -f speed_ms -f steering_percentage --output-format csv

      # Subscribe to multiple channels with different fields
      python online_extract.py -c /apollo/canbus/chassis -f speed_ms -c /apollo/planning -f decision

      # Output full message for multiple channels (no -f specified)
      python online_extract.py -c /apollo/canbus/chassis -c /apollo/planning

      # Mix: full message for one channel, specific fields for another
      python online_extract.py -c /apollo/canbus/chassis -c /apollo/planning -f decision

      # Use "." to explicitly output root level
      python online_extract.py -c /apollo/canbus/chassis -f .

      # List all supported channels
      python online_extract.py --list-channels
    """
    # Set log level (disable in porcelain mode)
    if not porcelain:
        logging.getLogger().setLevel(getattr(logging, log_level))
    else:
        logging.getLogger().setLevel(logging.CRITICAL + 1)

    # Handle list channels option
    if list_channels:
        click.echo(f"Supported channels ({len(CHANNEL_MESSAGE_TYPE_MAP)}):")
        for ch, msg_type in CHANNEL_MESSAGE_TYPE_MAP.items():
            click.echo(f"  {ch} -> {msg_type.__name__}")
        return 0

    # Check if channels is empty
    if not channels:
        click.echo("Error: Missing option '-c' / '--channel'.", err=True)
        ctx = click.get_current_context()
        click.echo(ctx.get_help())
        return 1

    # Parse channel and field pairings from sys.argv
    # Each -c starts a new channel, subsequent -f apply to the most recent -c
    channel_configs = []  # List of [channel, fields_list]
    channel_idx = 0
    field_idx = 0

    i = 1  # Skip script name
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg in ("-c", "--channel"):
            # New channel
            if channel_idx < len(channels):
                channel_configs.append([channels[channel_idx], []])
                channel_idx += 1
            # Skip channel value
            if i + 1 < len(sys.argv) and not sys.argv[i + 1].startswith("-"):
                i += 2
            else:
                i += 1
        elif arg in ("-f", "--field"):
            # Field for most recent channel
            if channel_configs and field_idx < len(fields):
                channel_configs[-1][1].append(fields[field_idx])
                field_idx += 1
            # Skip field value
            if i + 1 < len(sys.argv) and not sys.argv[i + 1].startswith("-"):
                i += 2
            else:
                i += 1
        else:
            i += 1

    # Convert to list of tuples
    # For empty fields, default to '.' (output full message)
    channel_configs_tuples = [(ch, flds if flds else ["."])
                              for ch, flds in channel_configs]

    # Create and start extractor
    extractor = OnlineExtractor(channel_configs_tuples, count, output_format,
                                separator, porcelain)
    try:
        extractor.start()
    except KeyboardInterrupt:
        if not porcelain:
            logger.info(
                f"\nStopped. Processed {extractor.total_processed} messages.")
        return 0
    except Exception as e:
        if not porcelain:
            logger.error(f"Error: {e}")
        return 1
    finally:
        extractor.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
