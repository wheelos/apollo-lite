# online_extract.py - Design Documentation

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Component Design](#component-design)
3. [Data Flow](#data-flow)
4. [Algorithm Details](#algorithm-details)
5. [Configuration](#configuration)
6. [Extension Points](#extension-points)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        online_extract.py                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │   CLI       │───▶│    Online    │───▶│  Channel     │   │
│  │   Parser    │    │   Extractor  │    │  Extractors  │   │
│  └─────────────┘    └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            ▼                    ▼            │
│                     ┌──────────────┐    ┌──────────────┐   │
│                     │   Callback   │    │  Message     │   │
│                     │   Handler    │    │  Processor   │   │
│                     └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            └────────┬───────────┘            │
│                                     ▼                        │
│                            ┌──────────────┐                 │
│                            │   Output     │                 │
│                            │   Formatter  │                 │
│                            └──────────────┘                 │
└─────────────────────────────────────────────────────────────┘
                           │         │
                           ▼         ▼
                    ┌──────────┐ ┌──────────┐
                    │  Cyber  │ │  stdout/ │
                    │    RT   │ │   file   │
                    └──────────┘ └──────────┘
```

## Component Design

### 1. CLI Parser

**Responsibility**: Parse command-line arguments and build channel configurations.

**Key Logic**:
```python
# Parse sys.argv to build channel-to-fields mapping
for each -c option:
    create new channel entry with empty fields list
for each -f option:
    append to most recent channel's fields list
```

**Output**: `List[Tuple[str, List[str]]]` - list of (channel, fields) tuples

### 2. Online Extractor

**Responsibility**: Main controller that manages Cyber RT lifecycle and coordinates extractors.

**Key Attributes**:
- `channel_configs`: List of (channel, fields) tuples
- `all_csv_fields`: Unified field list for CSV header
- `extractors`: List of ChannelExtractor instances

**Key Methods**:
- `_calculate_all_fields()`: Compute unique field names across all channels
- `_print_header()`: Print CSV header if needed
- `start()`: Initialize Cyber RT and create readers
- `stop()`: Cleanup resources

### 3. Channel Extractor

**Responsibility**: Handle message processing for a single channel.

**Key Attributes**:
- `channel`: Channel name
- `fields`: List of field paths to extract
- `all_csv_fields`: Unified field list (for CSV alignment)
- `msg_type_name`: Message type name for display

**Key Methods**:
- `callback(data)`: Process received message
- `_should_output_root()`: Check if full message should be output
- `get_display_fields()`: Get human-readable field description

### 4. Message Processor

**Responsibility**: Extract nested fields from protobuf messages.

**Function**: `get_nested_field(msg, field_path)`

**Algorithm**:
```python
def get_nested_field(msg, field_path):
    data = MessageToDict(msg, preserving_proto_field_name=True)
    for part in field_path.split('.'):
        if part in data:
            data = data[part]
        else:
            return None
    return data
```

### 5. Output Formatter

**Responsibility**: Format extracted data according to output format.

**Formats**:
- **Text**: `[channel][ts] value1 | value2`
- **CSV**: `channel,ts,field1,field2`
- **JSON**: `{"channel": "...", "ts": ..., "field": ...}`

## Data Flow

```
User Input (CLI)
       │
       ▼
┌─────────────────┐
│  Parse Arguments│
│  Build Configs  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Create Extractor│
│ Create Readers  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Spin Loop      │◀─────────────────┐
│  (Poll RT)      │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ Message Received│                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ Find Extractor  │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ Extract Fields  │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ Format Output   │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│  Print to Stdout│                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ Check Count     │                  │
│ (If -n specified)│─────────────────┘
└────────┬────────┘
         │
         ▼
    [Continue/Stop]
```

## Algorithm Details

### Channel-Field Association Algorithm

The key design challenge is associating fields with the correct channel when multiple `-c` and `-f` options are used.

**Algorithm**:
```python
def parse_channel_field_config(channels, fields, sys_argv):
    configs = []  # [(channel, [fields])]
    channel_idx = 0
    field_idx = 0

    i = 1  # Skip script name
    while i < len(sys_argv):
        arg = sys_argv[i]
        if arg in ("-c", "--channel"):
            # New channel
            if channel_idx < len(channels):
                configs.append([channels[channel_idx], []])
                channel_idx += 1
            i += 2  # Skip option and value
        elif arg in ("-f", "--field"):
            # Field for most recent channel
            if configs and field_idx < len(fields):
                configs[-1][1].append(fields[field_idx])
                field_idx += 1
            i += 2  # Skip option and value
        else:
            i += 1

    # Default to '.' for empty fields
    return [(ch, flds if flds else ["."]) for ch, flds in configs]
```

### CSV Header Unification Algorithm

For CSV output, all channels must share the same header with all fields from all channels.

**Algorithm**:
```python
def calculate_all_fields(channel_configs):
    all_fields = []
    for channel, fields in channel_configs:
        msg_type = CHANNEL_MESSAGE_TYPE_MAP.get(channel)
        type_name = msg_type.__name__ if msg_type else "Unknown"

        for f in fields:
            if f in ("", "."):
                # Use message type name for root-level output
                field_name = type_name
            else:
                field_name = f

            if field_name not in all_fields:
                all_fields.append(field_name)

    return all_fields
```

### Field Value Mapping Algorithm

When outputting CSV, each channel must map its fields to the unified header.

**Algorithm**:
```python
def map_fields_to_header(fields, msg_type_name, all_csv_fields):
    # Build display name to original field path mapping
    field_map = {}
    for f in fields:
        if f in ("", "."):
            field_map[msg_type_name] = f
        else:
            field_map[f] = f

    # Output values in header order
    out_values = []
    for display_name in all_csv_fields:
        if display_name in field_map:
            original_field = field_map[display_name]
            if original_field in ("", "."):
                # Output full message as JSON
                out_values.append(json.dumps(msg_dict))
            else:
                # Extract nested field
                value = get_nested_field(msg, original_field)
                out_values.append(format_value(value))
        else:
            # Field belongs to another channel
            out_values.append("")

    return out_values
```

## Configuration

### Channel-Message Type Mapping

```python
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
```

### Default Values

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output_format` | `text` | Output format |
| `separator` | `,` | CSV separator |
| `count` | `-1` | Process all messages |
| `log_level` | `INFO` | Logging level |

## Extension Points

### Adding New Channels

To add support for a new channel:

1. Import the message type:
```python
from modules.common_msgs.xxx_msgs/xxx_pb2 import XxxMessage
```

2. Add to `CHANNEL_MESSAGE_TYPE_MAP`:
```python
CHANNEL_MESSAGE_TYPE_MAP = {
    ...
    "/apollo/new/channel": XxxMessage,
}
```

### Adding New Output Formats

1. Add format to `--output-format` choices
2. Implement formatting logic in `ChannelExtractor.callback()`

### Adding Custom Field Processors

Extend `ChannelExtractor` to add custom field processing logic:

```python
class CustomChannelExtractor(ChannelExtractor):
    def process_field(self, value, field_path):
        # Custom processing logic
        return transformed_value
```

## Performance Considerations

1. **Message Conversion**: `MessageToDict` is called for each message. For high-frequency channels, consider caching.
2. **Field Extraction**: Nested field access walks the dict each time. Flat fields are faster.
3. **Output Formatting**: JSON is slower than text/CSV due to serialization.

## Threading Model

- Single-threaded design
- Cyber RT callbacks execute in Cyber RT's thread pool
- No shared mutable state between extractors
- Thread-safe output via `click.echo()`
