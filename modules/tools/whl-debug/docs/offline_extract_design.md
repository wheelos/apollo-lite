# offline_extract.py - Design Documentation

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Component Design](#component-design)
3. [Data Flow](#data-flow)
4. [Algorithm Details](#algorithm-details)
5. [Record File Format](#record-file-format)
6. [Performance Considerations](#performance-considerations)

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        offline_extract.py                    │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │   CLI       │───▶│    Offline   │───▶│  Channel     │   │
│  │   Parser    │    │   Extractor  │    │  Extractors  │   │
│  └─────────────┘    └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            ▼                    ▼            │
│                     ┌──────────────┐    ┌──────────────┐   │
│                     │  File Scanner│    │  Message     │   │
│                     │              │    │  Processor   │   │
│                     └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            ▼                    ▼            │
│                     ┌──────────────┐    ┌──────────────┐   │
│                     │   Record     │    │  Field       │   │
│                     │   Reader     │    │  Extractor   │   │
│                     └──────────────┘    └──────────────┘   │
│                            │                    │            │
│                            └────────┬───────────┘            │
│                                     ▼                        │
│                            ┌──────────────┐                 │
│                            │   Output     │                 │
│                            │   Formatter  │                 │
│                            └──────────────┘                 │
└─────────────────────────────────────────────────────────────┘
                           │
                           ▼
                    ┌──────────┐
                    │  stdout/ │
                    │   file   │
                    └──────────┘
```

## Component Design

### 1. CLI Parser

**Responsibility**: Parse command-line arguments and build channel configurations.

**Key Logic**: Same as `online_extract.py` - parses sys.argv to build channel-to-fields mapping.

**Output**: `List[Tuple[str, List[str]]]` - list of (channel, fields) tuples

### 2. Offline Extractor

**Responsibility**: Main controller that manages file processing and coordinates extractors.

**Key Attributes**:
- `input_dir`: Directory containing record files
- `channel_configs`: List of (channel, fields) tuples
- `all_csv_fields`: Unified field list for CSV header
- `extractors`: List of ChannelExtractor instances

**Key Methods**:
- `_get_record_files()`: Scan directory for record files
- `_calculate_all_fields()`: Compute unique field names across all channels
- `_print_header()`: Print CSV header if needed
- `start()`: Process all record files

### 3. Record Reader Wrapper

**Responsibility**: Read messages from Apollo Cyber RT record files.

**Key Attributes**:
- Uses `cyber.python.cyber_py3.record.RecordReader`
- Iterates through messages using `read_messages()`

**Message Format**:
```python
# Each message from RecordReader.read_messages() returns:
PyBagMessage(
    topic=str,        # Channel name
    message=bytes,    # Raw protobuf bytes
    data_type=str,    # Message type name
    timestamp=int     # Nanosecond timestamp
)
```

### 4. Message Parser

**Responsibility**: Parse raw message bytes using the correct message type.

**Key Logic**:
```python
def parse_message(raw_bytes, msg_type):
    msg = msg_type()
    msg.ParseFromString(raw_bytes)
    return msg
```

### 5. Channel Extractor

**Responsibility**: Handle message processing for a single channel.

Same as `online_extract.py`, with additional message parsing capability.

**Key Differences from online_extract.py**:
- Receives raw bytes instead of parsed message
- Must parse message using `MESSAGE_TYPE_MAP`

## Data Flow

```
User Input (CLI + -i directory)
       │
       ▼
┌─────────────────┐
│  Parse Arguments│
│  Scan Directory │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Create Extractor│
│ Get Record Files│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ For Each File   │◀─────────────────┐
│  Create Reader  │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ Read Message    │                  │
│ (RecordReader)  │                  │
└────────┬────────┘                  │
         │                           │
         ▼                           │
┌─────────────────┐                  │
│ Parse Raw Bytes │                  │
│ (msg.ParseFrom) │                  │
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
    [Next Message/Next File/Done]
```

## Algorithm Details

### File Scanning Algorithm

```python
def get_record_files(input_dir):
    files = [
        os.path.join(input_dir, x)
        for x in os.listdir(input_dir)
        if os.path.isfile(os.path.join(input_dir, x))
    ]
    files.sort()  # Alphabetical order
    return files
```

### Message Processing Loop

```python
def process_files(record_files, channel_extractor_map):
    for file in record_files:
        reader = RecordReader(file)
        available_channels = reader.get_channellist()

        for msg in reader.read_messages():
            topic, raw_data, data_type, timestamp = msg

            # Only process channels we're interested in
            if topic not in channels_to_read:
                continue

            extractor = channel_extractor_map[topic]
            extractor.process_message(raw_data, timestamp)

            # Check count limit
            if target_count > 0 and total_processed >= target_count:
                return
```

### Channel-Extractor Mapping

```python
def build_channel_extractor_map(extractors):
    return {
        extractor.channel: extractor
        for extractor in extractors
    }
```

## Record File Format

Apollo Cyber RT record files use the following format:

### File Structure

```
┌──────────────────────────────────────┐
│           File Header                │
├──────────────────────────────────────┤
│  ┌────────────────────────────────┐  │
│  │  Channel Header                │  │
│  ├────────────────────────────────┤  │
│  │  Message 1                     │  │
│  ├────────────────────────────────┤  │
│  │  Message 2                     │  │
│  └────────────────────────────────┘  │
│  ┌────────────────────────────────┐  │
│  │  Channel Header                │  │
│  ├────────────────────────────────┤  │
│  │  Message 1                     │  │
│  └────────────────────────────────┘  │
│            ...                       │
└──────────────────────────────────────┘
```

### RecordReader API

```python
# Create reader
reader = RecordReader(file_path)

# Get available channels
channels = reader.get_channellist()
# Returns: ['/apollo/canbus/chassis', '/apollo/planning', ...]

# Read messages
for msg in reader.read_messages():
    # msg.topic: channel name (str)
    # msg.message: raw protobuf bytes (bytes)
    # msg.data_type: message type name (str)
    # msg.timestamp: nanoseconds (int)
    pass

# Get message count for a channel
count = reader.get_messagenumber(channel_name)

# Get message type for a channel
msg_type = reader.get_messagetype(channel_name)
```

## Performance Considerations

### Memory Usage

1. **Message Parsing**: Each message is parsed individually and discarded
2. **No Message Caching**: Messages are not stored in memory
3. **Streaming Output**: Output is written immediately, not buffered

### Processing Speed

1. **File I/O**: Sequential file reading is efficient
2. **Protobuf Parsing**: `ParseFromString()` is fast for most messages
3. **JSON Serialization**: Slower than text/CSV for complex messages

### Optimization Tips

1. **Use `-n` for Testing**: Process a subset of messages first
2. **Limit Channels**: Only extract needed channels
3. **Use CSV Format**: Faster than JSON for large datasets
4. **Use `--porcelain`**: Skip logging overhead

## Design Differences from online_extract.py

| Aspect | online_extract.py | offline_extract.py |
|--------|-------------------|-------------------|
| **Message Source** | Cyber RT callback | RecordReader iterator |
| **Message Format** | Parsed protobuf | Raw bytes |
| **Parsing** | Automatic | Manual `ParseFromString()` |
| **Lifecycle** | Continuous (until Ctrl+C) | Finite (ends when done) |
| **Threading** | Multi-threaded (Cyber RT) | Single-threaded |
| **State Management** | Node management | File management |

## Error Handling

### File Level Errors

- Missing directory: Error message and exit
- No record files: Warning message and exit
- Corrupted record file: Error logged, skip to next file

### Message Level Errors

- Parse error: Error logged, message skipped
- Missing message type: Warning logged, message skipped
- Channel not in map: Warning logged, message skipped

### Recovery Strategy

The tool continues processing after non-fatal errors:
- Skip corrupted messages
- Continue to next file on file-level errors
- Log all errors for post-analysis

## Extension Points

Same as `online_extract.py`:

1. **Adding New Channels**: Update `MESSAGE_TYPE_MAP`
2. **Adding Output Formats**: Extend `ChannelExtractor.process_message()`
3. **Custom Field Processing**: Subclass `ChannelExtractor`
