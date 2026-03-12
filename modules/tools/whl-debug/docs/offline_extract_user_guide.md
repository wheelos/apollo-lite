# offline_extract.py - User Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Command Line Options](#command-line-options)
3. [Channel and Field Syntax](#channel-and-field-syntax)
4. [Output Formats](#output-formats)
5. [Advanced Usage](#advanced-usage)
6. [Examples](#examples)
7. [Troubleshooting](#troubleshooting)

## Introduction

`offline_extract.py` is a data extraction tool for Apollo Cyber RT record files. It reads data from `.record` files and extracts specified fields, supporting various output formats for post-analysis and debugging.

### Key Features

- **Multi-channel support**: Extract from multiple channels in one command
- **Multi-field extraction**: Extract multiple fields per channel
- **Batch processing**: Process entire directories of record files
- **Flexible output formats**: Text, CSV, or JSON
- **Flexible field syntax**: Dot notation for nested fields

## Command Line Options

### Required Options

| Option | Short | Description |
|--------|-------|-------------|
| `--input-dir` | `-i` | Path to directory containing record files (required) |

### Basic Options

| Option | Short | Description |
|--------|-------|-------------|
| `--channel` | `-c` | Cyber RT channel name (can be specified multiple times) |
| `--field` | `-f` | Field path to extract (can be specified multiple times) |
| `--count` | `-n` | Number of messages to process (default: -1 for all) |

### Output Options

| Option | Description |
|--------|-------------|
| `--output-format` | Output format: `text`, `csv`, or `json` (default: text) |
| `--separator` | Separator for CSV output (default: ",") |
| `--porcelain` | Suppress all log output, only print data |

### Utility Options

| Option | Description |
|--------|-------------|
| `--list-channels` | List all supported channels and exit |
| `--log-level` | Log level: `DEBUG`, `INFO`, `WARNING`, `ERROR` (default: INFO) |

## Channel and Field Syntax

The channel and field syntax is identical to `online_extract.py`. See [online_extract_user_guide.md](online_extract_user_guide.md) for details.

### Channel Specification

Each `-c` option starts a new channel group. Subsequent `-f` options apply to the most recent `-c`.

### Field Path Syntax

Use dot notation to access nested fields:

| Field Path | Description |
|------------|-------------|
| `speed_ms` | Top-level field |
| `pose.position.x` | Nested field (3 levels deep) |
| `header.sequence_num` | Nested field with underscore |
| `.` | Root level (entire message) |

## Output Formats

The output formats are identical to `online_extract.py`. See [online_extract_user_guide.md](online_extract_user_guide.md) for details.

### Text Format (default)
```
[/apollo/canbus/chassis][1769155734637735350] 2.45 | -5.2 | 12.3
```

### CSV Format
```
channel,ts,speed_ms,steering_percentage,throttle
/apollo/canbus/chassis,1769155734637735350,2.45,-5.2,12.3
```

### JSON Format
```json
{"channel":"/apollo/canbus/chassis","ts":1769155734637735350,"speed_ms":2.45}
```

## Advanced Usage

### Process Multiple Record Files

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms
```

All `.record` files in the directory are processed in alphabetical order.

### Multi-Channel Analysis

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms -f throttle \
  -c /apollo/planning -f decision \
  --output-format csv
```

### Limit Processing for Quick Test

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  -n 10
```

### Export for Data Analysis

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms -f steering_percentage \
  -c /apollo/control -f throttle -f brake \
  --output-format csv --porcelain \
  > analysis_data.csv
```

### Custom CSV Separator

```bash
python offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  --output-format csv --separator "\t"
```

## Examples

### Example 1: Extract Vehicle Speed

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/canbus/chassis -f speed_ms
```

### Example 2: Analyze Control Commands

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/control \
  -f throttle -f brake -f steering_rate \
  --output-format csv --porcelain \
  > control_analysis.csv
```

### Example 3: Multi-Channel Debug

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/canbus/chassis -f speed_ms -f gear_location \
  -c /apollo/planning -f decision \
  -c /apollo/control -f throttle -f brake
```

### Example 4: Localization Trajectory

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/localization/pose \
  -f pose.position.x \
  -f pose.position.y \
  -f pose.heading \
  --output-format csv
```

### Example 5: Extract Nested Planning Data

```bash
python offline_extract.py \
  -i 20260123/data \
  -c /apollo/planning \
  -f trajectory_point.0.path_point.x \
  -f trajectory_point.0.path_point.y \
  -f trajectory_point.0.path_point.theta
```

## Troubleshooting

### Issue: "No record files found in directory"

**Cause**: The specified directory contains no `.record` files.

**Solution**:
1. Verify the directory path is correct
2. Check that files exist: `ls -la /path/to/record_dir/`
3. Ensure files have `.record` extension

### Issue: "Channel not in predefined mapping"

**Cause**: The channel is not in the `CHANNEL_MESSAGE_TYPE_MAP`.

**Solution**:
1. Check available channels with `--list-channels`
2. Add the channel and message type to `CHANNEL_MESSAGE_TYPE_MAP` in the script

### Issue: Field returns "N/A" or empty

**Cause**: Field path is incorrect or field doesn't exist in the message.

**Solution**:
1. Use `-f .` to output full message and check available fields
2. Verify field path syntax (use dots for nested fields)
3. Check if the channel has data in the record files

### Issue: "Failed to parse message"

**Cause**: Message type mismatch or corrupted data.

**Solution**:
1. Verify the channel mapping in `MESSAGE_TYPE_MAP`
2. Check if record files are valid

### Issue: Processing is slow

**Cause**: Large record files or many channels.

**Solution**:
1. Use `-n` to limit processing for testing
2. Process specific channels instead of all
3. Use `--output-format csv` for faster processing than JSON

## Record File Format

The tool expects Apollo Cyber RT `.record` files. These are binary files containing serialized protobuf messages with channel headers.

```
/path/to/record_dir/
├── record.00000
├── record.00001
└── record.00002
```

Files are processed in alphabetical order (`record.00000`, `record.00001`, ...).

## Supported Channels

See [online_extract_user_guide.md](online_extract_user_guide.md#supported-channels) for the complete list of supported channels.

## Comparison with online_extract.py

| Feature | online_extract.py | offline_extract.py |
|---------|-------------------|-------------------|
| **Data Source** | Live Cyber RT | Record files |
| **Cyber RT Required** | Yes (running) | No (python modules only) |
| **Use Case** | Real-time monitoring | Post-analysis |
| **Input** | None | `-i` (directory) |
| **Processing** | Continuous | Finite (ends when done) |
| **Message Count** | `-n` then stops | `-n` or all messages |
| **Dependencies** | cyber.PyNode | RecordReader |

## Typical Workflow

1. **Record Data**: Use `cyber_recorder` to record data
   ```bash
   cyber_recorder record -a -o record_dir
   ```

2. **Extract Data**: Use `offline_extract.py` to extract fields
   ```bash
   python offline_extract.py -i record_dir -c /apollo/canbus/chassis -f speed_ms
   ```

3. **Analyze Data**: Import into analysis tools
   ```bash
   python offline_extract.py -i record_dir -c /apollo/canbus/chassis -f speed_ms \
     --output-format csv --porcelain > data.csv
   # Analyze in pandas, Excel, etc.
   ```
