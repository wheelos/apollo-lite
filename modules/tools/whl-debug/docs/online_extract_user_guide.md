# online_extract.py - User Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Command Line Options](#command-line-options)
3. [Channel and Field Syntax](#channel-and-field-syntax)
4. [Output Formats](#output-formats)
5. [Advanced Usage](#advanced-usage)
6. [Examples](#examples)
7. [Troubleshooting](#troubleshooting)

## Introduction

`online_extract.py` is a real-time data extraction tool for Apollo Cyber RT. It subscribes to multiple channels simultaneously and extracts specified fields, supporting various output formats for data analysis and debugging.

### Key Features

- **Multi-channel support**: Subscribe to multiple channels in one command
- **Multi-field extraction**: Extract multiple fields per channel
- **Real-time output**: Text, CSV, or JSON format
- **Flexible field syntax**: Dot notation for nested fields
- **Full message output**: Output complete message when needed

## Command Line Options

### Basic Options

| Option | Short | Description |
|--------|-------|-------------|
| `--channel` | `-c` | Cyber RT channel name (can be specified multiple times) |
| `--field` | `-f` | Field path to extract (can be specified multiple times) |
| `--count` | `-n` | Number of messages to process (default: -1 for infinite) |

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

### Channel Specification

Each `-c` option starts a new channel group. Subsequent `-f` options apply to the most recent `-c`.

```
-c <channel1> -f <field1> -f <field2> -c <channel2> -f <field3>
```

This means:
- `<channel1>` extracts `<field1>` and `<field2>`
- `<channel2>` extracts `<field3>`

### Field Path Syntax

Use dot notation to access nested fields:

| Field Path | Description |
|------------|-------------|
| `speed_ms` | Top-level field |
| `pose.position.x` | Nested field (3 levels deep) |
| `header.sequence_num` | Nested field with underscore |
| `.` | Root level (entire message) |

### Special Field Values

- `.` or empty string: Output the entire message
- Omitting `-f`: Same as specifying `-f .`

## Output Formats

### Text Format (Default)

```
[/apollo/canbus/chassis][1769155734.63773] 2.45 | -5.2 | 12.3
```

Format: `[channel][timestamp] value1 | value2 | value3`

### CSV Format

```bash
--output-format csv
```

Output:
```
channel,ts,speed_ms,steering_percentage,throttle
/apollo/canbus/chassis,1769155734637735350,2.45,-5.2,12.3
/apollo/canbus/chassis,1769155734722863806,2.52,-5.1,12.5
```

Features:
- Unified header with all fields from all channels
- Empty values for fields not belonging to current channel
- Consistent column order across all channels

### JSON Format

```bash
--output-format json
```

Output:
```json
{"channel":"/apollo/canbus/chassis","ts":1769155734637735350,"speed_ms":2.45,"steering_percentage":-5.2}
{"channel":"/apollo/canbus/chassis","ts":1769155734722863806,"speed_ms":2.52,"steering_percentage":-5.1}
```

Each line is a complete JSON object.

## Advanced Usage

### Multi-Channel with Different Fields

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f throttle \
  -c /apollo/planning -f decision \
  -c /apollo/localization/pose -f pose.position.x
```

### Mixed: Full Message + Specific Fields

```bash
python online_extract.py \
  -c /apollo/canbus/chassis \
  -c /apollo/planning -f decision
```

This outputs:
- Full message for `/apollo/canbus/chassis`
- Only `decision` field for `/apollo/planning`

### Custom CSV Separator

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms \
  --output-format csv --separator "\t"
```

### Data Collection for Analysis

```bash
# Collect data to CSV file
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f throttle -f brake \
  --output-format csv --porcelain \
  -n 1000 > chassis_data.csv
```

### Debug with Verbose Logging

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms \
  --log-level DEBUG
```

## Examples

### Example 1: Monitor Vehicle Speed

```bash
python online_extract.py -c /apollo/canbus/chassis -f speed_ms
```

Output:
```
[/apollo/canbus/chassis][1769155734.63773] 2.45
[/apollo/canbus/chassis][1769155734.72860] 2.52
[/apollo/canbus/chassis][1769155734.81947] 2.48
```

### Example 2: Collect Control Data

```bash
python online_extract.py \
  -c /apollo/control \
  -f throttle -f brake -f steering_rate \
  --output-format csv --porcelain \
  -n 500 > control_data.csv
```

### Example 3: Multi-Channel for System Debug

```bash
python online_extract.py \
  -c /apollo/canbus/chassis -f speed_ms -f gear_location \
  -c /apollo/planning -f decision \
  -c /apollo/control -f throttle -f brake
```

### Example 4: Localization Position Tracking

```bash
python online_extract.py \
  -c /apollo/localization/pose \
  -f pose.position.x \
  -f pose.position.y \
  -f pose.heading
```

### Example 5: Extract Nested Planning Data

```bash
python online_extract.py \
  -c /apollo/planning \
  -f trajectory_point.0.path_point.x \
  -f trajectory_point.0.path_point.y
```

## Troubleshooting

### Issue: "Channel not in predefined mapping"

**Cause**: The channel is not in the `CHANNEL_MESSAGE_TYPE_MAP`.

**Solution**:
1. Check available channels with `--list-channels`
2. Add the channel and message type to `CHANNEL_MESSAGE_TYPE_MAP` in the script

### Issue: No data output

**Possible causes**:
1. Channel is not publishing data
2. Cyber RT is not running
3. Wrong channel name

**Solutions**:
1. Check if Cyber RT is running: `cyber_monitor`
2. Verify channel name with `--list-channels`
3. Enable debug logging: `--log-level DEBUG`

### Issue: Field returns "N/A"

**Cause**: Field path is incorrect or field doesn't exist in the message.

**Solution**:
1. Use `-f .` to output full message and check available fields
2. Verify field path syntax (use dots for nested fields)

### Issue: CSV has empty columns

**Cause**: When extracting from multiple channels, each channel only has its own fields.

**Solution**: This is expected behavior. Fields not belonging to a channel will be empty.

## Supported Channels

| Channel | Message Type | Common Fields |
|---------|-------------|---------------|
| `/apollo/canbus/chassis` | Chassis | speed_ms, throttle, brake, steering_percentage, gear_location |
| `/apollo/localization/pose` | LocalizationEstimate | pose.position.x, pose.position.y, pose.heading |
| `/apollo/planning` | ADCTrajectory | decision, trajectory_type, trajectory_point |
| `/apollo/control` | ControlCommand | throttle, brake, steering_rate |
| `/apollo/perception/obstacles` | PerceptionObstacles | obstacle, timestamp |
| `/apollo/prediction` | PredictionObstacles | obstacle, timestamp |
