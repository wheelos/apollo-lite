# offline_extract.py - Quick Start Guide

## Overview

`offline_extract.py` is a data extraction tool for Apollo Cyber RT record files. It reads data from `.record` files and outputs specified fields, supporting multiple channels and field combinations.

## Prerequisites

- Python 3 environment
- Cyber RT python modules available
- Apollo record files (`.record` format)

## Installation

No installation required. The script is located at:
```
modules/tools/whl-debug/offline_extract.py
```

## Basic Usage

### 1. List All Supported Channels

```bash
python modules/tools/whl-debug/offline_extract.py --list-channels
```

Output:
```
Supported channels (9):
  /apollo/canbus/chassis -> Chassis
  /apollo/canbus/chassis_detail -> ChassisDetail
  /apollo/localization/pose -> LocalizationEstimate
  /apollo/planning -> ADCTrajectory
  /apollo/hmi/status -> HMIStatus
  /apollo/control -> ControlCommand
  /apollo/prediction -> PredictionObstacles
  /apollo/perception/obstacles -> PerceptionObstacles
  /apollo/routing_request -> RoutingRequest
  /apollo/routing_response -> RoutingResponse
```

### 2. Extract Single Field from One Channel

Extract chassis speed from record files:
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f speed_ms
```

Output:
```
[/apollo/canbus/chassis][1769155734637735350] 2.45
[/apollo/canbus/chassis][1769155734722863806] 2.52
[/apollo/canbus/chassis][1769155734822936175] 2.48
```

### 3. Extract Multiple Fields (CSV Format)

Extract chassis speed and steering percentage:
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f speed_ms \
  -f steering_percentage \
  --output-format csv
```

Output:
```
channel,ts,speed_ms,steering_percentage
/apollo/canbus/chassis,1769155734637735350,2.45,-5.2
/apollo/canbus/chassis,1769155734722863806,2.52,-5.1
```

### 4. Extract from Multiple Channels

Extract chassis speed and planning decision:
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  -c /apollo/planning -f decision
```

### 5. Output Full Message

Use `-f .` to output the complete message:
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f .
```

### 6. Limit Message Count

Extract only 100 messages:
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  -n 100
```

### 7. Porcelain Mode (Data Only)

Suppress log output, only print data:
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms \
  --porcelain
```

## Output Formats

### Text Format (default)
```
[/apollo/canbus/chassis][timestamp] value1 | value2
```

### CSV Format
```bash
--output-format csv
```
Output:
```
channel,ts,field1,field2
/apollo/channel,1234567890,1.0,2.0
```

### JSON Format
```bash
--output-format json
```
Output:
```json
{"channel":"/apollo/canbus/chassis","ts":1234567890,"speed_ms":2.45}
```

## Common Use Cases

### Analyze Chassis Data
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis \
  -f speed_ms -f steering_percentage -f throttle -f brake \
  --output-format csv > chassis_analysis.csv
```

### Debug Planning Output
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/planning \
  -f decision -f trajectory_type
```

### Extract Localization Data
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/localization/pose \
  -f pose.position.x -f pose.position.y -f pose.heading
```

### Multi-Channel CSV Export
```bash
python modules/tools/whl-debug/offline_extract.py \
  -i /path/to/record_dir \
  -c /apollo/canbus/chassis -f speed_ms -f throttle \
  -c /apollo/planning -f decision \
  --output-format csv --porcelain > output.csv
```

## Input Directory

The `-i` option specifies a directory containing record files:
```
/path/to/record_dir/
├── record.00000
├── record.00001
└── record.00002
```

Files are processed in alphabetical order.

## Tips

1. Use `--porcelain` mode when redirecting output to file
2. Use CSV format for data analysis in Excel/pandas
3. The tool processes all `.record` files in the input directory
4. Use `-n` to limit processing for quick testing
5. Each `-c` starts a new channel group, subsequent `-f` applies to that channel

## Difference from online_extract.py

| Feature | online_extract.py | offline_extract.py |
|---------|-------------------|-------------------|
| Data Source | Cyber RT (live) | Record files |
| Requires Cyber RT | Yes | No |
| Use Case | Real-time monitoring | Post-analysis |
| Input Option | None | `-i` (directory) |
