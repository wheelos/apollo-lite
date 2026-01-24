# send_routing.py User Guide

## Overview

`send_routing.py` is a command-line tool for sending routing requests (RoutingRequest) via Cyber RT. It helps you precisely control routing requests programmatically in both simulation and real vehicle testing scenarios, avoiding errors from manual Dreamview operations.

## Use Cases

### 1. Simulation Scenarios

In simulation testing, you can fix start and end points for consistent multi-round test comparisons:

- **Precise repeatability**: Each send uses identical waypoints, avoiding positional errors from mouse clicks in Dreamview
- **Batch testing**: Supports loop and timed sending for multiple control algorithm tests
- **Automation integration**: Can be part of automated test scripts without manual intervention

### 2. Real Vehicle Scenarios

In actual vehicle testing, you can automatically get the current vehicle position as the start point:

- **Dynamic start point**: Use `--add-pose` option to automatically fetch current vehicle position from localization channel as first waypoint
- **Fixed destination**: Preset target point or route in configuration file
- **Test comparison**: Suitable for multiple rounds of testing on the same route to evaluate algorithm consistency

## Features

- Load RoutingRequest configuration from file
- Automatically add current vehicle pose as start point
- Support single send, timed loop send, and interactive send
- Two pose refresh modes:
  - `once`: Use pose captured at startup (for simulation)
  - `fresh`: Get latest pose each time (for real vehicle)

## Installation and Dependencies

The tool is located in the Apollo project, no additional installation needed:

```bash
cd /apollo/modules/tools/whl-mock
./send_routing.py --help
```

## RoutingRequest File Format

Configuration file uses Protocol Buffer Text Format:

```protobuf
waypoint {
  pose {
    x: 272114.24
    y: 4020864.14
    z: 0.0
  }
  heading: 1.57
}
waypoint {
  pose {
    x: 272091.94
    y: 4020872.85
    z: 0.0
  }
  heading: 3.1040
}

parking_info {
  parking_space_id: "test_parking"
  parking_point {
    x: 272091.94
    y: 4020872.85
    z: 0.0
  }
  parking_space_type: VERTICAL_PLOT
  heading: 3.1040
}
```

### Field Descriptions

| Field | Description | Unit |
|-------|-------------|------|
| `waypoint[].pose.x/y/z` | Waypoint position coordinates | meters |
| `waypoint[].heading` | Waypoint heading angle | radians |
| `parking_info.parking_space_id` | Parking space ID | string |
| `parking_info.parking_point` | Parking space center point | - |
| `parking_info.parking_space_type` | Parking space type | enum |
| `parking_info.heading` | Parking space heading | radians |

## Usage

### Basic Usage

#### Single Send (Fixed Route)

For simulation scenarios, send preset fixed route:

```bash
./send_routing.py -i RoutingRequest_template.txt
```

#### Add Current Pose as Start Point

For real vehicle scenarios, automatically get current vehicle position as start point:

```bash
./send_routing.py --add-pose -i RoutingRequest_template.txt
```

### Advanced Usage

#### Timed Loop Send

Send routing request at specified intervals:

```bash
# Send every 1 second
./send_routing.py --loop --interval 1.0

# Send every 5 seconds, using cached start pose
./send_routing.py --add-pose --loop --interval 5.0 --pose-mode once
```

#### Interactive Send

Press `c` to send, `q` to quit:

```bash
# Basic interactive mode
./send_routing.py --interactive

# Interactive mode + get fresh pose each time
./send_routing.py --add-pose --interactive --pose-mode fresh
```

#### Use Custom Channel

```bash
./send_routing.py -c /apollo/custom_routing_request
```

## Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-i, --input` | Input file path | `RoutingRequest.txt` |
| `-c, --channel` | Target Cyber RT channel | `/apollo/routing_request` |
| `--add-pose` | Add current pose as first waypoint | off |
| `--localization-channel` | Localization data channel | `/apollo/localization/pose` |
| `--pose-timeout` | Pose timeout (seconds) | 10.0 |
| `--loop` | Loop mode | off |
| `--interval` | Send interval (seconds) | 1.0 |
| `--interactive` | Interactive mode (send on keypress) | off |
| `--pose-mode` | Pose refresh mode: once/fresh | `fresh` |
| `--wait` | Wait time before sending for service discovery (seconds) | 1.0 |
| `--count` | Number of times to send (0=infinite, loop mode only) | 0 |
| `--log-level` | Log level | `INFO` |

## Pose Modes Explained

### `--pose-mode once`

Use the pose captured at startup, all subsequent sends use this cached pose.

**Use case**: Simulation testing with fixed start point

```bash
./send_routing.py --add-pose --pose-mode once --loop
```

**Output example**:
```
Pose: (272114.24, 4020864.14)
Using cached pose for all iterations
Sending every 1.0s (Ctrl+C to stop)
[INFO] Sent RoutingRequest (seq=1): 3 waypoints
[INFO] Sent RoutingRequest (seq=2): 3 waypoints
...
```

### `--pose-mode fresh`

Get the latest vehicle pose each time before sending.

**Use case**: Real vehicle testing with dynamic start point

```bash
./send_routing.py --add-pose --pose-mode fresh --loop
```

**Output example**:
```
Pose: (272114.24, 4020864.14)
Getting fresh pose each iteration
Sending every 1.0s (Ctrl+C to stop)
[INFO] Sent RoutingRequest (seq=1): 3 waypoints
[INFO] Sent RoutingRequest (seq=2): 3 waypoints
...
```

## Examples

### Example 1: Simulation - Fixed Route Multi-round Testing

```bash
# Prepare configuration file
cat > test_route.txt << EOF
waypoint {
  pose { x: 272114.24 y: 4020864.14 z: 0.0 }
  heading: 1.57
}
waypoint {
  pose { x: 272091.94 y: 4020872.85 z: 0.0 }
  heading: 3.1040
}
EOF

# Send every 2 seconds for multi-round testing
./send_routing.py -i test_route.txt --loop --interval 2.0
```

### Example 2: Real Vehicle - Dynamic Start Fixed Destination

```bash
# Configuration file only contains destination
cat > destination.txt << EOF
waypoint {
  pose { x: 272091.94 y: 4020872.85 z: 0.0 }
  heading: 3.1040
}
EOF

# Automatically add current vehicle position as start point, send every 5 seconds
./send_routing.py --add-pose -i destination.txt --loop --interval 5.0
```

### Example 3: Interactive Testing

```bash
# Interactive mode, press c to send, q to quit
./send_routing.py --interactive --add-pose

# Output:
# === Interactive Mode ===
# c - send, q - quit
# > c
# [INFO] Sent RoutingRequest (seq=1): 2 waypoints
# > c
# [INFO] Sent RoutingRequest (seq=2): 2 waypoints
# > q
```

### Example 4: Automated Batch Testing

```bash
# Combine with bash script for automated testing
for i in {1..10}; do
  echo "Running test $i..."
  ./send_routing.py -i test_route.txt
  sleep 30  # Wait for test to complete
done
```

## FAQ

### Q: Why is my request not taking effect?

A: Please check the following:
1. Ensure Cyber RT is running: `cyber_launch status`
2. Ensure Routing module is running
3. Check if channel name is correct
4. Use `--log-level DEBUG` for detailed logs

### Q: How do I get current vehicle position?

A: Use one of the following methods:
```bash
# Method 1: Use cyber_monitor
cyber_monitor /apollo/localization/pose

# Method 2: Use this tool
./send_routing.py --add-pose --log-level DEBUG
```

### Q: What's the difference between `--pose-mode once` and `fresh`?

A:
- `once`: Get pose once at startup, use that pose for all subsequent sends
- `fresh`: Get latest pose before each send

### Q: How to debug RoutingRequest configuration?

A:
1. Send a request using Dreamview
2. Check Routing Response in Dreamview's PnC Monitor
3. Copy valid waypoint configuration to your file

## Technical Details

### Message Format

The tool sends Apollo `RoutingRequest` message, defined at:
`modules/common_msgs/routing_msgs/routing.proto`

### Channel Information

- **Default channel**: `/apollo/routing_request`
- **Localization channel**: `/apollo/localization/pose`

### Sequence Number

Each send automatically increments the sequence number, which can be used to track request order.

## Related Tools

- **Dreamview**: Visualize routing requests and responses
- **cyber_monitor**: Monitor Cyber RT channel messages
- **cyber_recorder**: Record and replay messages
