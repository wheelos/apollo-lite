# Routing Tools Comparison: send_routing.py vs routing_request.py

## Overview

Both `send_routing.py` and `routing_request.py` are tools for sending `RoutingRequest` messages via Apollo Cyber RT. This comparison highlights their strengths, weaknesses, and appropriate use cases.

## Feature Comparison Table

| Feature | send_routing.py | routing_request.py |
|---------|----------------|-------------------|
| Configuration file support | ✅ Yes | ❌ No (hardcoded) |
| Command-line arguments | ✅ Full CLI | ❌ None |
| Dynamic start point | ✅ Optional | ✅ Always |
| Multiple waypoints | ✅ Yes | ❌ No (start + end only) |
| Send modes | Single/Loop/Interactive | Once then exit |
| Service discovery wait | ✅ Configurable (default 1.0s) | ❌ No |
| Send count control | ✅ Yes | ❌ No |
| Custom channel support | ✅ Yes | ❌ No |
| Interactive mode | ✅ Yes | ❌ No |
| parking_info support | ✅ Yes | ❌ No |
| Ease of use | Requires CLI knowledge | Just run |
| Code complexity | Higher (~380 lines) | Lower (~205 lines) |

## send_routing.py

### Strengths (优势)

1. **Flexible Configuration**
   - Load routes from external `.pb.txt` files
   - No need to modify code to change routes
   - Supports multiple waypoints and parking_info

2. **Multiple Send Modes**
   - Single send with service discovery wait
   - Loop mode with configurable interval and count
   - Interactive mode for manual control

3. **Reliable Delivery**
   - Built-in service discovery wait (default 1.0s) ensures downstream receives messages
   - Writer return value validation
   - Proper cleanup before shutdown

4. **Production-Ready Features**
   - Configurable log levels
   - Custom channel support
   - Pose refresh modes (once/fresh)

### Weaknesses (劣势)

1. **Higher Complexity**
   - More code (~380 lines)
   - More dependencies (click, uuid, select)
   - Steeper learning curve

2. **Requires CLI Knowledge**
   - Need to learn command-line options
   - More complex usage syntax

### Best Use Cases (适用场景)

- ✅ Automated testing with multiple routes
- ✅ Production debugging and diagnostics
- ✅ Performance testing (loop mode with count)
- ✅ Interactive development and testing
- ✅ Scenarios requiring complex routing (multiple waypoints, parking_info)
- ✅ Integration with test scripts

**Example Usage:**
```bash
# Automated test: send 5 times with 2-second interval
./send_routing.py -i test_route.txt --loop --interval 2.0 --count 5

# Interactive testing with dynamic start point
./send_routing.py --add-pose --interactive

# Custom channel for debugging
./send_routing.py -i route.txt -c /apollo/custom_routing
```

## routing_request.py

### Strengths (优势)

1. **Simplicity**
   - Minimal code (~205 lines)
   - No dependencies beyond standard library
   - Easy to understand and modify

2. **Zero Configuration**
   - Just run the script
   - No command-line arguments to remember
   - Auto-fetches start point from localization

3. **Auto-Shutdown**
   - Publishes once and exits cleanly
   - Suitable for one-shot operations

### Weaknesses (劣势)

1. **Inflexible**
   - Destination hardcoded in source
   - Must modify code to change routes
   - No file-based configuration

2. **No Service Discovery Wait**
   - First send may fail if routing module isn't ready
   - No retry mechanism
   - Less reliable in production

3. **Limited Functionality**
   - Only supports start + end (no intermediate waypoints)
   - No parking_info support
   - No loop or interactive modes
   - Fixed channel name

### Best Use Cases (适用场景)

- ✅ Quick one-off tests with fixed destination
- ✅ Learning Cyber RT basics
- ✅ Embedded in other scripts
- ✅ Simple verification tests
- ✅ Development environments with simple routing needs

**Example Usage:**
```bash
# Just run it
python3 routing_request.py
```

**To change behavior, edit source:**
```python
ORIGINAL_END_X = -3.886315019772555  # Change end point
ORIGINAL_END_Y = 37.61260329902406
EXTEND_LENGTH = 5.23 / 2              # Change extension
```

## Decision Matrix

| Scenario | Recommended Tool | Reason |
|----------|-----------------|--------|
| Production testing | send_routing.py | Reliable delivery, flexible configuration |
| Quick verification | routing_request.py | Zero setup, just run |
| Automated test suite | send_routing.py | Loop mode with count, file-based config |
| Learning Cyber RT | routing_request.py | Simpler code, easier to understand |
| Complex routes | send_routing.py | Multiple waypoints, parking_info support |
| Debugging specific channel | send_routing.py | Custom channel support |
| Interactive development | send_routing.py | Interactive mode, pose refresh options |
| One-shot fixed route | routing_request.py | Simple, auto-exit |

## Known Issues

### Service Discovery Timing (Both Tools)

Cyber RT's service discovery mechanism needs time to register the writer before messages can be delivered to subscribers.

**send_routing.py**: ✅ **Solved**
```python
if wait > 0:
    time.sleep(wait)  # Default 1.0s
```

**routing_request.py**: ❌ **Not addressed**
```python
# No wait before publishing - first send may fail
```

**Recommendation for routing_request.py:**
Add a wait after creating the writer:
```python
writer = node.create_writer(ROUTING_REQUEST_TOPIC, RoutingRequest)
time.sleep(1.0)  # Allow service discovery
```

## Summary

| Tool | Best For | Trade-off |
|------|----------|-----------|
| **send_routing.py** | Production/testing/automation | Complexity for reliability and flexibility |
| **routing_request.py** | Quick experiments/learning | Simplicity for limited functionality |

**Bottom line:**
- Use **send_routing.py** for production, testing, and any scenario where reliability matters
- Use **routing_request.py** for quick one-off tests and learning purposes
