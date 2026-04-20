# Cyber Python Publish and Subscribe

This note captures the runtime pattern for using Python to publish and subscribe to Cyber RT messages in Apollo Lite.

## Required Initialization

Before running Python Cyber tools inside the container:

```bash
cd /apollo
source cyber/setup.bash
```

`cyber/setup.bash` prepares the Python runtime by:

- adding `/apollo` to `PYTHONPATH` for source-tree imports,
- adding `bazel-bin` to `PYTHONPATH` for generated `*_pb2.py` files,
- adding `bazel-bin/cyber/python/internal` to `PYTHONPATH` for `_cyber_wrapper.so`,
- exporting protobuf and Cyber runtime environment variables.

Without this step, Python publishers and subscribers often fail on protobuf imports or `_cyber_wrapper.so` lookup.

## Publish Pattern

`modules/tools/whl-mock/send_routing.py` is the reference publisher pattern:

1. `cyber.init()`
2. `node = cyber.Node(...)`
3. `writer = node.create_writer(channel, MessageType)`
4. Fill headers or other required fields
5. `writer.write(message)`
6. `cyber.shutdown()` during cleanup

Minimal example:

```python
from cyber.python.cyber_py3 import cyber
from modules.common_msgs.routing_msgs.routing_pb2 import RoutingRequest

cyber.init()
node = cyber.Node("routing_sender")
writer = node.create_writer("/apollo/routing_request", RoutingRequest)

request = RoutingRequest()
writer.write(request)

cyber.shutdown()
```

## Subscribe Pattern

`send_routing.py` already shows the reader side through its localization subscriber:

1. `cyber.init()`
2. `node = cyber.Node(...)`
3. `node.create_reader(channel, MessageType, callback)`
4. Keep the process alive while callbacks update local state
5. `cyber.shutdown()` during cleanup

Minimal example:

```python
from cyber.python.cyber_py3 import cyber
from modules.common_msgs.planning_msgs.planning_pb2 import ADCTrajectory

def callback(msg: ADCTrajectory):
    print(msg.header.sequence_num)

cyber.init()
node = cyber.Node("planning_listener")
node.create_reader("/apollo/planning", ADCTrajectory, callback)

while not cyber.is_shutdown():
    time.sleep(0.1)
```

## Recommended Tools

- Publish routing requests: `modules/tools/whl-mock/send_routing.py`
- Subscribe to planning debug output: `modules/tools/whl-mock/subscribe_planning.py`

## Runtime Rule

For planning and simulation debugging, prefer reading structured fields from `/apollo/planning` over adding new per-cycle text logs. The message already carries lane-borrow and other debug data in `ADCTrajectory.debug.planning_data`.
