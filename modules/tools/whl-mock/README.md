# whl-mock

`whl-mock` is a tool for publishing arbitrary message types in Apollo. It
generates message templates, lets you edit them, and publishes the customized
messages. Typical use cases include sending routing or other messages by editing
the corresponding template file (e.g., `RoutingRequest_template.txt`) and
publishing it from the command line.

## Quick Start

### 1. Build Dependencies

Compile `cyber` and `common_msgs`:

```shell
./apollo.sh build_cpu cyber common_msgs
```

### 2. Set Up Environment

Source the environment variables:

```shell
source scripts/runtime_env.sh
```

## Publishing Messages

1. Edit the desired message template file (e.g., `<your_msg_template>.txt`).
2. Publish the message:

```shell
# Example:
## prediction
cd modules/tools/whl-mock/
python publisher.py --publish -i PredictionObstacles_template.txt -t /apollo/prediction

## planning
python publisher.py --publish -i ADCTrajectory_template.txt -t /apollo/planning

# General usage:
# python publisher.py --publish -i your_message_template.txt -t /your_topic -p 0.1
```

3. In another terminal, run:

```shell
cyber_monitor
```

to verify the published message.

## Generating Message Templates

To publish a different message type:

1. Modify the message type in the code if needed.
2. Generate a new template:

```shell
python modules/tools/whl-mock/publisher.py --gen
```
3. Use log info

log-level have five choices :"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"

```shell
python modules/tools/whl-mock/publisher.py --gen --log-level DEBUG
```
4. Edit the generated `<your_msg_template>.txt` file as needed.
