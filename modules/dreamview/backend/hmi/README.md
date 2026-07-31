# Customize Your Own HMI

HMIWorker is a standalone singleton which processes HMI actions. If you want to
have a customized HMI instead the one integrated with Apollo Dreamview, just
develope a frontend and send operations to the backend which delegates to
HMIWorker.

## HMI Config

See modules/dreamview/proto/hmi_config.proto. It defines all supported modes,
modules, hardware and tools.

## HMI Worker

According to the HMIConfig, HMI Worker could trigger actions like:
- Change mode, map, vehicle and driving mode.
- Register event handler for changing mode, map and vehicle.
- Start, stop or execute other registered commands for modules.
- Execute registered tools.
- Submit DriveEvent which will be recorded as a ROS message.
- Get current HMIConfig and HMIStatus, which could be used for UI update.

### Process Orchestration Modules

To keep process control logic decoupled, HMI now separates responsibilities:

- `ModeRegistry`: load mode files and deeply merge `base_mode` inheritance.
- `ProcessManager`: own lifecycle state, dependencies, and exclusivity.
- `LocalRunner`: execute, stop, and reap dedicated process groups.
- `ReadinessProbe`: verify the managed process identity and readiness keywords.
- `HMIStatusBridge`: write module running state into `HMIStatus`.
