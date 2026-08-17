# Functional Modes

This folder contains functional HMI modes. Each pb.txt should
be an instance of HMIMode. Check the proto for detailed information.

## Shared Base Mode

To reduce duplicated config blocks, modes inherit base modules by function:

```
base_mode: "_base/runtime_base.pb.txt"
```

`base_mode` supports relative path (resolved from the current mode file) or
absolute path. Child module settings override only the fields they specify;
unmodified base settings remain in effect.

`auto_start: false` keeps an optional tool available for explicit HMI start
without launching it during `SETUP_MODE`. Modules with the same
`exclusive_group` cannot run simultaneously.

## Mode Categories

Current top-level modes are grouped into four functions:

- `runtime.pb.txt`: running mode.
- `sensor_calibration.pb.txt`: sensor calibration mode.
- `map_collection.pb.txt`: map collection mode.
- `testing.pb.txt`: testing mode (mock + recorder tools).

## Name Convention

We'll convert the file name to a readable title-case name automatically to
display on Dreamview. So please make it simple, clean and meaningful.

Some examples:

* runtime.pb.txt -> "Runtime"
* sensor_calibration.pb.txt -> "Sensor Calibration"
* map_collection.pb.txt -> "Map Collection"
* testing.pb.txt -> "Testing"

## Monitor New Channels

The Cyber Reader API requires client to provide data type when subscribing a
channel. So you need to extend the ChannelMonitor to
[handle new channels](https://github.com/ApolloAuto/apollo/blob/master/modules/monitor/software/channel_monitor.cc#L51).
