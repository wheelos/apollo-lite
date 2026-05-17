# Smart Recorder

## Introduction

Smart Recorder is a triggered roadlog capture service for autonomous-driving
incident analysis.

Its runtime model is:

1. record a segmented rolling ring under one task root
2. evaluate lightweight semantic triggers online
3. merge or extend overlapping incidents through trigger cooldown rules
4. export only the retained segment set plus metadata when an incident closes

The live path does not reread historical record files or rebuild messages on the
hot path.

The runtime is now split into focused modules:

1. `RoadlogRuntime` orchestrates startup, periodic sync, and shutdown
2. `RoadlogRecordingService` configures and owns the recorder plane
3. `RoadlogTriggerManager` owns trigger instances, trigger readers, and passive
   trigger ticking
4. `RoadlogSegmentManager` owns completed-segment discovery, pin counts, and ring
   eviction
5. `RoadlogEventManager` owns trigger arbitration, incident state, and export
   readiness
6. `RoadlogEventExporter` writes event manifests plus retained segments

Runtime rules that now hold by construction:

1. built-in triggers omitted from config default to disabled
2. recorder policy cannot exclude or rate-limit enabled trigger channels
3. export retirement re-checks newly arrived trigger hits before deleting an
   incident, so reopenable incidents are not split just because export was in
   flight
4. `coverage=complete` now requires continuous segment coverage across the full
   exported window

## How to use

1. Build apollo
2. python3 /apollo/scripts/record_message.py --help
3. `record_message.py` now launches `smart_recorder` with
   `--roadlog_root_dir=/apollo/data/roadlog/<task_id>` and resolves the modular
   binary from either the Bazel output or the installed package path
4. vehicle-specific `data_conf` should target
   `/apollo/modules/tools/roadlog/conf/smart_recorder_config.pb.txt`

## How to add new scenarios

1. Configure the new scenario in conf/smart_recorder_config.pb.txt, including
    time range, cooldown, name, description and etc.
2. Add new class inherit from base class "TriggerBase", and implement interface
    "Pull"
3. Add the instance of the new class into `RoadlogTriggerManager::InitTriggers`

## Runtime layout

`smart_recorder` now takes one root directory:

```bash
smart_recorder --roadlog_root_dir=/apollo/data/roadlog/<task_id>
```

and creates:

- `ring/` for rolling `.record.*` segments
- `events/` for exported incident packages
- `meta/` for trigger metadata logs

## Recorder-plane filtering

`smart_recorder_config.pb.txt` can now reduce recorder pressure without changing
trigger semantics:

```text
recorder_policy {
  include_channels: "/apollo/planning"
  exclude_channels: "/apollo/sensor/lidar/fusion/PointCloud2"
  channel_rate_limits {
    channel_name: "/apollo/sensor/camera/front_6mm/image"
    max_rate_hz: 1.0
  }
  large_message_policy {
    drop_message_size_bytes: 1048576
  }
}
```

Rules:

1. `include_channels` is an optional allowlist; when omitted, roadlog records all channels except explicit excludes
2. `exclude_channels` is a recorder-plane blacklist
3. `channel_rate_limits` keeps a topic but caps its saved frequency
4. `large_message_policy.drop_message_size_bytes` drops oversized payloads by
   size only
5. frequency control belongs in `channel_rate_limits`, not in
   `large_message_policy`
6. a topic cannot appear in both `include_channels` and `exclude_channels`
7. topics needed by enabled triggers cannot be excluded or rate-limited

This keeps resource control below the trigger plane, which is the intended
production boundary.
