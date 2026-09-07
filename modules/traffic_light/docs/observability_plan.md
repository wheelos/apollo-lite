# Traffic Light Observability Plan

## Goals

- Publish traffic-light debug visualization to web in real time.
- Persist complete, replayable debug evidence into record.
- Keep the perception critical path stable (no blocking UI or disk IO in hot path).
- Enable reproducible offline diagnosis with frame-level correlation keys.

## Design Principles (Industry Practices)

- Separate real-time decision path from debug path.
- Use tiered telemetry: lightweight always-on + rich debug-on-demand.
- Standardize correlation fields across all debug topics.
- Prefer topic-based record over local file dumps.
- Keep debug publishers non-blocking and bounded.

## End-to-End Flow

1. Ingest image and V2X in `TrafficLightComponent`.
2. Run stages (`Prompter -> Detector -> Binder -> Tracker -> Heuristic -> Fusion`).
3. Publish primary decision to `/apollo/perception/traffic_light`.
4. Publish rich debug message to `/apollo/perception/traffic_light/debug`.
5. Republish frame image to `/apollo/perception/traffic_light/debug/image` for web stream and record.
6. Record includes decision + debug proto + debug image topics.

## Data Model

### A. Primary output (existing)

- Topic: `/apollo/perception/traffic_light`
- Type: `apollo.perception.TrafficLightDetection`
- Purpose: low-latency vehicle behavior input.

### B. Rich debug output (new usage of existing proto)

- Topic: `/apollo/perception/traffic_light/debug`
- Type: `apollo.perception.TrafficLightDetection`
- Purpose: record-friendly structured diagnostics.
- Populated fields:
  - `header.timestamp_sec`, `header.camera_timestamp`
  - final `traffic_light[]`
  - `traffic_light_debug.signal_num`
  - `traffic_light_debug.debug_roi[]` from prompts
  - `traffic_light_debug.box[]` from detections/tracks
  - `traffic_light_debug.projected_roi[]` from map signals
  - `traffic_light_debug.rectified_roi[]` from tracked states
  - health summary in existing numeric fields where applicable

### C. Debug image output (new)

- Topic: `/apollo/perception/traffic_light/debug/image`
- Type: `apollo.drivers.Image`
- Purpose: direct web rendering and record replay.
- Note: overlay rendering can be added later in async worker; phase-1 republishes source frame to avoid hot-path CPU spikes.

## Interface Contract

- `IResultWriterPort` remains unchanged.
- Add a fanout writer implementation to publish one frame to multiple sinks:
  - primary decision writer
  - debug decision writer
- Add component-level image debug publisher, controlled by toggle.

## Reliability and Performance Constraints

- No `imshow`/`imwrite` in runtime critical path.
- Debug publishing must not block primary decision publication.
- Bounded memory for buffered data remains enforced.
- All debug streams use stable topic names and can be disabled.

## Record Strategy

For troubleshooting sessions, include at minimum:

- `/apollo/perception/traffic_light`
- `/apollo/perception/traffic_light/debug`
- `/apollo/perception/traffic_light/debug/image`
- `/apollo/v2x/traffic_light`
- source camera topic (if different from debug image topic)

Recommended operation modes:

- Always-on lightweight: primary output + debug proto.
- Triggered rich mode: add debug image topic when anomaly triggers.

## Web Integration

Dreamview image endpoint already consumes camera image topics via adapter flags.
To view traffic-light debug image directly:

- point `image_front_topic` (or equivalent running flag) to `/apollo/perception/traffic_light/debug/image`
- keep traffic light detection topic as `/apollo/perception/traffic_light`

## Rollout Plan

1. Implement debug proto publisher and fanout writer.
2. Implement debug image republisher in component.
3. Add toggles/channel names in component defaults.
4. Verify build and runtime topic outputs.
5. Add record command template for debug sessions.
