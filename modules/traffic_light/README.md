# Standalone traffic lights

The component consumes camera images and optional V2X messages, and publishes
`apollo.perception.TrafficLightDetection`. The pipeline factory is
`pipeline/traffic_light_pipeline.h`:

```text
Camera / pose / signal relations / lane context / V2X providers
  -> Tracker.Predict -> Prompter -> Detector(provider ROI inference)
  -> Tracker.Update -> Binder -> optional Heuristic -> Fusion -> Cyber output
```

The tracker owns camera-local physical identities, not map signal IDs.
Image-space constant-velocity Kalman prediction tracks center and size with
measurement-time seconds; high-score global assignment precedes low-score
continuation of confirmed tracks. This adapts ByteTrack's association policy,
not its complete upstream implementation. Low scores cannot create identities
or refresh color evidence. Missing color becomes UNKNOWN independently of
geometry expiry; clock/size discontinuities reset camera-local state.
Hungarian assignment reuses the repository's perception graph implementation.

Prompter consumes predicted boxes and uncertainty, fresh lane search regions,
and optionally reliable projections. Detector actually crops and letterboxes
the selected regions, then restores image coordinates. Periodic full-frame
exploration retains a reserved inference slot, including under ROI saturation.
Unbound visual tracks remain eligible to guide future search.

Binder prioritizes fresh lane/approach relations, observed arrows and lamp-group
cardinality; reliable projection is only supporting evidence when lane topology
is present. Missing evidence is not a conflict. Competing plausible lanes or
signals cause abstention; recent independently established relations can resolve
brief signal ambiguity but cannot refresh their own lifetime. Lane changes,
context epoch changes and time rollback invalidate history. Only current
observations contribute to groups or acquire bindings. Physical lamp clusters
do not imply a common control phase, and circular heads do not establish all
turn permissions. Ranking scores and margins are heuristic, not calibrated
probabilities.

Fusion publishes only valid, unambiguous bound visual identities or fresh,
identified strong V2X evidence. Prediction alone cannot publish a previous
visual permission. A mapped signal lacking state produces UNKNOWN; without
external signal identities the output stays empty. The heuristic stage cannot
create a measured light state or grant passage.
Before V2X arbitration, physical heads are consolidated by external `signal_id`,
not support/group ID. Conflicting colors or blink states produce one UNKNOWN
entry with zero state/output confidence; V2X cannot erase that conflict within
the frame. Thus consumers cannot choose a color by duplicate-entry order.
V2X is reduced to the newest valid timestamp per signal before scoring. A new
RED, UNKNOWN or weak update supersedes an older GREEN; equal-time color/blink
contradictions cannot grant V2X permission.

## Deployment boundaries

`conf/traffic_light.pb.txt` contains the shipped parameters. Color expiry,
geometry expiry and binding-history validity are separate durations.
`neural_detector.max_roi_inferences` bounds crop work and
`tracker.max_tracks_per_camera` bounds assignment and track storage per camera.
Deprecated frame-count expiry is retained for configuration compatibility;
expiry now uses seconds.

The default pose provider is invalid and the default signal cache starts empty.
An online `ILaneContextProviderPort` and signal-relation `IMapProviderPort`
must be supplied by the deployment; this module does not yet subscribe to a
production lane-topology stream. Pose is not required for semantic binding.
Provider setters are initialization-time injection boundaries, not concurrent
runtime reconfiguration APIs.

The configured ONNX weights are not included. The example COCO class mapping
only identifies generic traffic-light objects and does not provide verified
colors/arrows; deployment requires appropriate weights and class-label mapping.
Missing inference produces explicit degradation, not fabricated observations.
The provider uses `MultiBatchInference::InferWithStatus()` and discards all
frame candidates if any ROI invocation fails. Old output tensors, including
those left intact by a failed enqueue, are never decoded as new observations.
The existing void `Infer()` entrypoint remains for other inference consumers;
they must adopt the checked API separately to handle failures explicitly.
The checked path also requires model I/O to match the float/linear blob
contract and the actual resolved output shape before enqueueing into a buffer.
`dag/trafficlights.dag` loads the shared library and configuration. Do not run
this and the legacy traffic-light publisher on the same output topic.
The DAG names the fully qualified registered class and does not import the
legacy perception flagfile. Its shared library uses dynamic dependency linkage,
matching Cyber mainboard and preventing duplicate runtime registrations.
Malformed image stride/buffers, unrepresentable dimensions or timestamps and
odd-width YUV422 are rejected before tracking or inference; valid frames can
resume without inheriting a malformed input.

## Validation

Build and run compact tracker/binder unit tests and lint in the managed dev
container:

```bash
bazel test //modules/traffic_light:all --test_output=errors
```

Flow and simulated-hardware end-to-end coverage live in the independent
`ad_agents/integration_tests/traffic_light/` repository package. Its README
documents deterministic camera pixels, real Cyber input/output, topology oracle
and model-simulation boundaries. Annotation-injected OpenLane diagnostics and
their limitations are documented in [tools/README.md](tools/README.md).
