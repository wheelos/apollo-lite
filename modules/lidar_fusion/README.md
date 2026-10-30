# Dual-model LiDAR fusion

Independent CPU reference module joining the **existing** LiDAR detection and
semantic segmentation results with their original point cloud. It does not run
either neural network, change their configurations, or replace a production
perception/planning channel.

The six-layer design and approved first-delivery scope live in
[`ad_agents`](../../ad_agents/context/modules/perception/knowledge/dual-model-lidar-fusion-architecture.md).

## Baseline v1: design coverage

This baseline freezes the replay/shadow implementation, not production
acceptance. The original six-layer goals map to the implementation as follows:

| Layer or goal | Implemented | Remaining design or acceptance gap |
| --- | --- | --- |
| L1 alignment | Strict same-sweep identity, original point IDs, explicit rigid calibration and taxonomy validation | No asynchronous sweep compensation or guessed model remapping; producers must supply the same current sweep |
| L2 evidence | Preserved model evidence, independent local ground fit and explicit unsupported geometry | No upstream semantic entropy/model revision; complex terrain and physical ground accuracy remain unverified |
| L3 association | Core/margin connectivity, competing ownership and explicit support/conflict/insufficient states | Shared voxel graph is a CPU reference, not the proposed GPU association backend |
| L4 representations | Object candidates, independent measured/inferred/risk space, unknown relationships and read-only point views | Default recall uses axis-aligned enclosures; optional minimum-area rectangles are not L-shape fitting or guaranteed instance separation |
| L5 temporal feedback | Opt-in KF/Hungarian tracking, confirmation, history and current-point-supported recovery | Continuous production ODOM and tracking accuracy unverified; ground history does not refine the current fit |
| L6 consistency | Conservative source-layer, occupancy, reference, conflict and FREE checks | No validated planner adapter or closed-loop vehicle acceptance |
| Degradation | NORMAL, NO_SEG, NO_DET and GEO_ONLY, with bounded receive buffering | Real deployment channels, calibration and model qualification must still be supplied |
| Drivable area | Gated geometric traversability reference and diagnostic visibility | Real-record FREE remains disabled pending verified ray origins, ground support and vehicle contracts |
| Execution budget | Shared CPU topology/cells and conservative block queries | Serial CPU execution; no full point-evidence SoA or GPU backend; 5-10 ms target unmet |

Default behavior and opt-in quality-policy results are distinct baselines.
Quality policy, local object relations, robust ground and temporal processing
are not enabled implicitly. Full candidates, classification-use candidates and
measured occupancy must continue to be evaluated separately.

The checked-in [CPU workspace snapshot](../tools/perception_diagnostics/validation/cpu_workspace_v1.json)
records 51 frames / 204 four-mode payloads unchanged except latency, 65 C++
contract cases, two Cyber cases and 25 diagnostic Python cases. With identical
module `-O2` settings, three development-frame NORMAL medians decreased from
97.97-119.85 ms to 72.74-79.69 ms. These are fusion-only measurements, not
inference time, population P99 or a vehicle deadline guarantee. Earlier
negative quality baselines and subsequent opt-in results remain in the
[independent diagnostics](../tools/perception_diagnostics/README.md).

Large datasets, engines, replay payloads and viewer bundles are local evidence,
not part of this source baseline. Their fingerprints and locations are retained
in validation snapshots; reproducing them requires the stated external assets.
Existing producers and production configuration are unchanged.

Known review finding: with optional temporal feedback enabled, a confirmed
track's prediction can recover a second object from unowned outer points even
when a smaller current detection already represents that target. The two
observations can create duplicate tracks. The current recovery gate checks
point support, not whether that track already has a current observation.
This is a source-review finding, not a runtime-reproduced regression.
Temporal feedback remains disabled by default; this baseline does not accept
that path for deployment. Matching current observations before recovering only
unmatched tracks is required before accepting feedback behavior.

## Implemented path

```text
three Cyber readers -> bounded same-sweep buffer + steady-clock timeout
  -> L1 frame/index/calibration validation
  -> L2 immutable semantic/geometry/model evidence
  -> L3 connected core/margin association, competition and consistency states
  -> L4 detection/semantic-recall objects + occupancy + unknowns
  -> read-only point views / optional visibility and traversability
  -> optional L5 local-ODOM tracking, confirmation, history and feedback
  -> independent conservative output checks -> FusionOutput
```

`core/fusion.*` has no Cyber, CUDA or model-engine dependency. Output carries
original point IDs, raw labels/probabilities and detections, separate decisions
with source/reason, raw detector scores, conflict flags and per-source health.
Point-to-box competition resolves to one owner. Instance deduplication requires
both BEV IoU and shared point support; it does not modify measured occupancy.
Disconnected margin points remain independent. Ambiguous connected unknowns
retain object references rather than silently being absorbed.

`SingleFrameFusion` remains stateless. The optional `StatefulFusion` adapter
adds independent stable `track_id` fields; the original `id` and `object_ref`
remain frame-local. History and predictions never stand in for current points.

Candidate generation explicitly updates point decisions and produces a
`CandidateResult` containing only object and unknown candidates. Refinement
reads const point evidence and modifies only that result; it has no access to
health, raw detections or the space map. The orchestrator transfers candidates
back to the frame before spatial construction. Unknown risk rasterization
remains a spatial responsibility.

`SingleFrameFusion::Process` accepts independent `GroundPrior` and `ObjectPrior`
inputs. Only L2 receives ground history and only L3 receives predicted objects;
the temporal adapter owns construction of both. Neither prior can mutate the
other stage's input.

### CPU execution workspace

The six logical layers share frame-owned computation, not mutable evidence:

- The default connected-voxel path sorts point membership once and caches
  occupied-voxel adjacency. L3 and L4 recompute components with their own
  eligibility masks; removing owned points can split a component. Optional
  distance/semantic candidate connectivity retains its distinct graph semantics.
- Space construction, visibility and L6 share a touched-cell buffer with bounded
  dense slot lookup. Only touched cells become messages, in deterministic XY
  order, once at publication. Independent spatial/temporal checks retain their
  standalone entry points.
- Unknown/object relations use conservative actual-point bounds, lazy 128-point
  block bounds and existence queries rather than repeated full support scans.
  Boundary, nonunit-quaternion and extreme-coordinate cases retain exact
  fallback arithmetic. Evidence, occupancy and relation kinds are unchanged.
- Optional `FusionWork` reports eight stage intervals, voxel/edge counts and
  relation pair/point-transform counts without adding fields to output evidence.
  Profiling is off by default. Execution remains serial; no worker pool or GPU
  dependency is introduced.

`candidates.local_object_relations` defaults to false. When enabled, a candidate
does not inherit references from an entire scene-connected component. Direct
ownership, exact local relations and L6 shared-cell references still apply.
This is a separate behavior experiment, not an equivalent execution switch;
it neither enables semantic partitioning nor changes the ground estimator.

The CPU architecture regression and controlled comparison are recorded in
[`../tools/perception_diagnostics/validation/cpu_workspace_v1.json`](../tools/perception_diagnostics/validation/cpu_workspace_v1.json).
The benchmark optimizes fusion sources with `-O2`, keeping cached dependency
settings fixed; it is not a whole-stack release or model-inference benchmark.

## Input and calibration contract

Fill the empty fields in `conf/lidar_fusion.pb.txt`; this deployment template
deliberately fails initialization until configured. Supply all three input
channels, the common sensor frame and the actual versioned sensor-to-base
calibration. Translation is metres; rotation is radians, active
`Rz(yaw) * Ry(pitch) * Rx(roll)`. XYZ, full box orientation and optional velocity
are transformed consistently; output header frame is `base_link` by default.
No TF identity or model revision is guessed.

Both producers must consume the **same unfiltered point-cloud stream**.
Existing checked-in producer examples use different sensors and cannot simply
be launched together. A rigid extrinsic does not correct a wrong frame join.

Sweep identity uses matching nonzero copied `header.lidar_timestamp` and
noncontradictory sequence IDs. Without LiDAR timestamps, both sides need equal
sequence IDs and the copied header timestamp or current cloud measurement time
within 1 microsecond. This accommodates the current PointPillars header contract;
it is not approximate nearest-frame matching. A cloud needs a positive finite
measurement time and a provable header identity. Source frames/topics and model
taxonomies are independently validated.

Semantic labels may arrive in any order, but must cover each original point ID
exactly once with finite probabilities. Invalid XYZ keeps its original index and
raw semantic evidence but contributes no geometry. CenterPoint's accumulated
input count is **not** compared with current cloud size.
Class mappings are explicit: the template contains the current CenterPoint
nuScenes order and RangeRet SemanticKITTI learning labels. For PointPillars,
select its backend and replace detection mappings with KITTI
`0=CAR, 1=PEDESTRIAN, 2=CYCLIST`. Class changes must not relabel raw evidence.
Semantic entropy and segmentation model revision are absent upstream and are not
fabricated in this module.

The receive buffer accepts model results before or after their cloud. It releases
each cloud once when both results are available or `wait_ms` expires. Missing or
malformed sources select NORMAL / NO_SEG / NO_DET / GEO_ONLY. A monotonic output
frontier rejects duplicate/old clouds; restarting the component is required for a
record clock rewind. Bounded queue drops are visible in health. `latency_ms`
measures core compute only, not queue wait or neural inference.

## Conservative baseline and deliberate limitations

- Ground is a local geometric plane fit from noncollinear samples within
  `ground_seed_band` of configured base-frame `expected_ground_z`, constrained
  by slope and maximum residual. Unsupported ground stays explicitly invalid;
  its points conservatively occupy space. This flat/local-height baseline is
  not general robust terrain estimation and may lose support on grades, steps
  or poorly calibrated terrain. Confident semantic surface seeds can refine
  an independently accepted geometric tile, but cannot create new support.
  Evidence/cells expose slope, maximum residual and point-level seed count.
- Semantic recall defaults to connected voxels and axis-aligned enclosures.
  Optional `candidates` enable bounded range-adaptive connectivity, group/class
  separation, a one-degree minimum-area rectangle search and padded convex
  unknown hulls. Orientation remains unreliable; this is **not L-shape fitting**
  or a guarantee of separating touching same-class instances without detection
  seeds. Unknown/object relations explicitly distinguish duplicate, supplement,
  adjacent and ambiguous hypotheses without clearing occupancy.
- Spatial output is a sparse, single-frame 2D grid. Actual endpoints, geometric
  ground support, semantic group, measured occupancy, inferred occupancy and
  object references remain separate. Unknown candidate footprints additionally
  have an independent `occupancy_unknown` risk layer; they do not become
  detector evidence. Omitted cells are UNKNOWN. Boxes are
  conservatively rasterized by their projected AABB, clipped to the configured
  ROI. Discarding an instance cannot clear either occupancy layer.
- Optional `spatial.point_views` exports overlapping original point-ID lists
  for ground, non-ground, objects, structures, unknowns and corridor risk.
  Hiding background in a view never removes it from measured occupancy.
- Optional `spatial.trace_rays` uses a bounded CPU grid DDA and height-band
  masks; occupied/inferred/risk cells block the ray, and the region beyond
  stays unobserved. Bands are relative to the cell's ground estimate when
  supported. A compensated cloud without recoverable true ray origins must
  leave `verified_single_origin=false`: visibility is diagnostic only.
- Experimental single-frame traversability requires explicit length, width,
  height, ground clearance, margin, slope/step/roughness limits **and verified
  single-origin rays**; missing contracts reject enabling it. Full configured
  height-band coverage, ground support, and a conservative circumscribed
  footprint are required. Geometry and semantic policy remain separate:
  SURFACE permits FREE, SURFACE_SOFT permits FREE_SOFT, and boundary does not
  become FREE. Optional reachability starts at an actually FREE configured
  seed, never an assumed clear ego hole. This gridded reference is not a
  continuous-volume safety certificate or a planner command.
- `core/spatial.*` independently checks conflict flags, dangling references,
  object/unknown occupancy coverage and contradictory FREE. Conservative
  repairs preserve raw evidence and report reasons/counts. Any repair clears
  derived FREE/reachability rather than retaining stale clearance paths.
- GPU fusion and a planner adapter are not included. The 5-10 ms design target is not a
  measured result. Synthetic tests establish contracts, not real-world recall,
  free-space precision, braking behavior or vehicle readiness.

The baseline is for inspectable replay/shadow integration. Do not feed its
conservative occupancy to driving behavior without downstream validation.

## Optional temporal path

`temporal.enabled` defaults to false. Enabling it in the component additionally
requires explicit `pose_channel` and `odom_frame`. Publish this module's
`LocalPose`: current cloud payload time, continuous local-ODOM-to-base pose,
correct frame names, a reset epoch, valid flag and unit quaternion. Pose samples
are bounded and nearest-matched within `pose_tolerance`; no interpolation or
waiting for a late pose is implied. Missing/stale poses preserve single-frame
output but reset and disable temporal state for that frame.

The adapter reuses `wheelos_common`'s constant-velocity KF and Hungarian MOT.
Objects and unknowns use separate association kinds but a shared nonrecycled
track-ID allocator. Filtered physical velocity is on `track`, while raw detector
velocity/class/score remain unchanged. Class votes and sustained conflicts affect
only temporal identity. Unknown occupancy is immediate; stable confirmation
requires configured hits. Missed predictions are explicitly unobserved, with no
invented current occupancy. Only confirmed predictions supported by actual current
eligible points may recover an object during a detection miss. Recovery preserves
the observed classification role and conflict; geometry alone cannot upgrade an
identity-uncertain track. Conflict recovery retains weak point ownership and the
frame-level risk flag. Only new observations, not feedback, update class votes.

Optional bounded local-ODOM log-odds history uses measured occupancy and verified
FREE, never box-only inference. Unknown, unconfirmed and moving regions are
excluded/erased, including current measured footprints when the filtered box
lags. History expires. Ground history is exposed as a separate prior and
consistency feature; it cannot change the current ground fit or erase current
occupancy. Point-supported unknowns exclude their actual current and previous
support cells from history, not the unobserved interior of an extent polygon.
Missed tracks keep their last supported footprint until expiry.
Epoch, clock rewind/gap, diagnostic-mode change, excessive pose step,
invalid pose or tracker resource/numerical failure clears temporal state.
IDs are not recycled by a reset; a new initialized component starts a new ID
namespace. A Cyber record clock rewind still requires restarting the component's
monotonic receive buffer; a pose epoch alone does not reopen old cloud admission.

This is not an absolute-localization adapter or a validated production ODOM
source. Defaults and temporal noise/gates require scene-specific calibration.

## Build and verification

Inside the managed dev container as the mapped non-root user:

```sh
bazel test --config=dev --lockfile_mode=off \
  //modules/lidar_fusion:fusion_test \
  //modules/lidar_fusion:frame_buffer_test \
  //modules/lidar_fusion:spatial_test \
  //modules/lidar_fusion:candidates_test \
  //modules/lidar_fusion:temporal_test --test_output=errors
bazel build --config=dev --lockfile_mode=off \
  //modules/lidar_fusion:liblidar_fusion_component.so \
  //modules/lidar_fusion:component_test
```

The explicit/manual `:component_test` uses synthetic messages on actual Cyber
channels and verifies result-first arrival, publication and GEO_ONLY timeout
without attaching a late semantic result to the next sweep. Run its built binary
in the existing isolated test container, as the mapped user, with
`CYBER_PATH=/apollo/pkgs/core/cyber`, `APOLLO_ROOT_DIR=/apollo` and an isolated
`CYBER_DOMAIN_ID`. It does not execute either neural network or prove quality on
recorded data.

The temporal variant additionally publishes `LocalPose` over actual Cyber,
checks stable identity and current-point-supported recovery during the timeout.

### CPU-only cached replay

Build `//modules/lidar_fusion:replay_cached` in managed dev. No model engine,
CUDA runtime or inference is needed to reuse previously aligned model outputs:

```sh
bazel-bin/modules/lidar_fusion/replay_cached \
  /path/to/inputs /path/to/frozen-model-results /path/to/new-results 5
```

Inputs contain `fusion.pb.txt` and `*.cloud.pb`; model results contain matching
`*.cloud.pb.det.pb` and `*.cloud.pb.seg.pb`. Each frame/mode has one excluded
warm-up and the requested measured repeats. `timing.csv` retains every sample;
`summary.csv` uses the last measured sample, with model timing unavailable (-1).
Normalized `*.cloud.pb.{0,1,2,3}.pb` retain complete four-mode payloads except
`health.latency_ms`, and repeats must be byte-identical. Wrong-sweep rejection
and immutable input checks also execute. Optional final `--profile` additionally
writes `stages.csv`; use an unprofiled run for performance comparisons.

The normal `*.fusion.pb`, exact cached model copies, configuration and arguments
can feed the independent SemanticKITTI exporter directly. It still requires the
original prepared input manifest and schema. Existing results are never
overwritten; this runner is a diagnostic/benchmark tool, not a production
scheduler or a deadline guarantee.

After supplying a sensor-matched deployment configuration, the opt-in DAG is
`dag/lidar_fusion.dag`; it publishes one `FusionOutput` channel for all three
representations and diagnostic evidence. The DAG uses source-tree build paths;
adjust those paths for an installed package. The `:install` target packages the
plugin, configuration and DAG, not model files or a runnable vehicle setup.

## Recorded dual-model diagnostic

### Opt-in quality policy

`FusionConfig.quality` is an additive migration; all switches default to
false and no deployment is enabled. `separate_weak_detections` keeps low-score
CONFLICT results in immutable `hypothesis` with decision `RETAINED_WEAK`, rather
than letting them claim instance points. Measured occupancy and raw box inference
remain intact. Higher-score CONFLICT candidates stay in `object`, explicitly
marked `classification_use=IDENTITY_UNCERTAIN`; they are not classified objects.
`CLASSIFICATION_CANDIDATE` identifies a consumer role, not a calibrated probability.
Track-recovered and semantic-recall objects also carry that role.

`compatible_margin` requires compatible semantic evidence only for outside-box
margin ownership when segmentation is valid. Core geometric evidence and
high-score sparse detections remain available. `trusted_point_view` adds
`TRUSTED_OBJECT_POINTS` (kind 6) when point views are enabled: assigned THING
points agreeing with object class and confidence, excluding conflicted objects.
Old `OBJECT_POINTS` is unchanged in meaning and may include background.
The new view is empty without valid segmentation, not a claim that no target
exists. Foreground consumers must opt into this view rather than reinterpret
old serialized records.

`point_supported_unknown` marks unknown polygons as extent only. Spatial risk
and L6 consistency checks use actual unknown point cells rather than filling
their bounding extent; the gaps remain UNKNOWN, never automatically FREE.
Measured occupancy is unchanged. Extent-only polygons must not be rasterized
as solid obstacles by a consumer ignoring `point_supported_footprint`.
Existing unknown and spatial defaults preserve the old footprint behavior.

With `candidates.separate_semantic_instances=true`, semantic THING classes are
clustered apart from non-THING evidence and scene-wide background connectivity
alone does not suppress independent recall. Same-class touching objects still
require separate instance/temporal evidence; semantic class is not instance ID.
The option was rejected for the selected quality policy because it created too
many unvalidated instances. No inference GT, new model, threshold tuning or
runtime performance claim is part of this policy.

`quality.robust_ground` optionally retries rejected ground tiles using two
deterministic initial estimates and at most three inlier refits each.
`ground_inlier_ratio` defaults to 0.7 and must exceed 0.5. Existing seed-height,
minimum noncollinear support, slope and residual gates still apply. Semantic
refinement cannot move the independently accepted geometric plane beyond the
residual gate at tile seeds. Unsupported/competing surfaces remain unsupported;
there is no neighboring-tile or history-based ground extrapolation. This option
changes geometric evidence and therefore can change measured occupancy; it is
not a view-only filter and remains disabled by default.
On the current development diagnostics it raises semantic-ground recall but also
increases THING points classified as geometric ground. It is therefore an
experimental option, **not selected for the accepted quality policy**; object
match retention alone does not establish obstacle-point safety.

For cross-domain or otherwise unvalidated semantic identity, set
`quality.semantic_identity_unverified=true`. Raw labels, geometry and risk are
retained, but semantics cannot promote low-score detections, veto high-score
ones or create SEG_RECALL objects. Classification falls back to the original
detector threshold and the trusted semantic object view is empty. Source validity
and task qualification are different contracts; this flag is explicit, never
inferred from a dataset name. It does not validate semantic-ground accuracy.

The opt-in GPU `:replay` binary reuses the existing PointPillars and RangeRet
implementations. It does not duplicate their inference kernels or modify
producer deployments. The current record extraction tool uses the embedded
Cyber Record schemas (`cyber_record==0.1.13`), preserving cloud serialization,
point order, source timestamps, and recorded static calibration.

In the isolated **test** container with the dataset/model mounts and GPU:

```sh
PYTHONPATH=.cache/lidar-fusion-validation/record-reader \
python3 modules/lidar_fusion/tools/extract_record.py \
  --record /mnt/synology/apollo/sensor_rgb.record \
  --output /path/to/new-inputs \
  --topic /apollo/sensor/velodyne64/compensator/PointCloud2 \
  --samples 20 --base-frame base_link
```

`--base-frame` records an explicit operator assertion about the actual TF
parent, not a discovered identity transform. For this record the parent is
spelled **`novatel`**; the user identifies it as base_link. The tool records the
original spelling and quaternion. Cloud, localization and GNSS *payload*
timestamps share the old clock; record arrival and static TF use a different
clock. No temporal alignment is inferred from record timestamps.

Build `//modules/lidar_fusion:replay` with
`--config=dev --config=gpu --lockfile_mode=off` in managed dev. Then, in test:

```sh
bazel-bin/modules/lidar_fusion/replay \
  /path/to/new-inputs /path/to/new-results \
  /path/to/pointpillar.plan /path/to/rangeret.engine \
  0.00392156862745098 0.00392156862745098
python3 modules/tools/perception_diagnostics/summarize_replay.py \
  --input /path/to/new-inputs --results /path/to/new-results \
  --output /path/to/new-report.json
```

The two final arguments are explicit detector/segmenter intensity scales.
RangeRet's scale matches its checked-in configuration; PointPillars's
normalization on this recording is a **diagnostic assumption**, not verified
training equivalence. Expected ground z remains the baseline configured value
0, not an independently established vehicle ground reference.

The original cached PointPillars engine rejects the first record sample:
its 10,000-pillar capacity is insufficient. `tools/expand_pillars.py` can
derive a **local diagnostic** 40,000-pillar ONNX from the checksum-pinned
NVIDIA export without changing weights/nodes. It requires `onnx==1.17.0`.
Build a new engine with the existing detector's `tools/run_pointpillars.py
--build-onnx`, on the target GPU; never overwrite the existing NAS artifacts.
The source already fails ONNX's strict checker on a malformed `keepdims`
attribute; the derivative preserves that exact diagnostic, while TensorRT
build/runtime establish the limited executable contract. A cached KITTI
fixture produced **9 versus 10 predictions** with the original/expanded
engines, including class/score differences: they are **not numerically
equivalent models**, and no accuracy equivalence is claimed.

The replay writes original detection/segmentation protobufs, NORMAL fusion
protobufs, a CSV for all four modes, frozen arguments and configuration. It checks
input immutability, point mapping, source health, occupied/free consistency,
conflict flags and deliberate wrong-sweep rejection. Outputs refuse overwrite.
Visualization and independent evaluation now belong to
[`perception_diagnostics`](../tools/perception_diagnostics/README.md); the inference
runner no longer renders graphics. Historical baseline SVGs remain unchanged.
The summarizer verifies complete sample/mode coverage and freezes source,
input, executable, engines and result hashes.

Recorded validation covers **20 uniformly selected clouds out of 587**,
including first/last; it is not a full-record accuracy evaluation. These have
actual dual-model inference, four-mode fault injections and wrong-sweep tests.
There are no labels or downstream closed-loop results. The recording lacks
verified per-ray origins and vehicle capability parameters: **zero FREE is
required**, not a missing obstacle-free-area demonstration. The CPU reference
with diagnostic rays does not meet the original 5-10 ms target. See the
implementation plan in `ad_agents` for unresolved stage gates.

### Cached-model temporal replay

`tools/diagnostic_poses.py` requires
`--allow-absolute-pose-diagnostic` to derive `T_first^-1 * T_current` from the
recorded localization. Every pose is explicitly `diagnostic_only`; this does
**not** establish continuous ODOM. Sparse-sample dt and pose-step limits are
derived and frozen for diagnostic execution, not deployed as sensor defaults.
The CPU-only `:replay_fusion` reuses immutable original clouds and already saved
actual-model outputs, avoiding another neural-inference run:

```sh
PYTHONPATH=.cache/lidar-fusion-validation/record-reader \
python3 modules/lidar_fusion/tools/diagnostic_poses.py \
  --input /path/to/inputs --output /path/to/new-diagnostic-poses \
  --allow-absolute-pose-diagnostic
bazel-bin/modules/lidar_fusion/replay_fusion \
  /path/to/inputs /path/to/model-results /path/to/new-diagnostic-poses \
  /path/to/new-temporal-results
```

It compares stateful processing against a stateless same-config reference,
checks current measured occupancy and input immutability, and runs an independent
detection-dropout sequence. Per-frame protobufs, CSV, summary and configuration
are retained. Stable association/confirmation must actually trigger; recovery
counts must be read from the result, not inferred from enabling feedback.
These sparse, unlabeled diagnostic associations do not measure MOT accuracy.

### Evaluation baseline

The frozen record baseline is
[`validation/sensor_rgb_baseline_v1.json`](validation/sensor_rgb_baseline_v1.json).
It covers all 587 clouds for single-frame modes and a preselected continuous
60-frame temporal comparison. It is an **unlabeled operational baseline**, not
accuracy acceptance. In particular, class conflicts are frequent, enhanced
candidates increase unknown fragmentation, dropout recovery preserves only some
instances, and every diagnostic ray is blocked at the unknown-risk-covered
sensor-origin cell. Thus zero FREE does not demonstrate useful free-space
reconstruction. The snapshot links the full hashed local evidence.

For a full-record baseline, extract `--samples 587` for this recording and run
the four-mode neural replay above without changing the model/configuration.
Select a consecutive temporal subset **before** inspecting results. The temporal
runner now saves `.single.pb` as the same-candidate-config, no-history reference,
alongside clean `.temporal.pb` and detector-dropout `.dropout.pb` outputs.
Its CSV records all three fusion times and object/unknown counts.

`../tools/perception_diagnostics/evaluate_baseline.py` audits saved protobufs independently of the fusion
implementation: raw semantic/detection preservation, current-point-derived
measured occupancy, references, conflicts, no unverified FREE, complete mode
coverage and identical measured occupancy across temporal/dropout variants.
It exports per-frame detail CSVs, distributions, source/artifact hashes and
explicitly unavailable accuracy metrics. Generate descriptors from the source
schemas in managed dev, then run the evaluator in the existing Python environment:

```sh
protoc -I . -I pkgs/wheelos_msgs --include_imports \
  --descriptor_set_out=/path/to/schema.pb \
  modules/lidar_fusion/proto/lidar_fusion.proto \
  modules/lidar_semantic_segmentation/proto/lidar_semantic_segmentation.proto \
  pkgs/wheelos_msgs/wheelos_msgs/sensor_msgs/pointcloud.proto
python3 modules/tools/perception_diagnostics/evaluate_baseline.py \
  --input /path/to/full-inputs --results /path/to/full-results \
  --temporal-input /path/to/consecutive-inputs \
  --temporal-results /path/to/temporal-results \
  --schema /path/to/schema.pb --output /path/to/new-report
python3 -m unittest discover -s modules/tools/perception_diagnostics \
  -p evaluate_baseline_test.py -v
```

### nuScenes labeled subset

`tools/prepare_nuscenes.py` adapts original current LiDAR sweeps, official JSON
box annotations and exact-token cached CenterPoint runtime outputs. It verifies
the official LiDAR-to-ego calibration, ego pose and source identity; truth is
written separately and is never read by fusion. It does not unpickle metadata.
The default bounded experiment selects three validation scenes and four leading
keyframes per scene before evaluating results. The source ring field is not
silently treated as point time. Point IDs/order remain those of the original
single sweep; the detector's accumulated ten-sweep point count stays separate.

The replay optionally accepts an eighth argument, `CACHED_DETECTION_DIR`, after
the two intensity scales. In this explicit mode the PointPillars engine argument
is ignored (`-`), detector inference is skipped, same-frame detection protobufs
are loaded, and `det_ms=-1` denotes **not measured**, not instantaneous inference:

```sh
bazel-bin/modules/lidar_fusion/replay \
  /path/to/scene/inputs /path/to/scene/new-results \
  - /path/to/rangeret.engine 1 0.00392156862745098 \
  /path/to/scene/detections
```

RangeRet still runs freshly on every original sweep. Its SemanticKITTI training
and 64-beam projection differ from nuScenes' 32-beam data; semantic accuracy is
not established. The independent diagnostics module exports the labeled
center-distance comparison, interactive offline viewer and camera context.
No lidarseg labels, official mAP/NDS, temporal accuracy or validated FREE are
claimed.

Counts describe object/unknown **frame outputs**, not unique true obstacles.
Track continuation and confirmation are not IDF1/MOTA or identity-switch
measurements. AP, recall, false-positive rates, velocity error and FREE precision
require suitable labels/contracts; unlabeled self-consistency cannot replace
them. Timing includes the first inference but excludes engine initialization,
file serialization and Cyber queueing. Concurrent workloads on the shared host
must be recorded; these offline timings are not production throughput.
