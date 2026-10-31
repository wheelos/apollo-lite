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

## Current capabilities and delivery boundaries

This README is the merged current module guide. Historical backup content was
merged here; backups are not a second authoritative capability list. Source,
configuration and executed evidence take precedence over documentation.

| Capability | Current implementation | Availability / boundary |
| --- | --- | --- |
| Same-sweep alignment and calibration | C++ `core/alignment/`, `core/evidence/` | Validated frame/index/extrinsic/taxonomy joins; no asynchronous sweep compensation |
| Detection and semantic fusion | C++ `core/association/`, `core/runtime/` | Association, current-point support, conflict ledger, semantic recall and independent unknowns; optional refinements require explicit configuration |
| Occupancy and unknown risk | C++ `core/spatial/` | Independent measured, inferred-object and unknown source layers; filtered point views never clear occupancy |
| Native ground and gated traversability | Existing C++ fusion/spatial path | Local-plane ground and visibility/vehicle-gated spatial reference; not general terrain or accepted real-record FREE |
| Observed ground, slope, roughness, step and depression depth | Independent C++ `core/terrain/surface*` plus Python reference | Standalone numerical implementation checked against 56 real frames; relative measurements, not physical pit/bridge identification or deployed vehicle limits |
| Risk-constrained terrain candidate detection | C++ `core/terrain/traversability.*` plus Python reference | Explicit limits, UNKNOWN handling and same-root components; standalone offline path, not production FREE or verified ego reachability |
| Clearance and static trajectory footprint risk | Python `spatial_support/{clearance,trajectory,passage}.py` | Offline closed-cell distances, swept disk/rectangle probes and conservative turn envelopes; no real planner trajectory qualification |
| Cost and narrow-passage layers | Python `spatial_support/{cost,passage}.py` | Independent terrain/source/availability layers; no planner decision or steering feasibility guarantee |
| Tracking and short-term memory | C++ `core/tracking/` | Opt-in KF/Hungarian, confirmation, current-point-supported recovery and history; continuous production ODOM/MOT accuracy unqualified |
| Visibility and occlusion | C++ `core/spatial/visibility.cc` | Opt-in ray reference; actual per-return origins and vehicle observation contracts required for FREE |
| Degradation and consistency | C++ core/component | NORMAL / NO_SEG / NO_DET / GEO_ONLY, health, bounded buffering, reference/occupancy/FREE checks; no slowing/braking controller |
| Publication | C++ component and `publication/pod.*` | Full/functional protobuf profiles and explicit CORE_POD transport; consumer and recorder migrations remain separate |
| Debug visualization | Independent `terrain_debug.py` and diagnostic viewer | BEV, occupancy, terrain, risk, connectivity and coverage images under local `data/`; rendering never changes numerical outputs |

The terrain v6 experiment executed 56 real-input frames and 44 post-fit
SemanticKITTI label checks. Its frozen 34-case matrix contains 28
mechanistic/counterexample passes, five diagnostic-only boundaries and one
physical-evidence block. Only one of the 36 native-risk frames retained a
risk-constrained root component (17 cells / 4.25 square metres). Therefore
working candidate detection is **not** effective around-vehicle traversability
acceptance. Sparse map omissions, multi-layer ambiguity and unknown risk remain
explicit; semantic ground overlap cannot authorize deleting risk. These results
are a development diagnostic, not independent holdout accuracy.

## Spatial capability delivery priorities

The complete P0-P3 design is in
[`ad_agents/context`](../../ad_agents/context/modules/perception/knowledge/lidar-fusion-driving-capabilities-design.md).
The priorities below distinguish source delivery from deployment acceptance.

| Priority | Responsibility | Delivery boundary |
| --- | --- | --- |
| P0 | Preserve obstacles, independent occupancy, unknowns and source health | Existing native core evidence; never replace risk with filtered points |
| P1 | Continuous observed ground and terrain measurements | Local observed root, raw layers, full-rank slope/roughness, step/depression diagnostics; vehicle-ground and physical accuracy unqualified |
| P1 | Clearance and footprint risk queries | Offline static probes with explicit terrain, coverage, visibility and ROI unknowns |
| P2 | Cost layers and narrow-passage diagnostics | Numerical layers and directional rectangle queries implemented; planner/vehicle integration unqualified |
| P2 | Dynamic tracks and local-ODOM memory | Native opt-in tracking/history; continuous ODOM and tracking accuracy still required |
| P3 | Verified visibility | Actual origin and height-column observations required before FREE |
| P3 | Time-indexed occupancy and collision prediction | Not implemented by static probes; needs qualified tracking, timing, uncertainty and actual trajectories |
| P3 | Slowing/braking/degradation policy | Fusion supplies evidence; planner/safety owners must implement and validate actions |

### Directory ownership and data flow

```text
modules/lidar_fusion/
  core/alignment/                     sweep identity, validation, buffering
  core/evidence/                      transforms, ground fit, point evidence
  core/association/                   detection/prior association, candidates
  core/spatial/                       occupancy, connectivity, views, rays
  core/terrain/                       observed layers, measurements, limits
  core/tracking/                      tracker adapter, history, temporal state
  core/runtime/                       single-frame orchestration
  core/*.h                            legacy public include forwarding only
  proto/lidar_fusion.proto              source/evidence contracts
  component/                           scheduling and publication boundary
  publication/pod.*                    explicit borrowed transport codec
  publication/functional_scene.*       functional-scene projection
  tools/                               inference/cached replay and probes
  tools/terrain_diagnostics.cc          standalone numerical text replay
modules/tools/perception_diagnostics/
  ground_{experiment,continuity,patch}.py
  terrain_surface.py                   raw observed layers and measurements
  terrain_traversability.py            limits, risk and same-root components
  terrain_traversability_experiment.py  frozen real-input validation
  terrain_debug.py                     BEV/occupancy image export
  terrain_native_validation.py         C++ original-XYZ/reference comparison
  spatial_support/
    adapter.py                         immutable full-protobuf adaptation
    terrain.py                         native-grid height/terrain proxy
    observed_terrain.py                 separate raw measurement diagnostics
    clearance.py                       occupied-cell-square distances
    trajectory.py                      swept-disk path samples
    cost.py                            independent cost/availability layers
    passage.py                         rectangle and corner envelopes
```

```text
exact current-sweep XYZ / original IDs + immutable native risk + source hashes
  -> full-evidence and identity audit (truth remains evaluation-only)
  -> observed ground layers and local root
  -> slope / roughness / boundary step / relative depression measurements
  -> explicit diagnostic terrain limits
  -> terrain candidates | independent risk and full-cell coverage vetoes
  -> separate same-root four-connected regions, without gap filling
  -> NPZ / JSON evidence -> BEV and occupancy debug images

full native FusionOutput + config + frozen ground NPZ
  -> one-way spatial_support adapter
  -> observed terrain + unchanged inferred/unknown risk layers
  -> closed-cell-square distances and explicit static footprint probes
  -> cost / passage / sampled risk diagnostics (not planner commands)
```

These stages are stateless in their declared frame-local XY/Z coordinates,
in metres. Already transformed native evidence must not be transformed twice.
Temporal accumulation remains owned by `core/tracking/` and continuous local ODOM,
not the frame-local numerical experiment. No absolute pose is substituted.

The older `spatial_support.terrain` uses fitted-ground medians and maximum
local plane residual as a proxy. The newer observed surface estimator fits
selected raw cell elevations and reports sample residual standard deviation.
These are different statistics, neither calibrated physical roughness.
Missing/collinear support remains unavailable rather than a guessed slope.

Clearance uses distances to **closed cell squares**, not centers or amodal
object solids. Disk probes cover every heading conservatively; rectangle
probes retain segment headings and inflate actual turns. Translation samples
are no farther apart than half a cell and include half-gap inflation.
`first_risk_arc_m` is the first flagged sample, not exact contact or braking
distance. Path outputs retain `OBSTACLE_RISK`, `TERRAIN_LIMIT`, `UNKNOWN` and
`SUPPORTED_ONLY` reasons. Even `SUPPORTED_ONLY` does not authorize driving;
candidate ground cannot inherit baseline FREE. Grid/query budgets and shape
errors fail visibly, not as empty-safe results.

### Validation and promotion

The initial spatial-support slice used 36 frozen ground-comparison frames with
illustrative vehicle dimensions and paths. Its analytic checks cover slopes,
steps, holes, sparse rank, layered distances, ROI boundaries and swept
footprints. Later terrain work expanded to 56 original point-cloud inputs.
The earlier radial comparator's six connectivity regressions and unsafe
sparse-filter ablation remain historical evidence, not erased successes or
proof of the newer algorithm's physical accuracy.

Promotion still requires applicable terrain/vehicle limits, verified
visibility, actual planner trajectories and downstream integration regression.
Dynamic prediction and braking are separate missing capabilities, not implied
by static cost/path diagnostics. Real-data consistency and post-fit semantic
metrics cannot substitute for physical traversability truth.

### Observed terrain numerical modules

`core/terrain/surface.h` and its stage-owned translation units own raw
original-ID height layers, local surface
initialization/propagation and observed slope, roughness, boundary/span and
relative depression measurements. `core/terrain/traversability.{h,cc}` consumes
that result with caller-supplied limits and independent risk/coverage grids.
Their typed result/status interfaces are independent of Cyber, CUDA and neural
inference. `:terrain_surface`, `:terrain_traversability`, `:terrain_test` and
`:terrain_diagnostics` are distinct Bazel targets. The new libraries are not
implicitly called by `SingleFrameFusion`; existing ground/default outputs stay
unchanged.

The standalone CLI accepts `--input PATH --output PATH`; `--help` documents
versioned `TERRAIN_V1` config/limits, original-ID XYZ rows and full risk grid.
Output includes cell measurements/states/reasons, raw layer/source membership,
original point rows, selected root and explicit missing metrics. It consumes
raw XYZ, not Python-fitted terrain features.

The managed-dev standalone binary was compared against all 56 frozen real
inputs: point identities/classification, raw layers/source IDs, selected roots,
cell states, risk and connectivity agree exactly; numeric measurements agree
within relative `1e-8` / absolute `1e-9` tolerances. This is numerical-port
evidence, not physical ground accuracy or effective risk-covered vehicle access.
The normal Bazel graph remains blocked by missing hardware checkouts/registry
modules; the direct managed C++ build is not a Bazel or component-build pass.
Neither the lockfile nor dependency graph was rewritten to bypass that failure.
The managed standalone validation also runs eight GoogleTests and 15 malformed
protocol cases against the new C++ libraries/CLI; it does not cover an unbuilt
component or bypass the existing production regression targets.

### Refactored implementation boundaries

The single root Bazel package retains all existing public target names.
Seven legacy `core/*.h` entry points forward to canonical functional headers;
new module callers use the canonical paths. Schemas, channels, configuration,
class signatures and installation paths are unchanged.

`core/runtime/single_frame_fusion.cc` only coordinates existing stages.
Alignment, evidence, detection/prior association, candidate construction,
occupancy and publication projection each have their own owner. Spatial ray
tracing is separate from cell storage and output checks. Terrain layer selection,
plane fitting and measurements are separate from risk classification. Tracking
adapters and history remain separate from the temporal state owner.

Core implementation files now top out at 392 lines, compared with the previous
1,695-line fusion and 902-line terrain files. Tests follow feature ownership;
the original 44 fusion cases remain under the same `:fusion_test` target.
Coherent tools and fixtures are not split solely to meet an arbitrary line cap.

Fresh managed direct builds pass 44 fusion, 18 candidate, 10 spatial, 13 temporal
and four frame-buffer cases. Eight terrain cases, 15 CLI rejection cases and
56 frozen real-frame comparisons also pass. Current implementation/test
translation units and the compatibility headers were compiler-checked.
These checks do not clear the official Bazel/component graph blocker above.
Refactor evidence lives in
`.cache/lidar-fusion-validation/refactor/implementation-report.json`;
new parity outputs use its `terrain-parity/` directory, leaving historical
debug manifests unchanged.

```sh
python3 modules/tools/perception_diagnostics/terrain_native_validation.py \
  --binary /apollo/.cache/lidar-fusion-validation/terrain-cpp/terrain_diagnostics \
  --output data/lidar_fusion_debug/cpp
```

`data/lidar_fusion_debug/index.html` links both Python-reference and actual C++
BEV/occupancy outputs. Native text inputs/outputs, NPZs and
`cpp/native-validation.json` retain source/binary hashes, field comparisons and
descriptive 5/10/20-metre origin-centered coverage windows. These windows are
diagnostic summaries, not vehicle qualification limits.

Observed terrain and diagnostic traversability are separate numerical stages:
raw point identities and height layers belong to the surface estimator;
explicit terrain limits, independent risk and exact-root connectivity belong
to the traversability classifier. The offline validation path must retain
UNKNOWN and missing-evidence states and cannot inherit production FREE
authorization. A selected surface is not verified vehicle ground, and a
connected observed-root component is not ego access across the blind zone.

Debug views from the frozen real-data comparison are generated by
[`terrain_debug.py`](../tools/perception_diagnostics/terrain_debug.py).
The local entry point is `data/lidar_fusion_debug/python/index.html`, with
per-frame raw BEV, source occupancy, terrain, risk decisions, connectivity
and coverage PNGs. These files are ignored generated data; their manifest
records numerical archive and image hashes. Rendering never modifies fusion
evidence, filters risk points or runs a planner.

The optional temporal duplicate-recovery finding was reproduced and fixed:
`AssociatePrior` now gives sufficiently supported current candidates precedence
over recovery from their unowned outer returns. The regression
`SmallerCurrentDetectionDoesNotRecoverDuplicateOuterSupport` retains one object
track and independent outer evidence. This conservative gate can suppress
ambiguous nearby recovery; it does not clear risk. Feedback remains disabled
by default and still requires continuous ODOM and tracking-accuracy evidence.

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

The `core/runtime/` fusion orchestrator has no Cyber, CUDA or model-engine
dependency. Output carries
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

Two additional candidate experiments also default to false:
`candidates.recall_confident_foreground` recalls connected, confident same-class
subsets of retained mixed unknowns only when their fraction reaches `support_ratio`
and the existing recall limits hold. Connectivity is recomputed after removing
background; the entire mixed unknown and its risk remain. It uses the default
voxel graph and rejects combinations with separate/far candidate connectivity.
`candidates.extend_detection_support` extends only supported, classification-use
DET candidates using their own confident same-class assigned points, anchored
by at least `core_min_points` such points inside the original box. Center and
orientation are fixed: one-sided visible support cannot relocate an amodal
detector hypothesis. No face moves beyond the original `association_margin`; raw
hypotheses are unchanged. The extension has an explicit reason and contributes
only additional inferred occupancy, never erasing measured occupancy. Neither
experiment uses semantically unverified identity or establishes amodal box
accuracy.

Extension also checks all local evidence in the newly enclosed shell, including
unassigned points. It rejects a proposal when confident static/vegetation votes
reach the existing `conflict_ratio` among those votes and confident same-class
THING support. Geometric ground, out-of-height-band points and low-confidence
semantics do not vote. This reuses the frame's spatial index; it does not scan
the whole cloud per production candidate or change the original detector box.

The CPU architecture regression and controlled comparison are recorded in
[`../tools/perception_diagnostics/validation/cpu_workspace_v1.json`](../tools/perception_diagnostics/validation/cpu_workspace_v1.json).
The benchmark optimizes fusion sources with `-O2`, keeping cached dependency
settings fixed; it is not a whole-stack release or model-inference benchmark.
For timing budgets, verify generated protobuf and runtime dependency flags too:
core-only `-O2` with fastbuild dependencies is a distinct baseline from
`--config=opt` (`-c opt` for precise targets). A build-mode improvement is not
an algorithm or source-code speedup.

Legacy configurations hand the complete `FusionOutput` to the Cyber writer.
The explicit functional publication mode below separates the dense trace.
Core computation, projection, message sizing/serialization and actual
queue/transport costs are separate measurement boundaries; an isolated
serialization result is not production end-to-end latency.

### Functional publication

`publication.detail` defaults to `FULL_EVIDENCE` for existing configurations.
The unconfigured deployment template explicitly selects `FUNCTIONAL_SCENE`;
configure/migrate its consumers before using it. The core and optional temporal
adapter still compute and check the complete evidence frame. Projection happens
only at the component publication boundary, not inside fusion decisions.

| Function | Published scene fields |
| --- | --- |
| Object fusion | `object`, complete `hypothesis`, association semantic ledger, raw detector hypotheses/scores and conflict/classification roles |
| Unknown and topology | `unknown`, relations and enabled `shadow_partition`; partitions remain support subsets, not verified instances |
| Spatial risk and gated traversability | Complete `space_map`, including independent measured, inferred and unknown risk and all existing FREE gates |
| Background/foreground filtering | Enabled `point_view` original-ID lists; filtering a view never clears occupancy |
| Temporal and degradation | `track`, `history_cell`, `odom_frame`, `health`, `risk_event` and frame/calibration/model identity |

Only the dense `point_evidence` and `point_decision` arrays are omitted from
`FUNCTIONAL_SCENE`. `publication` explicitly identifies the profile and includes
the original cloud header, measurement time, channel, point count and exact
sensor-to-base transform. Every point ID in objects, unknowns, partitions,
semantic support and views indexes that **same unfiltered source sweep**.
Consumers needing positions must join that exact cloud and apply the supplied
transform; an unavailable/stale cloud is not empty space. Published positions,
boxes and cells otherwise keep their existing base-frame contracts.
Configured spatial options and semantic-partition availability remain explicit.
No new view, partition, FREE or temporal feature is implicitly enabled.

Set `publication.diagnostic_channel` to a distinct channel to retain the complete
frame alongside the functional scene. The diagnostic writer sends only while
a reader is present; its failure does not invalidate an already published scene.
Without that explicit diagnostic subscription, the omitted trace is not retained
for later retrieval. It is intentional output detail, **not SOURCE_MISSING**.
Do not run full-frame `CheckOutput` or dense GT exporters on a functional scene;
checks run before projection, and exporters require the full diagnostic channel
or unchanged complete cached replay outputs. Omitted profile metadata means the
legacy complete contract, never a new compact payload.

The manual `//modules/lidar_fusion:publication_probe` target measures a real
writer and callback in two separate processes using an already saved full or
functional `FusionOutput`. In the managed test runtime, set `CYBER_IP=127.0.0.1`
and an isolated `CYBER_DOMAIN_ID`, create a new empty receipt directory, then run:

```text
publication_probe receive EXPECTED_PB /test/lidar_fusion_publication/CASE RECEIPTS COUNT
publication_probe send EXPECTED_PB /test/lidar_fusion_publication/CASE RECEIPTS WARMUPS REPEATS NEW_CSV
```

`COUNT` must equal `WARMUPS + REPEATS`. Discovery and each sample are bounded.
An untimed same-channel bootstrap marker confirms a real data callback: topology
discovery alone does not establish a matched volatile RTPS endpoint. Setup markers
are separate from the exact frames; timed publications are never retried.
The receiver verifies complete bytes before acknowledging. CSV separates writer
wall/CPU time and publication-to-callback time; receipt polling is outside the
callback interval. This excludes fusion, input queues and model inference.
Transport depends on the resolved Cyber SDK and runtime configuration: ordinary
protobufs fall back from configured cross-process ICEORYX to RTPS in the current
SDK. Do not infer zero-copy or shared-memory delivery from configuration alone.

### Core POD publication

`publication.transport` is independent of output detail. Absent transport keeps
the legacy `PROTOBUF` writer and message type. The unconfigured, fail-closed
template explicitly selects `CORE_POD` on a separate `/pod` topic; both its
primary output and optional diagnostic channel use `PodMessage`. Existing
`Reader<FusionOutput>` consumers must not subscribe to those topics.

The adapter calls core `Writer<PodMessage>::Loan` and serializes the original
`FusionOutput` protobuf directly into the borrowed memory before moving each
loan into `Publish`. It never constructs an owned `PodMessage` or falls back to
copied `Write`/RTPS. This is **transport zero-copy**, not a typed end-to-end
zero-copy scene: protobuf encoding, decoding and fusion allocation/release
remain. The functional projection and all fusion policies are unchanged.

The current core chunk capacity is `8 MiB - 256` bytes, including the POD
header. A functional scene typically fits one chunk; complete roughly 10 MB
diagnostics require two. The adapter streams across canonical fragments without
an intermediate full-frame string. All loans are acquired before any fragment
is submitted. `max_pod_frame_bytes` defaults to 64 MiB and bounds accepted frame
size, not available memory: the SDK's outstanding-loan limit, pool pressure or
held consumer samples can still reject a smaller frame.

The wire contract uses the existing version-1 `PodChunkHeader`, payload kind
`UNKNOWN` (not an image/point-cloud layout), and application discriminator
`schema_hash = 0x46555331` (`FUS1`, not a checksum). `timestamp_ns` preserves the
original LiDAR timestamp. `frame_id` is the publisher's monotonic wire-message
ID, not the source sequence. `reserved[0..3]` carry publisher epoch, total
protobuf bytes, fragment byte offset, and packed `(count << 32) | index`.
The original source/frame/calibration metadata remains in the protobuf payload.

Consumers use `ConfigureFusionPodReader` to provision the local pending queue
and ICEORYX subscriber depth for a complete bounded frame, then pass each
`shared_ptr<PodMessage>` to a `FusionPodAssembler` owned by that reader. The
assembler retains borrowed owners and parses their views through protobuf
streams without a reassembly copy. Do not copy `PodMessage` by value: core
deep-copies even a borrowed message. One ordered publisher stream is supported
per assembler; restart epochs are explicit. Release/reset the assembler and
reader when stopping consumption.

Only `COMPLETE` delivers a decoded frame. Invalid schema, sizes, timestamps,
reordered/missing fragments and byte-budget violations never emit a partial or
default empty scene. A new complete frame can replace an incomplete one;
consumers must still monitor freshness rather than interpret missing delivery
as free space. A writer result is submission, not callback acknowledgement.
With no readers, publication explicitly skips encoding/loans. Same-process
INTRA readers or mixed/remote routes cannot use this loan path and fail visibly;
use an explicit protobuf configuration where those routes are required.

Recording is also a consumer migration. The current generic `cyber_recorder`
constructs `RawMessage` readers, so its POD capture/replay path is **not
qualified** here. Use a POD-aware decoder/retention consumer or the legacy
protobuf path; do not feed a saved POD chunk directly to a dense protobuf/GT
exporter. No existing records or SDK configuration are rewritten.

Append `--core-pod` to **both** publication-probe commands above to require real
borrowed callbacks. CSV additionally separates sizing, loans, direct encoding,
publish and decoded-ready callback time. POD callback entry alone precedes
protobuf decoding and is not comparable to a decoded protobuf callback.
The probe retains chunks across subsequent publications and verifies them after
reader shutdown/release. Optional sender `--pool-pressure` exercises a
multi-fragment frame with only enough outstanding-loan capacity for its prefix:
the whole submission must reject without exposing a fragment, then recover
after releasing the held loans. It characterizes loan-resource pressure, not
the total global mempool capacity.

For offline codec regression only, `publication_probe codec-matrix PATHS_TXT
NEW_CSV` checks every listed complete protobuf twice through heap-backed borrowed
fragment views. This does not initialize Cyber or claim a runtime zero-copy
test; the separate two-process probe provides that evidence.

### Frame allocation and ownership

The component allocates each published protobuf on its own frame-owned Arena.
Aliasing `shared_ptr` ownership retains that Arena until the last writer/reader
reference is released, including after the processing callback or component
has stopped. Complete diagnostics and functional scenes have independent
owners: retaining a functional scene does not pin its omitted dense trace.
There is no cross-frame Arena reset/reuse while consumers hold a message.

Core processing and projection construct their temporary message on the
destination's Arena to avoid a hidden cross-Arena deep copy. Ordinary heap
callers remain supported. This changes allocation and release, not evidence,
fusion decisions, field availability or the transport codec.

### Evidence-preserving diagnostics

`hypothesis.association.semantic_evidence` describes the original eligible
**core**, excluding margin and final candidate membership. It distinguishes
source availability, qualified semantic identity, confident coarse-group counts,
confident known-THING point-ID subsets, low-confidence points and unavailable
semantics. Raw subsets remain inspectable when identity is unqualified; they do
not become qualified votes. Counts and top-1 confidence are not calibrated
existence probabilities.

`cross_thing_disagreement` requires qualified identity, `core_min_points`, a
known THING detection, and one different known THING class reaching the existing
`conflict_ratio` over all core points. Different alternative classes are not
pooled. A false flag does not exclude weak or mixed opposing evidence. This
additive description does **not** change legacy `consistency`, `class_conflict`,
candidate admission, classification-use role, ownership or geometry. Both the
raw detector box and exact semantic support subsets remain available; neither
source is declared correct.

`candidates.shadow_semantic_partitions` defaults to false. With valid, qualified
segmentation, it adds read-only `shadow_partition` records for retained unknown
parents containing confident known-THING support. Known classes and confident
non-generic coarse groups are clustered separately using the existing
`CandidateComponents` L-infinity distance test at `connectivity_size`, without
far-range expansion. Low-confidence/generic points and geometrically ineligible
seeds remain in an unexplained remainder. Per parent, disjoint child point-ID
sets exactly cover its support set; the original parent is retained unchanged.
There is no boundary growth, minimum-instance-size claim, fitted box, heading,
score, object promotion or spatial-risk reduction. Touching same-class instances
may still be merged and partial surfaces may fragment. These are semantic
support partitions, **not verified instances**.

Clustering uses original-point index views rather than copying protobuf points.
Verified voxel cliques skip redundant comparisons only when their actual bounds
and semantic compatibility guarantee connectivity. Neighbor lookup and union
order stay deterministic; all remaining edges retain the exact distance test.
This changes neither the connectivity radius nor the partitioning policy.

All additions coexist with old serialized payloads. Old consumers may ignore
the new fields but must not assume that an old payload computed these
diagnostics. Shadow output is built after spatial checks and never enters
candidate or occupancy construction; its compute is included in latency and the
optional output-stage timing. No production switch or downstream driving policy
is enabled by this diagnostic work.

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
- `core/spatial/` independently checks conflict flags, dangling references,
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
and immutable input checks also execute. Optional `--profile` additionally writes
`stages.csv`; use an unprofiled run for performance comparisons.
`--shadow-partitions` explicitly enables read-only semantic partitions in the
saved run configuration. Both flags may follow the optional repeat count:

```sh
bazel-bin/modules/lidar_fusion/replay_cached \
  /path/to/inputs /path/to/frozen-model-results /path/to/new-shadow-results \
  1 --profile --shadow-partitions
```

The normal `*.fusion.pb`, exact cached model copies, configuration and arguments
can feed the independent SemanticKITTI exporter directly. It still requires the
original prepared input manifest and schema. Existing results are never
overwritten; this runner is a diagnostic/benchmark tool, not a production
scheduler or a deadline guarantee.

After supplying a sensor-matched deployment configuration, the opt-in DAG is
`dag/lidar_fusion.dag`; it publishes the configured `FusionOutput` scene profile
and optionally a separate complete diagnostic channel. The DAG uses source-tree build paths;
adjust those paths for an installed package. The official `:install_dir` target installs the
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
