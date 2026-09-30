# Local Map

`local_map` owns the independent planner's single ODOM-frame scene truth.
`LocalScene` treats an affirmative free-space polygon, curb polylines, and
static obstacle polygons as the primary world representation. Valid measured
lane evidence overlays topology/rules and selects LANE mode. Without usable
lane evidence, valid positive free-space evidence publishes AREA with empty
lane fields; without valid free space the scene is INVALID.
Source envelopes carry frame, clock, ODOM epoch, measurement/publication times,
deadline and health; observation and lane uncertainty are separate fields.
Individual environment objects carry IDs and geometry, not independent
per-object timestamps or track lifecycles. Base-link observations are
transformed with measurement-time ODOM before publication.

## Independent world-model modules

The Bazel targets expose real, typed module boundaries rather than one scene
construction target:

| Target | Input → output | Responsibility |
| --- | --- | --- |
| `lane_association` | Fixed-grid candidate/history samples → fused samples or rejection | Spatial association gate, minimum overlap, conservative smoothing and immediate inward restrictions. It owns no track identity or clocks. |
| `temporal_lane_map` | Paired observations + measurement-time ODOM → bounded temporal snapshot | Pose compensation, resampling, bounded identity/history and freshness. |
| `measured_lane_geometry` | Measured paired cubic boundary intervals → sampled boundary geometry / paired observation | Derive finite measured heading, curvature and arc length; no inferred range. |
| `lane_synthesis` | Temporal snapshot → `LaneLayer` | Materialize observed boundary/lane entities with per-sample provenance and expiry; do not infer topology, permission or rules. |
| `drivable_environment` | Bounded base-link region/curb/obstacle observations + measurement-time pose → ODOM `DrivableEnvironment` | Validate and transform affirmative free-space and static restrictions independently of lane tracking; validate independently supplied ODOM layers before publication. |
| `navigation_prior` | Expiring local lane-ID intent + scene → scene with attached intent or rejection | Attach intent only; it adds no geometry, connectivity or crossing permission. |
| `scene_assembly` | Scene source + environment + optional lane layer/intent → validated `LocalScene` | Check provenance, polygon geometry, paired lane orientation and entity references, then select LANE or AREA without fabricating geometry. |
| `local_scene_builder` | Source admission/events → versioned scene value or explicit invalid result | `LocalSceneBuilder` owns frame/clock/epoch/time admission, invalidation, lifecycle and module routing; geometric algorithms remain in their targets. |

Shared data-only contracts are in `lane_types` and
`local_scene_contracts`. Local planning depends on scene contracts; only the
scene-producing simulation adapter depends on `local_scene`. This permits
replacing/expanding one algorithm without adding a planner dependency or
creating a second source of scene identity.
`drivable_environment.{h,cc}` implements the environment target;
`local_scene_builder.{h,cc}` implements the producer, while `local_scene.h`
holds scene data contracts. Here "map" means a rolling local ODOM scene, never a global map.
These are value structs: publication/admission enforces version immutability,
not C++ `const` fields on every contract.

The association capability remains deliberately bounded: ingestion supplies
one ordered paired left/right observation. The separable association function
operates on fixed-grid paired samples, while `TemporalLaneMap` retains the one
corridor's identity and history. It does not claim general unpaired-boundary
association, split/merge lineage, or intersection construction.
`measured_lane_geometry` also accepts paired cubic boundaries only over their
common declared measurement interval. It derives sampled heading, curvature
and arc length without extrapolating a speed horizon or fabricating straight
lanes/default widths.

The joint environment accepts one counter-clockwise convex measured free-space
polygon, ordered curb polylines, and counter-clockwise convex static obstacle
polygons. Unsupported or stale shapes fail explicitly. Unknown space is not
free. The world model remains vehicle-independent; local planning applies its
footprint and clearance once, checks continuous swept capsules against every
constraint, and keeps dynamic future occupancy in the existing Prediction
input.
The static environment is a conjunction of **positive free-space coverage**
and curb/obstacle exclusions, not a polygon Boolean union or a raster grid.
Lane geometry does not grant drivable area outside that coverage. The builder
and scene assembler keep source timestamps and deadlines separate from scene
publication; republishing never extends observation lifetime. The assembler
rejects malformed ODOM environment geometry and non-paired/reversed lanes even
when they were supplied by a future independent producer.
The environment sequence/measurement/publication frontier is separate from
the currently usable environment payload. Invalid or expired geometry
withdraws the scene without permitting an older source to be replayed; only a
new ODOM epoch resets that frontier.
`LocalSceneBuilder` requires a healthy stationary explicit epoch, monotone ODOM
and measurement-time poses for observation transforms. It disarms on invalid
current ODOM, withdraws lane eligibility on absent/invalid/expired lane data
while retaining independently valid area evidence, and withdraws the whole
scene when environment evidence fails. Successful builds assign a new scene
sequence, not a new measurement timestamp. A navigation prior attaches only
to an assembled LANE layer and may select an existing observed successor; an
AREA build does not retain a supplied prior and explicitly states in
`mode_reason` that navigation could not apply without a lane. It does not
validate the prior's contents in AREA; no route is implied.

## Producer contract

1. `BeginEpoch` arms a healthy stationary ODOM epoch; `AddOdometry` supplies
   monotone poses. No global pose may initialize or correct ODOM.
2. `ObserveEnvironment` receives one measured positive region with optional
   curb and static-obstacle exclusions. `Observe` receives paired measured
   lane boundaries, or `ObserveLaneAbsence` explicitly revokes lane history.
   Both use the ODOM pose interpolated at their **measurement time**.
3. `Build(now, &scene, optional_navigation)` emits a new scene sequence.
   With valid region and lane: LANE. With valid region but no eligible lane:
   AREA (no fabricated reference). Without valid region or ODOM: error and
   INVALID output with reason. Navigation is advisory only for observed
   topology, never a geometry source or permission grant.

The published C++ value is mutable in memory; callers must treat a version as
immutable. Planning admission checks the full scene for same-version changes.
Invalid operations return an explicit status and do not publish a usable
scene. The scene producer does not wait for dynamic Prediction; planning
joins matching graph and occupancy versions separately.

The source-side adapter supplying paired base-link lane evidence and measured
environment objects, real ODOM/uncertainty estimation, unpaired boundary
association, persistent curb/obstacle tracking and multi-lane graph construction
are not implemented here. See the
[core audit](../../../wheelos-service/context/modules/world_model/knowledge/core-framework-completeness-review.md#source-backed-architecture-audit-2026-09-30)
for acceptance evidence and remaining integration gates.

The former `modules/world_model/relative_map` package was removed. Shared
`relative_map::MapMsg` and `NavigationInfo` message contracts remain owned by
`wheelos_msgs` for unrelated legacy consumers; this package does not provide a
compatibility bridge or recreate their absolute-localization behavior.
