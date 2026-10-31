# OpenLoc diagnostic Cyber host

This independent, opt-in consumer links the public native OpenLoc library
with Cyber. It does not change the product module graph or existing RTK/NDT
components. It includes an explicitly selected fixed-lag diagnostic path, but
**is not a production localization replacement or a live-deadline guarantee.**

For the existing-dataset **fusion algorithm** baseline, see
[`OpenLoc raw-native source ablation`](../../ad_agents/context/modules/localization/knowledge/openloc-three-source-dataset-baseline.md#current-raw-native-result-2026-10-24).
It runs the independent OpenLoc IMU/LiDAR+map/GNSS replay, not the local-only
Cyber executable or the prepared-input global host below. Actual map/GNSS corrections, common-epoch trajectory
consistency and unanchored GNSS-only behavior are recorded separately from
the sender/prefetch timing baseline; neither claims target-graph or production
acceptance.

## Structured NovAtel vehicle ingress

The global host can select `structured_gnss` instead of `raw_gga` or prepared
XY. It subscribes to `apollo.drivers.gnss.GnssBestPose` on `gnss_topic` and,
when configured, `apollo.drivers.gnss.Heading` on `structured_gnss.heading.topic`.
The source clock conversion is explicit: current NovAtel GPS-epoch seconds and
historical Unix-timestamped records must not share an inferred offset.
Live receive time is captured at the callback; recorded mode uses the original
header publication/receive time, not a new replay wall clock.

Position admission requires all relevant fields, WGS84, a pure computed
`NARROW_INT` solution, finite positive receiver standard deviations, satellite
count, differential age, source freshness and fixed-solution dwell. `PSRDIFF`,
float, INS-derived solution types, missing uncertainty and stale/repeated times
are not qualified. BestPose has no HDOP; its absence is represented explicitly,
not manufactured as zero. Only XY is submitted. The diagonal ENU receiver
variances are projected into the configured UTM grid including local convergence
and scale; unknown receiver cross-covariance remains unqualified. MSL height
is never silently treated as ellipsoid height or a full-pose observation.

Dual-antenna heading admission has its own status, satellites, standard
deviation, calibrated baseline-length, freshness and stability checks. The
reported degree azimuth is converted to map-frame radians using the declared
true-north direction in that map. Configure the primary-to-secondary vector
in the local estimator's IMU/base frame, not an assumed forward-facing antenna
pair. UTM grid north and true north need not coincide. The fixed north angle
is a declared local-area approximation, not an automatic datum/grid calibration.
Position and heading go to the same selected global owner; neither updates
local ODOM or invents height/roll/pitch/full-pose availability. Adapter gates
and actual owner correction decisions are separate journal entries.

`testdata/native_novatel_global_profile.pb.txt` is a **synthetic wiring
example**, with fake extrinsics/origin, not a vehicle-ready configuration.
Set the actual clock, map reference, lever arm, antenna baseline and thresholds
before use. The unchanged local host takes raw `Imu` from
`/apollo/sensor/gnss/imu` and timed LiDAR with its own complete calibrated
profile. Do not feed `CorrectedImu` or receiver INS attitude as raw independent
measurements, or use same-receiver INS output as independent ground truth.
The existing diagnostic output remains opt-in and untrusted: this does not
switch `/apollo/localization/pose`, `/tf`, RTK/NDT or downstream planning.

From the managed test container, in this independent consumer workspace:

```bash
env -u LD_LIBRARY_PATH bazel test --config=dev --lockfile_mode=off -c opt \
  //:native_structured_gnss_adapter_test //:native_global_worker_test \
  //:native_global_cyber_test \
  --test_env=CYBER_PATH=/apollo/pkgs/core/cyber --test_output=errors
```

New vehicle recordings still need raw IMU, per-point LiDAR time, BestPose,
independent dual-antenna Heading, source status and real calibration. The
September segment lacks raw IMU and Heading; it cannot establish that closed
loop merely because these adapters compile.

For the current NovAtel driver, verify the receiver actually emits
`RAWIMU`/`RAWIMUX`, `BESTGNSSPOS` and `HEADING`: `BESTPOS` alone is not the
BestPose publication path, and the parser does not currently dispatch
`HEADING2`. Record `INSPVAX` status and the `INSPVA`/`CORRIMUDATA`-derived
odometry/corrected IMU as additional diagnostics, not independent fusion inputs.

## Implemented boundary

`native_imu_diagnostic` consumes calibrated forward/left/up raw
`apollo.drivers.gnss.Imu` and runs one `NativeOdometry` owner. IMU-only
quiet-window initialization is explicitly enabled; it cannot establish zero
translational velocity or qualified motion uncertainty. No wheel, LiDAR,
bootstrap registration, map or GNSS observations are consumed by this executable.

`native_lio_diagnostic CONFIG.pb.txt` adds calibrated raw
`apollo.drivers.PointCloud`, relative motion bootstrap and local LIO. Its strict
protobuf text profile requires both source clock conversions, sensor frame,
calibration identity, complete rigid extrinsic, bootstrap voxel size and every
numeric LIO parameter. Unknown, duplicate, missing and nonfinite parameters,
fractional counts and nonunit extrinsic quaternions are rejected. The provided
`testdata/native_lio_profile.pb.txt` is synthetic, not a vehicle calibration.

The IMU callback enqueues into a single owner worker. Raw cloud decoding does
not hold the IMU enqueue mutex. Exact-zero XYZ no-return sentinels are counted
and removed before bounded stride sampling; original point times and scan end
are preserved. The worker checks frame/calibration, rigid transform and ordered
finite points before bootstrap as well as LIO. It records the raw end/source
receive time separately from the first eligible processed IMU frontier.
The FIFO is bounded to 256 events/16 MiB, with at most two pending scans;
overflow and worker exceptions terminate explicitly rather than silently drop
accepted inputs. A scan lacking its IMU frontier at shutdown is journaled as
`HISTORY_UNAVAILABLE`.

By default geometry runs synchronously to preserve the frozen parity arm.
Set `lidar_commit_delay_ms` explicitly to a value in `[5,100]` to calculate
post-initialization LIO on an isolated public `LidarTask`. The owner retains at
most 128 accepted IMUs, replays them after compute, and commits at the exact
live frontier. It never rewrites already published output poses. At the first
IMU receive frontier at/after the declared delay, an unfinished task is marked
`LIDAR_STALE` and later discarded; unfinished end-of-input tasks are not
committed. A separate `.tasks.csv` records queued/committed/rejected/deadline/
shutdown outcomes. There is only one outstanding task.

Relative startup remains synchronous by default to preserve existing parity.
Set `startup_task_timeout_ms` explicitly in `[1,500]` to compute provisional
relative registration and motion initialization on an isolated
`NativeMotionBootstrap::Task`/`NativeOdometry::StartupTask`. One task owns its
copied IMU/bootstrap/local state; the live owner continues accepting IMUs and
publishing unavailable output until initialization can actually commit.
The host retains at most 128 subsequent accepted IMUs, replays every one,
and requires the exact live IMU frontier and original owner identity before
adopting startup. Previously published output is never rewritten; no map,
GNSS, absolute seed or new stationary assumption enters this path.

The `.startup.csv` ledger records queued, retained relative history,
initialized, deadline/buffer retirement, discarded and uncommitted shutdown
outcomes. Both steady time from task launch and source receive age bound
commit; there is no cancellation or concurrent second startup. Quiet-window
initialization of the live owner retires the pending task rather than
overwriting that owner. A retired computation cannot run concurrently with
LIO geometry. Async exceptions are polled even without another IMU and surface
through `Check()`/`Finish()`. The independent publication timer exposes
actual age without renewing source stamps.
The cloud reader has a two-message queue and the IMU reader a 256-message
queue. Optional live producer-sequence enforcement and persistent admission
records are described below. They detect visible sequence discontinuities,
not complete transport loss or qualified overload behavior.

### Explicit raw-source ingress contract

Set `require_live_source_sequences: true` only for **single-producer live**
IMU and cloud streams with genuine consecutive `Header.sequence_num`.
Missing headers, gaps, duplicates and regressions fail before worker enqueue;
uint32 rollover is supported. Each stream starts from its first observed
sequence, recorded as a baseline. Loss before that baseline, loss after the
last received packet, producer-side loss before numbering, and silent writer
replacement cannot be inferred. This is detection, not reliable delivery,
automatic retry, loss-location attribution or live deadline qualification.
Default headerless live behavior is unchanged and explicitly unqualified.
Recorded mode retains its existing original-sequence/retry contract and
cannot be combined with this live option.

`JOURNAL.source.csv` is created before any reader and flushed after every
actual enqueue, including stream, receive time, original producer sequence,
admitted count and whether sequence checking was active. Adapter failures,
worker failures/overflow and publisher failures leave fatal records; a
failed enqueue is never counted as admitted. Journal failures terminate
explicitly. The existing success-only `.ingress.csv` summary remains
compatible with recorded replay. Input readers are shut down and executing
callbacks joined **before** captured worker/journal state is destroyed, on
normal shutdown and exception paths.

Independent-process cases cover actual IMU/cloud admission followed by
visible source gaps, missing sequence headers and missing point coordinates.
Distinct synthetic readiness samples establish callback delivery before fault
injection; topology discovery alone is not treated as transport readiness.
No real observations are retried by the strict live adapter.

A separate 10 ms Cyber timer publishes the latest immutable native snapshot.
Neither message writes nor TF publication hold the estimator/callback mutex.
This is requested timer frequency, not fresh 100 Hz ODOM or deadline
qualification. The message reports actual steady-clock publication intervals.
Usable snapshots also carry the owner's actual row-major 15D nominal error
covariance, biases, angular velocity and explicit model/calibration flags.
Held publications preserve that payload exactly; stale/faulted output withdraws
it with pose/velocity. This is not the sufficient augmented native error model
and does not enable target-graph admission. The diagnostic global owner still
uses its explicitly configured diagnostic sigmas rather than relabeling these
native marginals as qualified independent motion.

| Channel | Message | Contract |
| --- | --- | --- |
| `/openloc/diagnostic/odometry` | `NativeOdometryDiagnostic` | New/held/unavailable, source measurement/receive time and sequence, session/generation, age, integrity reason and explicit untrusted status |
| `/openloc/diagnostic/tf` | `apollo.transform.TransformStampeds` | New usable samples only; original measurement timestamp; never `/tf` |

Native coordinates refer to the **IMU origin in local ODOM**, not ENU or the
rear axle. Accordingly this host does not publish `LocalizationEstimate`.
Both ODOM and IMU frame names include session/generation; epoch changes cannot
silently reuse an old diagnostic TF edge. This is epoch notification/isolation,
not a reset-request or consumer-acknowledgement protocol.

Held samples never renew the source timestamp; expired/faulted samples contain
no pose or velocity. No extrapolation or invented covariance is published.
All output is `diagnostic_only=true`, `trusted=false`, even for finite native
states. Writer failures and callback exceptions surface through `Check()`/
`Finish()` and cause executable failure. Cyber submission success is not a
delivery acknowledgement. The input reader has a bounded 256-message queue;
queue-drop accounting and qualified overload behavior are not implemented.

The raw worker also forwards `CORRECTION_BUDGET_EXCEEDED` from the owner's
existing output-envelope assessment. Such samples retain source clocks and
identity but carry no pose/velocity/TF. Local propagation continues unchanged,
and later in-envelope samples may restore diagnostic output in the same epoch.
This reversible withdrawal is distinct from uncertainty qualification and does
not make wheel freshness a prerequisite for diagnostic IMU/LiDAR ODOM.

The raw IMU/LiDAR host also latches a fatal source/worker failure in the same
publisher and immediately submits an unusable, no-pose/no-velocity status
before teardown. The last native source identity/times remain unchanged;
subsequent owner updates cannot restore usability. No TF is emitted for that
status. Its receipt withdraws the global worker's local availability, including
pending map work; GNSS and held ODOM cannot revive the retired local output.
Four separate-process ingress-fault cases require actual receipt of this
terminal status as well as the persisted failure ledger. A broken publisher
cannot promise delivery; transport failures are logged/journaled and the
process still exits unsuccessfully.

The connected recorded-map path can explicitly opt into the existing
enqueue-acknowledgement protocol: set `online_map.recorded_ack_session` to the
local replay session and pass `--map-admission-ack` with `--map-cloud-topic`
to the sender. Byte-identical last-packet retries are re-acknowledged without
reenqueue; conflicting retries, missing identities and skipped sequences fail.
Live profiles cannot enable this mode. ACKs mean successful bounded enqueue,
not a successful match/correction. The graph/worker duplicate-observation gate
is unchanged. This closes an observed first-map-packet loss after topology
discovery; it does not turn Cyber write success into reliable live delivery.

## Opt-in single target-graph owner

Add `graph` to the local LIO profile and `target_graph` to the global profile.
The former exports **separate** `NativeJointOdometryInput` messages after all
serialized LIO commits at a frontier. The latter changes `odometry_topic` to
that wire type and constructs only `FixedLagGlobalAlignment`. With no field,
the existing `DiagnosticGlobalAlignment` path is unchanged. Neither mode runs
the other as a fallback or feeds absolute corrections back to local ODOM.

Synthetic functional-test settings, **not vehicle or live timing defaults**:

```text
# NativeLioHostConfig addition
graph {
  output_topic: "/openloc/diagnostic/joint_odometry"
  motion_error_model_id: "native-frozen-linearization"
  imu_sample_period_s: 0.01
  maximum_error_dimension: 1024
  maximum_memory_rows: 4096
  maximum_snapshots: 64
  history_duration_s: 0.1
  odometry_stride: 1
}
```

```text
# NativeGlobalHostConfig: replace odometry_topic and add target_graph.
odometry_topic: "/openloc/diagnostic/joint_odometry"
target_graph {
  motion_error_model_id: "native-frozen-linearization"
  maximum_error_dimension: 1024
  history_duration_ms: 60
  maximum_observation_age_ms: 60
  maximum_states: 100
  solve_budget_ms: 100000
  verification_frames: 2
  allow_uncalibrated_inputs: true
  assume_deterministic_map: true
  scan_voxel_m: 0.05
  minimum_scan_points: 20
  normal_neighbors: 6
  motion_error_model_qualified: true
}
```

The model qualification declares sufficiency, **not empirical calibration**.
Both sides must name the same model. The atomic packet preserves the native
unit-driving-variable basis, actual output projection, rectangular previous/
current cross-covariance and source IDs. An optional scan carries the actual
LIO-consumed pre-correction deskewed points and loadings, including original
measurement/receive times and the potentially later conditioning node.
It does not modify already published nominal ODOM payloads.
Scan reference and commit frontiers are exported even when ODOM is strided.
Dimension is bounded by 2048, each packet by 16 MiB and the target worker queue
by 256 events/64 MiB. Missing/truncated/oversized models fail explicitly; the
graph cannot substitute diagnostic sigmas or truncate dependencies.

For shared-map mode, configure the existing complete `online_map` fixed pose
hypothesis, its ODOM epoch and validity interval, immutable PCD/hash and
`gnss_map_construction: ABSENT`. The map is explicitly conditioned on as
deterministic. The global host consumes the transported shared scan, **not**
another raw-cloud reader/decoder. Off-owner ICP includes the full loading of
the configured fixed alignment applied to conditioning ODOM; it never labels a
graph/GNSS posterior seed deterministic. This path does not support the
separate regional-policy or raw-map enqueue-ACK modes. Match failures, busy
tasks, retired generations and expired hypotheses are journaled, not replaced
by the diagnostic owner.

Prepared map/GNSS inputs require explicit `graph_qualification`; absence does
not imply independence. Raw GGA may carry that external declaration in
`raw_gga.graph_qualification`, still behind its actual checksum, quality and
dwell gate. RTK FIX and configured sigma cannot manufacture the declaration.
The journal retains source-model/evidence identities and conditioning
sequence/time. Reset reconstructs the selected owner only, preserves local
ODOM and requires every named consumer ACK before admitting new absolute
evidence.

Publication names `selected_owner`, `motion_error_model_id` and the untrusted
graph assessment. `target_graph_available` describes a fresh diagnostic
full-pose graph result, never calibrated trust. All output remains
`diagnostic_only=true`, `trusted=false`; unavailable models withdraw target
availability with an explicit reason.
Source/worker faults also publish an immediate unavailable message on this
separate joint channel, including before the first valid graph snapshot.
Graph sampling/writes are serialized and terminal invalidation is sticky;
neither a later owner callback nor a held nominal snapshot can restore it.
The cross-process source-gap case verifies actual receipt, not just write
success. Broken transport still cannot promise delivery.

The maintained cross-process regression uses actual raw IMU/LiDAR native
snapshots with **all four scans delayed**, actual shared ICP, three map
updates, lag retirement and admitted raw GGA in the same graph. It preserves
nominal ODOM and original scan references. This is recorded diagnostic
functional evidence, not a new two-capture dataset result, wall-time
throughput/overload qualification, survey uncertainty or accuracy acceptance.

The raw sender → local executable → selected target-graph executable regression
now also passes: 224 raw IMUs, four raw scans and two raw GGA packets yield
three actual map corrections and one GNSS correction through that sole graph,
with the final subscribed source frontier at 224. These are synthetic sensor
fixtures processed by the real native owner, not external/frozen ODOM or a new
real-capture baseline.

The recorded sender option `--global-topic TOPIC --target-graph`
checks the selected owner/model, retains diagnostic/untrusted labels and
requires actual available graph output. It uses recorded processing
backpressure and journals sender lateness; it is not a throughput benchmark.
It requires local `graph` configuration and excludes a separate raw-map topic.
For offline functional replay, explicitly set `recorded_source_clock: true`
in **both** local `graph` and global `target_graph`, with recorded input enabled.
This separate graph channel evaluates source age from original measurement and
receive times; it does not claim wall freshness. Actual arrival-to-publication
delay remains in `processing_delay_s`, global `local_processing_delay_s` and the
subscriber journal. Publication clocks and nominal ODOM are unchanged. The
sender allows up to 30 seconds per offline processing frontier (otherwise two);
neither bound is a live deadline. Source-clock profile/wire mismatches fail.
The full-chain run observed processing delays above two seconds and sender
lateness above seven seconds, so it does **not** establish live throughput.

Without this explicit offline mode, the original freshness checks still
withdraw late models, including in recorded input mode. The earlier failed
wall-freshness attempts remain evidence of that limitation. A withdrawn frame
now leaves the worker's previous submitted motion endpoint unchanged, so the
next retained bridge does not reference a locally unpublished model. This is
not a delivery ACK; lost transport or retired support remains unavailable.

## Build and exercise

Inside the managed container as its mapped non-root user:

```bash
cd /apollo/modules/openloc_localization
env -u LD_LIBRARY_PATH bazel mod show_repo --config=dev --lockfile_mode=off \
  @openloc @core @wheelos_msgs
env -u LD_LIBRARY_PATH bazel test --config=dev --lockfile_mode=off \
  -c opt //:native_lio_config_test //:native_lio_worker_test \
  //:native_publication_test //:native_cyber_publisher_test \
  //:native_imu_host_test //:native_recorded_cloud_test \
  //:native_global_worker_test //:native_global_cyber_test \
  //:native_map_matcher_test //:native_gga_adapter_test \
  //:native_graph_transport_test \
  --test_env=CYBER_PATH=/apollo/pkgs/core/cyber \
  --test_output=errors
env -u LD_LIBRARY_PATH bazel build --config=dev --lockfile_mode=off \
  -c opt //:native_imu_diagnostic //:native_lio_diagnostic \
  //:native_lio_recorded_replay //:native_lio_cyber_replay \
  //:native_global_diagnostic //:native_global_cyber_replay
```

The local paths resolve to `/apollo/pkgs/{openloc,core,wheelos_msgs}`.
The consumer preserves OpenLoc's tested Boost component overrides and selects
Apollo's existing gRPC 1.69.0 baseline to avoid the old gRPC/BoringSSL
compatibility-level conflict. This is local-source consumption, not registry
release validation. Dependency-root patches are not automatically inherited:
this consumer explicitly carries the same Eigen unsupported-header patch for
PCL startup and GTSAM `<cassert>` patch for the diagnostic global owner's
dependency closure. Linking that closure does not qualify the target graph.

## Prepared-input global diagnostic host

`native_global_diagnostic CONFIG.pb.txt` runs separately from the local
IMU/LiDAR executable. One bounded worker owns `DiagnosticGlobalAlignment`;
copies of `NativeOdometryDiagnostic`, prepared GNSS (or opt-in raw GGA),
optional map observations and opt-in global reset/consumer ACK are its inputs.
It cannot reset, initialize or
correct local ODOM. Start it explicitly:

```bash
CYBER_PATH=/apollo/pkgs/core/cyber env -u LD_LIBRARY_PATH \
  bazel-bin/native_global_diagnostic testdata/native_global_profile.pb.txt
```

The example profile is **synthetic**, not vehicle calibration. Replace every
reference/calibration/content identity and choose a new journal filename.
The journal must not exist. The explicit gravity-level assumption and
uncalibrated ODOM position/rotation sigmas are required; they are not a
native conditional error model. `recorded_input` must match all input streams.

The GNSS adapter must provide quality/dwell-admitted antenna XY in the declared
reference, original integer Unix measurement/receive ns, producer
session/generation, source sequence, lever arm, preparation/covariance source
and optional GNSS/map dependence. `receiver_admitted=false` is explicitly
rejected and recorded. This prepared mode does not independently validate
the adapter's receiver admission. Optional raw mode is described below.
No heading, height, reference qualification or full pose is manufactured.
The ODOM wire still uses double seconds, so its converted ns can be rounded;
Prepared GNSS integer identities remain exact.

### Optional raw NMEA GGA input

Adding `raw_gga` to the same profile switches `gnss_topic` from prepared XY
to `apollo.drivers.gnss.RawData`; the two input types cannot be mixed.
Framing, checksum verification, original UTC time decoding, receiver quality/
fixed-solution dwell and projection run on the **same global worker**.
`GnssPositionGate` (`@openloc//:gnss_position_gate`) is a public
middleware-independent facade over the existing receiver gate, not a second
estimator or a new admission algorithm.

For a synthetic WGS84/UTM50N frame only, append:

```text
raw_gga {
  utm_zone: 50
  northern_hemisphere: true
  origin_easting_m: 500000
  origin_northing_m: 4427757.218738374
  lever_x: 0
  lever_y: 0
  lever_z: 0
  diagnostic_sigma_m: 1
  preparation_id: "synthetic-wgs84-utm50n"
  minimum_satellites: 6
  maximum_hdop: 3
  maximum_differential_age_s: 10
  fixed_stability_ns: 2000000000
  maximum_gap_ns: 2000000000
  gnss_map_construction: UNKNOWN
}
```

Deployment must replace the synthetic reference/calibration/content IDs,
origin, IMU-to-antenna lever and preparation identity with the declared frame
contract. Origin is never inferred from the first fix. Projection is explicit
WGS84 geographic degrees to UTM grid metres minus that origin, using a PROJ
pipeline without `proj.db` or datum/grid downloads. This is not exact ENU.
Only the declared ordinary six-degree zone/hemisphere and UTM latitude range
are admitted; special zone extensions are not inferred.

Inputs are a NMEA byte stream, not arbitrary binary receiver messages:
checksum-valid `GPGGA`/`GNGGA`, newline terminated, including fragmented or
multiple sentences per packet. Other NMEA types cannot provide position
observations. Packet size is at most 4096 bytes, buffered sentence at most
256 bytes; overflow or incomplete shutdown fails explicitly. Each packet has
a processing outcome; each complete GGA has a separate `gga` journal result.
Malformed/checksum/truncated GGA resets receiver dwell. Fixed quality 4,
satellites, HDOP and differential age must satisfy the configured shared gate;
missing HDOP/age is rejected. Missing receiver accuracy is explicitly allowed
only for this unqualified diagnostic mode; the configured sigma is **not**
reported as receiver accuracy.

UTC time-of-day retains up to nine fractional digits and uses the nearest
receive UTC day, including midnight; leap seconds are rejected rather than
guessed. Measurements older than two seconds or over 20 ms ahead of receive
are rejected, without time shifting. New native ODOM epochs clear partial
framing and restart receiver stability before global reacquisition.
Live receive time is captured at the Cyber callback. Headerless live packets
are supported because the current GNSS raw driver emits them; their sequence
counts callbacks, **not upstream source delivery**. Recorded mode instead
requires original Unix receive time and a positive contiguous source sequence
in the RawData header. Its double seconds may round receive ns; GGA
measurement ns remain exact. Source sequence gaps/regressions fail explicitly;
there is no automatic retry.

The same diagnostic owner consumes admitted antenna XY and the explicit lever.
Raw GNSS cannot initialize, reset or correct local ODOM; height/heading/full
pose/trust remain absent. Live receiver/clock/overload qualification is not
established by this adapter.

To enable map admission, add `map_topic` to the profile, for example
`map_topic: "/openloc/diagnostic/prepared_map"`. With no field, no map reader
is created and the existing no-map behavior is unchanged. The map adapter
must supply an IMU-origin `map_from_imu`, original integer measurement/receive
ns, matching epoch/reference/calibration/content, preparation/covariance source,
ambiguity/recovery flags and declared local-scan/deskew/GNSS dependencies.
Those declarations are audit evidence, not a joint statistical qualification.
The same `content_id` identifies the configured global/map contract for both
absolute sources; unrelated references are not silently mixed.

Full-pose map input requires exactly 36 row-major covariance entries ordered
as translation in map then left rotation in map, and no projection fields.
Projected input requires rank 1-5, `rank*6` row-major projection entries and
`rank*rank` projected covariance entries, without a full covariance array.
The existing owner validates geometry, covariance, source time/order, ambiguity,
epoch and multi-observation acquisition. Projected evidence cannot establish
missing full-pose axes. Prepared mode consumes matcher results; the mutually
exclusive raw mode below runs the matcher in this process.

The worker FIFO is bounded to 256 outstanding events/1 MiB, each at most
64 KiB, except bounded sampled raw scans (512 KiB). Overflow, malformed local snapshots, journal/transport failures and
worker exceptions fail explicitly. Receiver/reference/epoch and owner rejects
are recorded without pretending that retained evidence is a correction.
The 10 ms timer publishes `NativeGlobalDiagnostic` only on a configured
`/openloc/diagnostic/` topic. GNSS-only output remains XY plus diagnostic
alignment XY/yaw with `full_pose_available=false`. After map verification,
fresh full-pose evidence permits diagnostic `map_from_imu`/`map_from_odom`
and `full_pose_available=true`, not trusted ENU/rear-axle localization or a TF
edge. `trusted` and `target_graph_available` remain false in both modes.
Held snapshots do not renew source time or arrival. GNSS and projected-map
updates never renew the full-pose evidence lease. Expired local/full evidence
omits poses and XY instead of publishing stale SE(3) as current. New ODOM
epochs retire the old alignment and require reacquisition.
Shutdown explicitly stops readers retained by the Cyber node before draining
the worker.

### Optional raw scan-to-map input

`online_map` replaces `map_topic` with raw `apollo.drivers.PointCloud`.
It needs an explicit map PCD path and SHA256, LiDAR frame, full
`imu_from_lidar`, bounded window radius/cell, task deadline (1-500 ms),
preparation/dependency declarations, and either a diagnostic `initial_map_from_odom`
hypothesis or an explicit regional search policy. The hypothesis has **both** an ODOM session/generation and a
Unix-ns validity interval; it is never inferred from identity or applied to
native ODOM. See `NativeOnlineMapConfig` in
`native_global_diagnostic.proto` for required fields. No default production
map or vehicle calibration is supplied.

Map loading verifies the actual SHA256 before and after load using the
container's `sha256sum`, with no shell expansion. Only uncompressed float
XYZ/XYZ-intensity PCD, at most 128 MiB/two million points, is admitted.
Intensity is not a matching measurement. Map contents and the declared
global contract remain immutable for the process lifetime; hash integrity
does not qualify physical-up/geodetic accuracy or independence from GNSS.

Raw scans are bounded to 200,000 points/8 MiB before copying from Cyber.
Every original point is checked for finite XYZ and ordered positive integer
Unix time before any sampling. Exact-zero no-return sentinels are removed;
deterministic stride bounds retained points to 12,000. Input count, sentinel
count and stride are journaled. The scan-end wire is double seconds: only
the calculated two-conversion rounding envelope permits an original integer
endpoint later than its rounded wire reference; point timestamps are never
shifted. Recorded receive time comes from the original header; live receive
time is captured at the callback.

One pending/running task owns copied scan/history, with no estimator pointer.
It waits for the continuous ODOM frontier without blocking ODOM/GNSS/reset
processing, then deskews into the end-IMU frame using copied native poses
(at most one second/512 samples, no interpolation across gaps over 50 ms).
The existing public `ScanToMapLocalizer` runs point-to-plane ICP with its
unchanged geometry gates. Full/projected results enter the **same** diagnostic
global owner as GNSS. Local ODOM cannot receive corrections. Fresh verified
full-map alignment supplies later tracking guesses; once its lease expires,
the configured acquisition mode must obtain new evidence.

For acquisition without a pose seed, supply `regional_policy_csv` and
`regional_policy_sha256` instead of **all** `initial_map_from_odom` and
`hypothesis_*` fields. The bounded `name,value` CSV explicitly declares
`field_min_{x,y,z}`, `field_max_{x,y,z}`, `search_min_{x,y,z}`,
`search_max_{x,y,z}`, `observed_min_{x,y,z}`, `observed_max_{x,y,z}`,
`yaw_min_rad`, `yaw_max_rad`, `tilt_q{x,y,z,w}`, `field_resolution_m`,
`distance_sigma_m` and `inlier_distance_m`. These are conditional search,
tilt and coverage priors, not a known live location or qualified visibility.
The file is hash-checked before/after parsing; missing, unknown, duplicate or
nonfinite values and invalid bounded field/score configurations are rejected.

The public `DiagnosticRegionalSearch` reuses the existing bounded particle
search, full-rank geometric refinement and distinct-frame hypothesis tracker.
Truncated or competing modes cannot supply a pose. A stable unique proposal
still needs the same global owner's separate multi-observation verification.
Local epoch/global alignment identity changes or loss of a previously fresh
full-pose lease restart search/tracker state. This is **seed-free conditional
regional acquisition/recovery, not unconstrained whole-map/unknown-tilt
relocalization**; the configured initial-hypothesis mode remains separate.

The task deadline includes owner-queue and ODOM-frontier wait from callback
admission. Busy, missing history, matching failure, deadline expiry, source
staleness, ODOM epoch retirement and global reset retirement are explicit
outcomes. An over-deadline computation is allowed to finish, but never
committed; another matcher task cannot start concurrently. Shutdown drains
admitted tasks, rejects a missing final ODOM frontier and surfaces async
exceptions. `map_tasks_completed` counts terminal task decisions separately
from the ingress `processed_events` watermark. Cyber's two-cloud reader queue
still does not qualify complete upstream drop accounting or live deadlines.

`native_global_cyber_replay` accepts `raw_map JSON` in this mode. Its
admission/completion-paced protocol continues ODOM events while a scan waits
for its frontier and retains asynchronous `map_match` outcomes. This proves
functional wiring, not open-loop throughput.

The real-input host proof is retained under
`pkgs/openloc/.agents/notes/native_online_map_host_20261026_v4/`.
It uses existing recorded raw point clouds plus unchanged frozen native ODOM,
not a new raw-LIO run. Offline same-capture map hypotheses remain explicitly
future-informed; synthetic separate-process tests independently cover actual
cloud delivery, matching and full-pose publication. Neither establishes
independent accuracy, target-graph availability or production readiness.

The seed-free regional proof is
`pkgs/openloc/.agents/notes/native_regional_host_20261026_v1/audit.json`.
A fresh third-capture raw IMU/LiDAR replay makes 146 map corrections while
preserving all 5,999 native ODOM rows and local scan decisions except measured
processing time. Its unchanged ODOM feeds 120 actual raw point clouds to the
Cyber global host, which commits 106 map corrections without GNSS or a pose
seed. The same-capture draft map and manual region/identity-tilt/coverage
priors remain explicitly unqualified.

Separate injected-fault evidence is
`pkgs/openloc/.agents/notes/native_regional_host_faults_20261026_v1/audit.json`.
Removing 26 consecutive clouds withdraws expired full pose and triggers
regional reacquisition (72 corrections). Explicit reset/consumer retirement
withdraws pose before acquisition in the next alignment generation
(69 corrections). Both arms preserve every ODOM input line byte-for-byte.
These are completion-paced diagnostic functions, not live rate/overload or
accuracy qualification.

The independent startup proof is
`pkgs/openloc/.agents/notes/native_async_startup_host_20261026_v2/audit.json`.
All three raw 30-second captures initialize through the isolated startup
boundary, preserve all 17,999 input IMU identities/times and subsequently
commit delayed LIO tasks. Startup catch-up stays bounded; a final LIO task
without another IMU remains explicitly uncommitted. This proves functional
startup/LIO scheduling, not common trajectories with synchronous startup,
fresh 100 Hz output, complete ingress-drop accounting, target-graph
qualification or independent accuracy.

### Explicit global alignment reset and consumer retirement

The opt-in protocol discards **global alignment only**, within the existing
immutable reference/calibration/content contract. It cannot change the CRS/
origin or call local `ResetStopped`; that local API still requires independent
stopped-wheel evidence absent from this host. GNSS/map never request a reset.
Default profiles create no reset readers and retain their existing behavior.

Append these fields to enable a synthetic two-consumer example:

```text
reset_topic: "/openloc/diagnostic/global_reset"
alignment_session: "unique-host-run"
reset_consumers: "planner"
reset_consumers: "observer"
```

Choose a fresh `alignment_session` for every process run (not the ODOM session).
Generation starts at one. `NativeGlobalResetRequest` on `reset_topic` contains
that session, `expected_alignment_generation`, positive monotonic `sequence`
and an explicit `reason`. A matching request retires the old global owner,
including queued observations/history, increments alignment generation and
withdraws XY/full poses. A replacement is created only after destroying the
old owner, with the same configuration; no competing estimator runs.

Diagnostic output repeats `alignment_session`, `alignment_generation`,
`reset_sequence`, `reset_reason`, `reset_pending` and named
`pending_reset_consumers`, even without a usable pose. Each configured consumer
must retire prior-generation caches before sending `NativeGlobalResetAck` to
`RESET_TOPIC/ack`: session, new generation, reset request sequence and
`consumer_id`. Until **all** named consumers confirm, output is unusable with
`WAITING_FOR_RESET_CONSUMERS` and absolute inputs are explicitly rejected.
No timeout implicitly acknowledges a missing consumer. This is a diagnostic
contract, not an implementation of production planner/controller retirement
or authenticated operator authorization.

Identical last-request retries and duplicate valid consumer ACKs are
idempotent. Conflicting/stale requests, another request during a pending reset,
unknown consumers and wrong session/generation/sequence ACKs are rejected and
journaled. Requests need not stop local ODOM: it continues unchanged during
the barrier. Clearing the barrier does not restore the retired global estimate;
fresh multi-observation verification/acquisition is required.

When enabled, every prepared GNSS/map observation must carry the matching
`alignment_session` and `alignment_generation` in addition to its original
ODOM epoch. Old or missing tokens are rejected, never relabelled. Measurement
time must advance beyond the latest ODOM frontier at final consumer ACK.
Raw GGA has no global token on wire; the worker clears framing and receiver
dwell on reset/final ACK and rejects pre-frontier measurements before dwell.
New absolute evidence cannot reuse old pending alignment or receiver history.
Correction/owner-decision counters may restart, so consumers must key them by
alignment session/generation. These global fields are separate from ODOM
`session`/`generation` and do not imply new local origin, qualified graph or
trust. Normal ODOM epoch retirement remains independently visible; it is not
silently converted into a local reset request.

Worker and separate-process regressions exercise withdrawal, two named
consumers, stale/missing tokens, duplicate/conflicting requests, old queued
map retirement, raw framing/dwell reset and actual reacquisition. The maintained
replay accounts a synthetic 24-event sequence, with one map commit in each
alignment generation and unchanged ODOM generation. The two unchanged
prepared capture controls still commit 3/10 corrections. Evidence:
`pkgs/openloc/.agents/notes/native_global_reset_host_20261026_v1/audit.json`.
This completes the bounded diagnostic reset/ACK function, not production
consumer adoption, frame-origin migration, accuracy or live deadline acceptance.

`OUTPUT_TOPIC/processed` reports individual processing outcomes, not live
delivery/deadline qualification. The test-only `native_global_cyber_replay`
uses these acknowledgements to rehost a bounded prepared stream:

```text
native_global_cyber_replay PROFILE.pb.txt EVENTS.jsonl NEW_OUTPUT
```

Each line is `odom JSON`, `gnss JSON`, opt-in `raw_gga JSON` (RawData, with
base64 `data`), opt-in `map JSON`, `reset JSON` or `reset_ack JSON` using the protobuf JSON
schema, in declared source receive order. This is ACK-paced, not open-loop
live acceptance. Worker tests cover acquisition/tracking, held/stale,
unavailable local output, rejects/duplicates, epoch retirement/reacquisition,
malformed input and deterministic overflow. The separate-process test covers
actual typed Cyber delivery, correction publication and retirement; it waits
for both input and output discovery before sending the first observation.
Map coverage adds map-only initialization, GNSS-horizontal-to-map handover,
both-source correction revisions, queued/projected observations, ambiguous/
malformed/reference/duplicate/retired input rejection, and full-pose expiry
without renewal from partial observations. Journals identify each owner
decision's absolute source and declared dependencies. `processed_events`
is a publication watermark including rejects, not a correction count; the
replay waits for its final event's watermark so a final map/GNSS correction
cannot be mistaken for an older publication at the same ODOM sequence.

Frozen raw-native ODOM/GNSS gate outputs from two 30-second captures have also
been rehosted through typed Cyber: all 6,029/6,030 input events are accounted,
with 3/10 actual owner corrections and partial publications. This uses explicit
uncalibrated host covariance, not the retained native covariance. It is not a
new raw-sensor algorithm replay, precision comparison or live 100 Hz result.
Evidence is retained under
`pkgs/openloc/.agents/notes/native_global_host_20261026_v3/`.
After map admission was added, those two no-map inputs still yield 3/10
corrections. A separate synthetic prepared-map/GNSS replay yields two map
commits plus one GNSS commit and full diagnostic publication. This proves
host wiring, **not** real-data map-matcher performance; the mixed-source
cross-process regression is also synthetic. Current wiring evidence:
`pkgs/openloc/.agents/notes/native_map_host_20261026_v1/audit.json`.
Raw GGA adaptation now also consumes 29 original checksum-valid sentences
per capture with unchanged frozen native ODOM messages, accounting all
6,029/6,030 events and committing 1/8 actual corrections. The adapter restarts
two-second stability on native epoch establishment; this explains two fewer
corrections than the pre-admitted controls, which retain 3/10. GGA measurement
times are exact; original associated bag receive times pass through the
double RawData header and their rounding is recorded. Original raw-driver
packet arrival/chunking was not recorded by this exercise. Synthetic separate-
process cases cover recorded mode and the current driver's headerless live
format; unit cases cover framing, checksum, midnight, quality, gap/epoch reset
and explicit incomplete shutdown. Evidence:
`pkgs/openloc/.agents/notes/native_raw_gga_host_20261026_v2/audit.json`.
This is a raw-GNSS adapter replay over frozen ODOM, not a new raw IMU/LiDAR
run, accuracy result or live cadence/deadline acceptance.
Live scan-to-map scheduling, production reset-consumer adoption,
target conditional-error graph and live overload qualification
remain separate implementation/acceptance work.

## Local input clocks and recorded replay

The IMU executable requires exactly three arguments:

```text
native_imu_diagnostic IMU_TOPIC MEASUREMENT_TO_UNIX_OFFSET_SECONDS SESSION
```

Set `CYBER_PATH` to the deployed Cyber configuration directory. Supply the
sensor's verified epoch/time conversion explicitly: the IMU schema labels
measurement time as GPS epoch, which must not be silently treated as Unix.
No leap-second offset is guessed. The offset is an epoch conversion, **not**
permission to shift stale bag measurements into freshness. Input vectors must
already be calibrated FLU specific force and angular rate at the IMU origin.
Sessions use ASCII letters, digits, `_` or `-`; choose a new session for a new
run. Stop with SIGINT. Do not run this experimental quiet-window frontend as
vehicle control localization.

The LiDAR executable defaults to live wall-clock receive timestamps.
`recorded_receive_time_from_header=true` is an explicit **recorded diagnostic**
mode: it requires original Unix receive time and positive source sequence in
each sensor message header. It does not relabel old data as current. Outputs
carry `recorded_input=true`; their age is source measurement-to-original-receive
plus host-local steady elapsed time, not a live wall-clock or sender-lateness
qualification. Never use this mode for live admission.

Recorded mode acknowledges **worker enqueue**, not algorithm commit, on
`IMU_TOPIC/recorded_ack` and `LIDAR_TOPIC/recorded_ack`. A bounded driver can
retry the identical last message until admitted. Identical retries are
idempotent; conflicting payloads with the same sequence, sequence gaps and
missing source headers fail explicitly. Acknowledgements are written outside
the ingress mutex. Live mode does not enable this protocol.
On clean shutdown, `.ingress.csv` records admitted IMU/cloud counts and the
recorded-mode flag; it is not complete live-transport drop accounting.

Three boundary cases cover source preservation/freshness, epoch isolation and
invalid geometry. A real native-owner/Cyber test covers new/held/stale and epoch
delivery. Two independent-process cases exercise raw IMU epoch conversion,
diagnostic TF, stale output, clean shutdown and explicit malformed-input
failure. No synthetic result establishes recorded-data accuracy or deadlines.
Evidence: `.agents/notes/native_cyber_validation_20261023_v1/`.
The frozen cross-process arm submitted 560 synthetic IMU messages and received
52 new samples, 19 held samples, one stale record and 52 diagnostic TF messages;
maximum reported publication interval was 10.101 ms. These counts include
startup and source starvation, not 560 pose updates or a deadline pass.

## Recorded-data worker adapter

The test-only `native_lio_recorded_replay INPUT PROFILE.pb.txt NEW_OUTPUT`
consumes the existing prepared raw IMU/point CSV contract, including the
original integer Unix measurement/receive times, and exercises the worker
together with the real Cyber timer. It paces ingress by original receive time;
it does not rebase old measurements to wall-clock freshness. Profile offsets
must be zero because the prepared inputs already use Unix epoch. This adapter
bypasses Cyber sensor readers and does not count subscriber delivery; the
separate-process synthetic test covers that input boundary.

Use the same optimized compilation mode as the frozen native replay when
comparing recorded runtime. A fastbuild arm overflowed explicitly near scan 32
of the first capture; the failed journal remains in
`.agents/notes/native_lio_cyber_validation_20261023_v1/`.
Do not interpret this unoptimized failure as a deployment-performance result
or expand the queue to hide it.

The optimized arm consumed all three prepared 30-second captures without
overflow: 6,000/6,000/5,999 source states and 300 scan decisions per capture.
All ten pose/velocity components, source metadata, validity, covariance-model
flags and integrity reasons equal the frozen native baseline exactly; scan
source/reference frontiers and decision reasons also match. No numerical
tolerance was introduced. Thirteen cases cover publication, native/Cyber
delivery, three separate-process raw-input cases, four worker cases and two
strict profile cases. The LiDAR process case submitted 640 IMU messages and
32 clouds, consumed usable untrusted output, and journaled geometry and LIO
acceptance. Discovery/Write are not consumption acknowledgements; that test
uses bounded actual-output completion rather than assuming a fixed send count
guarantees initialization.

Frozen evidence: `.agents/notes/native_lio_cyber_validation_20261023_v2/`.
This establishes recorded **worker** parity, not recorded Cyber sensor-reader
acceptance, subscriber delivery/deadlines or indoor/outdoor accuracy.

## Actual recorded Cyber reader exercise

The test-only `native_lio_cyber_replay INPUT PROFILE.pb.txt NEW_OUTPUT`
publishes prepared raw sensor messages to a separately running
`native_lio_diagnostic`, waits for recorded admission acknowledgements, and
subscribes actual diagnostic ODOM/TF. It preserves source timestamps and logs
transport submission count, sender lateness, received source identity/age,
publication intervals and pose/velocity. Raw CSV parsing is shared with the
direct worker adapter. It is not an open-loop realtime performance runner.

All three 30-second captures passed this separate-process chain with 50 ms
delayed LIO: exactly 6,000/6,000/5,999 admitted IMUs and 300 admitted scans each,
with every scan decision accounted and original source/reference frontiers
equal to the direct worker arm. There were 178/122/158 actual LIO correction
commits and 25/12/2 explicitly discarded late tasks. Received usable **new**
ODOM counts were 1,705/1,196/1,340, not fresh 100 Hz throughout startup/run.
Maximum reported timer intervals were 10.424/10.213/12.019 ms; sender lateness
reached 199.893/117.861/206.745 ms. These are observed limitations, not passes
against a qualified deadline budget. No queue, geometric gate or deadline was
expanded to hide them.

Eighteen cases now cover the original boundaries plus delayed catch-up,
uncommitted shutdown, original recorded clocks/replay labels, idempotent
retries and malformed/conflicting protocol inputs. The first unacknowledged
reader arm lost early scans and later overflowed; it remains in
`.agents/notes/native_cyber_reader_20261023_v1/`. The acknowledged, explicitly
labelled arm is frozen in `native_cyber_reader_20261023_v3/`: `audit.json`,
`verification.json`, XML/logs and `index.html` local-trajectory overlays.
Its 0.135/0.145/0.154 m maximum differences from the synchronous local baseline
are scheduling/ablation differences, **not absolute accuracy errors**.

### Connected raw-native and global hosts

`native_lio_cyber_replay ... --global-topic TOPIC` additionally subscribes
the independently running global host. It requires an actual pre-input
heartbeat, original diagnostic/untrusted/recorded labels, usable alignment
and the final native source frontier through **both** subscribers.
`global_publications.csv` records actual global capabilities and correction
identities. It never writes ODOM: the global reader consumes the running
local process, not frozen trajectory rows.

`--raw-gga INPUT.jsonl --gga-topic TOPIC` optionally sends original recorded
`RawData` protobuf JSON packets in receive order interleaved with the same
raw IMU/cloud window. Their header identities and payloads are unchanged.
The input is bounded and requires ordered, finite receive clocks inside the
IMU window. Reconstructed original sentences and double-header receive-time
rounding are not raw-driver chunking or exact integer receive-time evidence.
These packets are sent once, not retried as local ACK protocol messages.

For map replay, use `--map-cloud-topic TOPIC` with a **different** topic
from the local recorded cloud reader, and configure the global raw-map reader
there. Each original cloud is sent once to that topic with the same payload
and header. Local ACK retries must not be broadcast to the map owner, whose
duplicate-observation guard remains unchanged. Map delivery is not reliable
transport; original unique input accounting must be checked from its journal.
Default local-only replay behavior remains unchanged.

Actual connected third-capture raw IMU/LiDAR, relative asynchronous startup,
delayed LIO and conditional-region map acquisition preserved 5,999 IMU and
300 cloud admissions, committed 161 local LIO tasks and 139 global map
corrections, and received 1,405 full-pose publications through the final
source frontier. No GNSS, offline ODOM substitution or pose seed was used.
Evidence: `native_connected_hosts_20261026_v2/audit.json` under package notes.
Its same-capture map, declared region and identity tilt remain diagnostic
priors, not unconstrained relocalization or independent accuracy.

The second-capture actual connected GNSS-only and map+GNSS arms each retain
6,000 IMU, 300 clouds, all 29 GGA packets, asynchronous relative startup and
135 committed local LIO tasks. GNSS-only commits one horizontal correction
and never publishes full pose; map+GNSS commits 70 map and 11 GGA corrections
through the same owner and receives 1,324 full-pose publications.
Evidence: `native_connected_three_sources_20261026_v3/audit.json`.
The map arm retains its existing epoch/time-limited, same-capture,
future-informed **global-only** hypothesis; it is not seed-free acquisition.
These are functional composition runs, not a deterministic ODOM ablation,
qualified covariance, independent accuracy or live 100 Hz acceptance.

The repository-owned
`NativeLioHostTest.ActualLocalAndGlobalProcessesConsumeRawSources` now starts
all three actual processes with a bounded synthetic raw-input fixture.
It asserts 800 local IMU and 40 cloud admissions, one raw GGA event, all 40
unique map-cloud events, actual correction commits and full-pose output through
native source 800. Generated profiles, inputs and subscriber/owner journals
are retained in Bazel test outputs. This catches readiness/option wiring and
local-retry versus map-delivery regressions without requiring ignored datasets.
The full affected set has 24 passing cases; archived evidence is
`native_connected_regression_20261026_v1/audit.json` under package notes.

## Reproducible local baseline before optimization

`testdata/recorded_baseline.json` fixes the three prepared 30-second inputs,
manifest and synchronous-reference hashes, explicit 0.2 m bootstrap voxel
and 50 ms delayed-LIO budget. `tools/recorded_baseline.py` generates profiles
from each input's numeric configuration and manual extrinsic; it does not
depend on a profile in an earlier run directory. Prepared input/reference
directories are external validation assets, currently under the independent
OpenLoc checkout's ignored notes, **not shipped test fixtures**. Restore those
exact hashed assets under `--openloc-root`, or explicitly supply another
protocol via `--manifest`; missing/changed assets fail rather than substituting
synthetic data or regenerating a baseline silently.

Inside the same mapped-user managed container, after the optimized build:

```bash
python3 -m unittest discover -s tools -p '*_test.py'
env -u LD_LIBRARY_PATH CYBER_PATH=/apollo/pkgs/core/cyber \
  python3 tools/recorded_baseline.py --output NEW_BASELINE_DIRECTORY
env -u LD_LIBRARY_PATH CYBER_PATH=/apollo/pkgs/core/cyber \
  python3 tools/recorded_baseline.py --replay-mode paired --repeats 3 \
  --output NEW_COMPARISON_DIRECTORY
```

The runner refuses to overwrite evidence. It hashes inputs/references, archives
selected source/configuration plus ELF/resolved runtime libraries, checks those
identities again at completion, and saves exact commands, per-source sender
lateness, actual subscriber poses/ages, scan/task/ingress journals, `report.json`
and offline `index.html` XY comparisons. It requires all original source
clocks and scan reference frontiers, all admitted inputs and a final subscribed
source frontier, actual LIO commits, finite usable poses and diagnostic TF.
It rejects stale-as-usable, unusable-with-pose and duplicate-new publications.
Incomplete/failed runs retain their logs and cannot produce a final report.

The initial unoptimized run is preserved in
`.agents/notes/native_reproducible_baseline_v1/`: all 17,999 IMUs and 900 scans
passed the recorded chain and scan-frontier checks. New usable counts were
1,779/1,213/1,409; maximum timer intervals were 11.994/10.919/10.171 ms.
This is a **local diagnostic integration baseline**, not the full indoor/outdoor
replacement baseline, a truth-relative accuracy result or a live 100 Hz pass.

`native_lio_cyber_replay ... --prefetch-cloud` is an opt-in **replay-driver**
optimization. One future decodes the next cloud while the sender continues
IMU pacing; only the sender writes/awaits admission, and it retains the
original source ordering, timestamps and pacing origin. At most one pending
cloud and the current outgoing cloud coexist. Decode exceptions propagate
through `get()`; unwinding joins the pending decode. The default remains
serialized. This does not move host startup registration off its owner thread,
accelerate the estimator, or repair live queue-drop accounting.

Paired mode alternates serialized/prefetch order across repetitions. The
protocol declares at least three pairs per capture and a minimum 20% decrease
in the median run's p95 IMU-send lateness before reporting the sender goal
met. It also reports new usable output, LIO commits and synchronous-trajectory
differences; no deadline/geometry/queue budget is increased. A failed
performance goal stays false in the report, rather than changing its threshold.

## Remaining application work

The actual native-local/raw-map/raw-GGA composition and isolated startup
scheduling now exist. Remaining work includes the sufficient native LIO
conditional-error model for the target graph, unknown whole-map/tilt recovery,
producer restart/complete overload accounting, and production-consumer adoption
of the diagnostic reset/ACK boundary. Scenario coverage, output-envelope
failures, live 100 Hz budgets and independent accuracy are not declared passed.
No production localization topic or default is switched by these diagnostic
functional milestones.

The global executable derives its Cyber node identity from its configured
output topic. Independent diagnostic streams can coexist without Cyber's
duplicate-node shutdown; two owners of the same output stream still conflict.
