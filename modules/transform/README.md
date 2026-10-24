# Transform calibration

## API and implementation boundaries

The top-level C++ headers are the supported interfaces:

| Header | Responsibility |
| --- | --- |
| `buffer_interface.h` | Injectable read contract and backend-independent `TransformException`; no writes or tf2 templates |
| `buffer.h` | Runtime ownership, startup and explicit dynamic-history reset; application assembly selects the instance |
| `transform_query.h` | Strict frame queries, protobuf/Eigen conversion and lookup diagnostics; no implicit latest fallback |
| `timed_transform_resolver.h` | Explicit bounded hold and cache interpolation/extrapolation policy; caller-serialized lifecycle |
| `transform_broadcaster.h` | Dynamic TF publication over Cyber |
| `transform_diagnostics.h` | Diagnostic snapshots; JSON serialization is out of line |
| `ground_reference.h` | Shared nominal vehicle-ground geometry, not terrain estimation or a generic tf2 primitive |

`internal/` contains implementations, the tf2 stores, Cyber listener and resolver
cache. Buffer uses composition behind a private implementation, not public tf2
inheritance. Algorithms cannot insert raw tf2 edges or bypass ingress validation.
The listener owns transport only; the store owns graph validation, queries and
epochs. A mixed store serves timed queries and a static-only store serves rigid
extrinsics. These are distinct query views, not competing calibration sources.
History reset explicitly restores accepted static edges in the mixed store;
preservation does not depend on the vendored tf2 `StaticCache::clearList()` behavior.

`calibration/static_transform_loader.h` is a restricted interface shared by static
publication and the vehicle-calibration validator, with access for their fixture
tests. It is not part of the general query API. `components/` contains the startup
component. `tests/` contains tests and a test-only ingress peer; production targets
cannot depend on that peer. Implementation targets default to private visibility.
Existing public library labels and top-level public include paths are retained;
the former loader/component include paths and unused generic `transform<T>()`
helpers are not compatibility APIs.

The header installation uses an explicit seven-header allowlist. The separate
source delivery includes implementation, component and test sources for rebuilds;
it is not the public SDK header set. This layout keeps one Bazel package instead
of adding a subpackage for every implementation class.

The vendored tf2 remains the transform-tree engine; this refactor does not upgrade
it or introduce ROS transport. A future upstream upgrade must pin its version and
validate time/message adapters, static restoration, type conflicts and epoch
invalidation independently.

## Calibration ownership and runtime behavior

Static TF startup selects calibration through the common vehicle registry.
Transform has no module-local calibration files or legacy registry. Export the same
selection to every calibration consumer before launching any component:

```text
WHEELOS_ASSET_ROOT=<absolute read-only asset root>
WHEELOS_VEHICLE_PROFILE_KEY=vehicles/<vehicle-type>/config/vehicle_profile.pb.txt
WHEELOS_CALIBRATION_MANIFEST_SHA256=<expected manifest SHA256>
```

The profile selects a fixed version under
`vehicles/<vehicle-type>/calibration/<version>/`. Schema 2 additionally pins the
vehicle identity, manifest and every referenced asset. Never modify a running
bundle in place; switch profiles only with a coordinated consumer restart.

Validate the selected bundle in the managed development container:

```bash
bazel run --config=dev //modules/tools/vehicle_calibration:validate_calibration
```

This checks the registry, complete static graph/YAML pairs, supported perception
metadata and camera models, and declared LiDAR-height files. It exits nonzero
on error and prints vehicle/version/manifest digest on success. Run it without
local overrides against published module versions for release acceptance.
The cross-domain validator lives in `modules/tools/vehicle_calibration`;
Transform itself has no Perception dependency.

`StaticTransformComponent` publishes only after every selected extrinsic passes
validation. Missing, malformed, mismatched or non-finite extrinsics fail startup;
there is no fallback to a partial tree or module-local defaults. Static queries
compose and invert edges from a separate static-only TF store. Queries can
wait for a bounded steady-clock timeout; they never fall back to dynamic TF.
Perception uses its existing `obs_tf2_buff_size` query budget during startup.
The component validates one complete snapshot before creating its writer and
publishes it once. It does not maintain an incremental calibration cache,
merge by child frame, or support hot reload.

Receivers reject conflicting static values, static publisher module identities,
parent changes, cycles, and static/dynamic reuse of a child frame. Identical
static retransmissions from the same module are allowed (including quaternion
sign equivalence). Each incoming message is checked before inserting its edges;
rejected messages are counted in diagnostics. Module names are diagnostic
identity, not transport authentication: deployment must still ensure one selected
static publisher, and must isolate conflicting live/replay publishers.

`TransformQuery` requires an explicit `BufferInterface*`; only application
assembly selects the runtime `Buffer` singleton. Buffer initialization checks
both configured TF topics and both readers, and runtime construction fails fast
on initialization error. Rigid LiDAR consumers use bounded static-only lookup;
they never cache a dynamic latest query as an installation extrinsic. NDT uses
`T_base_lidar` for predicting the LiDAR pose and recovering the body pose.

`T_target_source` maps source coordinates into the target frame, in metres.
Static YAML quaternion squared-norm error must not exceed `1e-3`; accepted
rounding error is normalized before publishing, so downstream composition uses
unit quaternions rather than approximate rotations.

Timed queries use the same policy in simulation and reality modes. If the
requested time is unavailable, a non-hardware-triggered caller may hold the
latest sample only when its timestamp is no later than the request and its
age is within `latest_lookup_fallback_tolerance_sec`. Zero disables this
fallback. The exact checked sample is returned with its actual timestamp;
there is no second latest lookup that could bypass the age check.
This bounded hold is separate from `enable_extrapolation`, which controls the
resolver's cache interpolation/extrapolation path. Hardware-triggered callers
do not use latest-sample fallback. Explicit latest queries remain available
through `TransformQuery` with time zero.

All TF waits use steady-clock deadlines without advancing the shared simulation
clock. Buffer callbacks and diagnostic state are synchronized. Clock rollback
observed by reception or the resolver's epoch query, and explicit Buffer clear,
invalidate dynamic history while retaining static calibration. Resolvers discard
cached poses when the Buffer epoch changes, including during a lookup. Changing
the injected query or resolver options also clears that resolver's cache.
Resolvers are caller-owned and require serialized calls; diagnostic snapshots
may be read concurrently.

Resolver options have one field per budget: `query_timeout_sec`,
`cache_duration_sec`, `max_extrapolation_sec`, and
`latest_lookup_fallback_tolerance_sec`. Values must be finite and nonnegative;
zero is honored rather than replaced by a default. `SetOptions` failure prevents
resolution until valid options are supplied. Timed resolution requires a finite
positive measurement timestamp; latest queries belong to `TransformQuery`.

SensorManager consumes the selected metadata/intrinsics and preserves its
existing query API. Camera-height consumers use the selected LiDAR's auxiliary
calibration. NDT's `ndt_calibration_sensor` and Dreamview's
`dreamview_calibration_sensor` select a sensor ID/frame; empty is accepted only
for a single-LiDAR inventory. NDT's incoming point-cloud frame must match the
selected LiDAR. Dreamview accepts the selected LiDAR, unified `base_link`
clouds and other statically connected frames, transforming each into the same
ground-relative, base-aligned display frame before filtering.

Ground height is measured along nominal `base_link` +Z, not along a tilted
sensor's Z axis. Shared geometry uses
`h_sensor = h_lidar + z_base_sensor - z_base_lidar`; point clouds use the same
base-frame ground offset after their full rigid transform. This is calibration
geometry, not live terrain/slope estimation. Camera ID/link/optical-frame
adaptation and height loading are shared by the four onboard camera/lane
components rather than reimplemented per component.

Schema 1's global `lidar_height_file` remains supported for its single LiDAR
and is checked by the validator. This does not supply missing legacy perception
metadata: SensorManager still requires explicit selected metadata/intrinsics.

The supported Brown model requires
`K = [fx,0,cx; 0,fy,cy; 0,0,1]` and five distortion coefficients. Unsupported
matrix entries outside `1e-6` tolerance are rejected; accepted roundoff is
canonicalized. Onboard camera/lane and traffic-light components require a
common calibration resolution within each component. Incoming image messages
must match that resolution and the provider's supported packed 8-bit encodings
(`rgb8`, `bgr8`, `gray`, `y`), with an explicit packed row step and exact payload
length. Padded rows, implicit resizing/cropping and mismatched payloads are
rejected before copying; callers stop processing the failed frame. Supporting
other layouts or resolutions requires explicit conversion and matching
intrinsics, not reinterpretation of the buffer.

The former `CalibrationRegistry`, static-config protobuf and bundled sample
YAMLs have been deleted. No old API or Bazel target aliases are provided.
Standalone offline tools that parse historical files must receive their own
explicit input; those formats are not runtime selection authorities and the
tools must not write into published bundles. Common manifest schema handling
remains Common's responsibility, not a second Transform configuration path.

No production vehicle bundle is synthesized by this change. Deployment requires
measurement-backed assets with verified coordinate conventions and provenance.
The test-only bundle builder creates isolated temporary fixtures, never runtime
defaults. Roadlog records the selected vehicle/version/manifest identity in
each segment's header, including test-only status. Unselected capture is
explicitly marked; invalid selected calibration prevents recorder startup.
Coordinating restart/readiness across processes remains a deployment
responsibility; per-process identity logs or the recorder's identity alone do
not prove cross-process consistency.
