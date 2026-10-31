# OpenLoc Apollo localization

This module embeds `pkgs/openloc` through the existing bounded
[`native_runtime`](../openloc_localization/BUILD) host libraries. One raw
IMU/LiDAR owner produces continuous local motion; one explicitly selected
`FixedLagGlobalAlignment` owner consumes its joint motion, shared-scan map
matches and GNSS. Global corrections never feed back into local ODOM.
No diagnostic-baseline fallback, second solver or GNSS-odometry substitution
is used. No public relative-ODOM topic or planning/control integration is
provided in this first version.

## Inputs and outputs

| Channel/config | Type and contract |
| --- | --- |
| `local.imu_topic` | `apollo.drivers.gnss.Imu`: calibrated instantaneous FLU acceleration (including gravity) and angular rate; explicit source-to-Unix offset and frame |
| `local.lidar_topic` | `apollo.drivers.PointCloud`: raw LiDAR-frame XYZ, ordered absolute per-point nanoseconds, scan-end measurement time; not compensated clouds |
| `global.gnss_topic` | `apollo.drivers.gnss.GnssBestPose`: receiver status/uncertainty, explicit source clock, UTM projection and IMU-to-antenna lever arm |
| Optional `structured_gnss.heading.topic` | `apollo.drivers.gnss.Heading`: independently configured dual-antenna baseline |
| `/apollo/localization/pose` | `LocalizationEstimate`: absolute UTM rear-axle position, RFU-to-world quaternion, heading, velocity and angular rate |
| `/apollo/localization/msf_status` | `LocalizationStatus`: initialization/unavailable/stale/error or diagnostic warning |
| TF | Same measurement time, calibrated FLU body pose; common defaults to `map -> base_link` |

Output names are configurable and match NDT's standard interfaces. Do not run
NDT/RTK and this module as simultaneous writers of the same pose/TF.
`/apollo/sensor/gnss/odometry` is deliberately **not** a local-motion input:
it is an absolute INS solution, not independent continuous ODOM.

The map PCD must use gravity-up, UTM-grid-aligned XYZ, with XY reduced by the
configured `structured_gnss.origin_easting_m/origin_northing_m`. Its height is
relative to `utm_origin_height_m`, in the explicitly named `vertical_datum`.
The publisher restores these offsets. It does not infer a datum, perform
geoid conversion, or rotate an arbitrary map into UTM.

The selected calibration's `base_link` and rear-axle frame use
**forward/left/up** axes. `vrp_frame` defaults to `base_link`: this requires the
selected vehicle to place its body origin at the rear axle. Otherwise select
the calibrated rear-axle FLU frame from that same static graph.
Both orientation and the translational lever arm are applied. Rear-axle velocity includes
`R_map_imu * (angular_velocity_imu x translation_imu_vrp)`, rather than simply
copying IMU-origin velocity. Linear acceleration and uncertainty are omitted:
the host does not supply qualified rear-axle acceleration/covariance.

TF publishes `T_parent_base_link`, mapping body coordinates into the global
parent. It retains FLU axes, while the standard `LocalizationEstimate` quaternion
maps **RFU** into the global frame. That fixed axis conversion is applied only
at the message boundary, not stored as a sensor calibration or copied into TF.
The parent/child defaults are resolved from common's
`localization_tf2_frame_id` / `localization_tf2_child_frame_id` (`map` /
`base_link`); `world` is not an implicit alias for `map`. A different explicit
parent requires the same physical UTM reference contract in every consumer.
No invented `world -> map` or `map -> odom` identity is published.
The future relative pipeline must replace this direct absolute-only TF edge
with the single-owner `map -> odom -> base_link` tree, not add a competing edge.

## Configuration and startup

[`conf/localization.pb.txt`](conf/localization.pb.txt) is intentionally
non-runnable. Supply sensor selectors and site/algorithm values, then set
`deployment_configured`. Common `MapSelection::GetSelectedMap` supplies the
selected directory and map identity (the same KVDB/resource selection as NDT).
`global.online_map.map_pcd` must be relative to that directory; absolute paths,
parent traversal and symlinks escaping the selected directory are rejected.

Common's `vehicle_calibration::Registry` selects the immutable calibration via
`WHEELOS_ASSET_ROOT` and `WHEELOS_VEHICLE_PROFILE_KEY`, honoring its test-only and
manifest-digest gates. `imu_sensor`, `lidar_sensor` and `gnss_sensor` are required
sensor IDs or frames in that inventory; optional heading also requires
`secondary_gnss_sensor`. Start the existing `StaticTransformComponent` for the
same selected profile before localization. Bounded static-only `TransformQuery`
lookups supply LiDAR-to-IMU, body/rear-axle-to-IMU, antenna lever arm and optional
dual-antenna baseline. There is no module-local extrinsic YAML parser or
dynamic/latest fallback.

Deployment text deliberately omits host-required resolved fields. Only this
parse permits partial messages; selection/TF resolution fills those fields,
then full protobuf and runtime validation is mandatory. Inline extrinsics,
resolved frame/calibration IDs and a manual map identity are rejected, not
silently overridden. Changing the selected map or vehicle profile requires a
coordinated restart; there is no hot reload.

Startup rejects absent configuration, mismatched local/global calibrations,
unsupported recorded-clock/reset modes, missing maps, bad hashes and existing
journals before creating sensor readers. Runtime source gaps (when enabled),
invalid sensor fields and worker failures become terminal errors; restart with
a fresh session and fresh journal paths after correcting the cause.

Required deployment inputs:

- Actual raw topics, selected sensor IDs, source clocks and IMU noise/sample
  period. Rigid transforms and frames come only from common/static TF.
- Versioned bounded PCD, SHA256, UTM zone/hemisphere/origin and height datum.
  NDT tiled maps are **not** accepted directly. The reused map loader accepts
  uncompressed float XYZ/XYZI PCD up to 128 MiB and two million points.
- Explicit global-only startup `initial_map_from_odom` hypothesis, its Unix
  validity interval, and matching local session/generation. This seeds map
  search only; it cannot initialize local ODOM. GNSS XY alone does not establish
  the full absolute pose.
- Native motion model identity and sufficient-model declaration; GNSS
  uncertainty/dependency evidence; deterministic-map and absent GNSS/map
  construction dependency declarations. Do not turn unknown dependencies into
  independent factors merely to enable publication.
- New writable journal paths and scenario-appropriate queue/history/solve
  budgets. The template's numeric profile is not a calibrated vehicle profile.

Build in the managed dev container as the mapped invoking user:

```bash
bazel build --config=dev -c opt //modules/localization:liblocalization_component.so
bazel test --config=dev -c opt //modules/localization/... \
  --test_env=CYBER_PATH=/apollo/pkgs/core/cyber --test_output=errors
```

After configuration, the Cyber entrypoint is
`cyber_launch start /apollo/modules/localization/launch/localization.launch`.
The DAG timer samples output every 10 ms; it does not extrapolate states or
guarantee a 100 Hz fresh estimate. Worker threads own local and global compute.
Internal graph packets are passed in-process, not published as public ODOM.

## Availability boundary

Only fresh, full absolute graph poses with exact matching local source
time/sequence/epoch produce `LocalizationEstimate` and TF. Uninitialized,
horizontal-only, stale, failed or unmatched results publish status only.
Held samples do not refresh measurement time or repeat TF.

The reused native graph/ICP pipeline remains **diagnostic-only and uncalibrated**.
Published poses carry warning status and explicitly non-good MSF quality; they
are never labeled `OK`, normal or trustworthy. Loss of full-pose evidence
suppresses absolute output even when local ODOM continues. Existing graph
capacity/model-admission limits remain visible, not replaced with configured
covariance. Raw dataset accuracy, real-time operation, vehicle deployment,
and planning/control acceptance are not established by this integration.

## Dependencies

Development resolves `openloc` through the opt-in root `.bazelrc.dev` override.
`openloc.MODULE.bazel` carries the tested Eigen/GTSAM patches and consistent
Boost 1.89 closure required by GTSAM/PCL; these are root-only Bazel overrides
and affect the workspace dependency graph, not just this module. The reused
host sources under `modules/openloc_localization` are part of the delivery.
No registry version has been published, and the formal lockfile is unchanged;
non-development/release dependency resolution is not claimed.
