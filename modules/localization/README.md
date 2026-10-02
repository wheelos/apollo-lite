# Unified localization

The localization runtime separates continuous local motion from global
registration:

- `LocalLocalizationComponent` owns `odom -> base_link`. IMU and wheel
  propagation continue across indoor, outdoor, tunnel, and GNSS-denied
  transitions. Global observations must not reset or correct this local state.
- `GlobalLocalizationComponent` owns `map -> odom`. GNSS, map matching, and
  recovery observations update this alignment with their time, reference, and
  uncertainty; they do not publish a competing local pose.
- `LocalizationHealthComponent` and the localization health bridge publish
  independent directional assessments. A partial observable constraint is not
  promoted to a full-pose or TF claim.

The runtime uses the `LocalOdometry`, `GlobalLocalization`, and directional
assessment contracts in `proto/unified_localization.proto`. The default launch
combines local, global, and health processes. `localization_odom_only.launch`
and `localization_lane_keeping.launch` select reduced profiles. Runtime
configuration templates intentionally require deployment calibration and
measurement budgets where those values cannot be inferred safely.

The legacy MSF estimator is retired. Standalone NDT map support and point-cloud
IO remain under `modules/ndt_localization/map_support` and
`modules/localization/common/pointcloud_io`; they are not an alternate global
localization authority. The historical `/apollo/localization/msf_status` channel name remains as a
compatibility interface for existing RTK, standalone NDT, and monitoring
consumers.

The implementation and unit-test baseline does not establish vehicle
qualification. See
`wheelos-service/context/modules/localization/knowledge/directional-localization-implementation-plan.md`
for remaining model, replay, and vehicle acceptance gates.
