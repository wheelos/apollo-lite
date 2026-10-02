# Localization Health

`localization_health` independently validates the local ODOM stream, estimator
assessment, aggregate capabilities, and directional constraints. It publishes
health status and transition events without republishing a global pose.

## Channels

- Local ODOM input: `/apollo/localization/health/local_pose`
- Assessment input: `/apollo/localization/health/assessment`
- Health status: `/apollo/localization/health/status`
- Transition event: `/apollo/localization/health/event`

## Session identities

`LocalizationHealthStatus.session_id` and `LocalizationHealthEvent.session_id`
identify the accepted producer/ODOM-generation session. A changed producer
session is accepted only when both assessment time and local measurement time
advance beyond the committed frontier. The previous producer session is then
retired; late assessments from bounded recent retired sessions are rejected,
logged, and reported as `REASON_SESSION_CHANGED` without resetting live health
state. The monotonic time frontier also rejects genuinely late sessions after a
retired-session identifier ages out of the bounded set.

`supervisor_session_id` is a separate process-instance identity generated once
from the supervisor process ID and steady-clock ticks. It remains stable across
producer session changes and configuration initialization.

## Profiles

- `conf/localization_health.pb.txt` is the default map profile and requires
  qualified global pose evidence for nominal operation.
- `conf/localization_health_odom_only.pb.txt` requires neither global nor lane
  evidence for nominal operation.
- `conf/localization_health_lane_keeping.pb.txt` requires authoritative lane
  lateral and heading constraints, but no global pose.

## Directional constraints

`DirectionalConstraint.projection` contains exactly six coefficients. Rows act
on `[delta translation, left delta rotation]`, with both perturbations expressed
in `reference_frame`. `reference_id` identifies the fixed reference point and
relation used to define the row. Local body-attitude axes must be rotated into
ODOM by the assessment producer before publication.

For `unit: "meters"`, translation coefficients are dimensionless and rotation
coefficients carry the geometric length scale in meters per radian. This allows
coupled position/rotation map eigen-directions. For `unit: "radians"`,
translation coefficients must be zero and rotation coefficients are
dimensionless.

Each row is validated independently. It requires a nonzero finite projection,
known unit and reference, positive finite standard deviation within its error
budget, fresh evaluation and validity times, a matching ODOM epoch, and a
fresh numeric continuous local ODOM stream. Full-position uncertainty
thresholds do not invalidate an otherwise qualified direction.

Heading uncertainty in health status comes from
`LocalizationAssessment.yaw_std`. Pose quaternion-component standard
deviations are checked only for finite, nonnegative covariance values and are
not interpreted as Euler yaw uncertainty.

`DIRECTION_CONSTRAINED` requires independent sensor evidence. Recovery requires
at least `direction_recovery_frames` distinct, strictly increasing observation
sequences and timestamps over `direction_recovery_min_span`; defaults are three
frames and 0.1 seconds. The recovery key includes session, ODOM generation,
identity, reference, source and unit. Projection association ignores sign and
scalar representation changes, allowing only numerical direction differences
(1e-8 after maximum-coefficient normalization). It compares against the original
recovery direction, not a drifting frame-to-frame anchor. Actual direction
changes, evidence loss, expiry, malformed rows or epoch changes revoke the
direction immediately. Duplicate row identities in one assessment are rejected,
not counted as multiple observations. The original row, uncertainty and error
budget remain unchanged in the output.

`DIRECTION_PROPAGATED` represents an estimate propagated without a new
independent observation. It is published as propagated and never promoted to
measured/constrained.

For lane keeping, `lane-lateral` with projection
`[0, 1, 0, 0, 0, 0]` meters and `lane-heading` with projection
`[0, 0, 0, 0, 0, 1]` radians must both be currently constrained and share the
same reference frame, reference ID, and source. Equivalent nonzero scalar
multiples are accepted with the producer's consistently scaled uncertainty and
budget. This pair can satisfy lane-level
nominal quality when full ODOM position uncertainty exceeds the nominal
threshold, but hard time, numeric, or continuity faults still revoke all
directions.

`lane_level_valid` means that the independent lateral and heading relationship
is qualified with bounded uncertainty. It does not mean that the vehicle
footprint is contained by a lane polygon and does not grant driving permission.
A measured lateral and heading pair may qualify the relation-only LANE_KEEPING
profile at an explicit forward reference station; rear or full-footprint
coverage is not required for this health capability. Planning or the world
model must separately require its containment and obstacle evidence, including
`LaneRelativeState.containment_valid` and `containment_reason` where that
interface is used.
