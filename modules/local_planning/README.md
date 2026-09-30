# Local Planning

Independent ODOM-only planning foundation alongside unchanged legacy planning.
`planning_context/` is an optimizer- and Scenario-independent library for
admission and construction of planning algorithm conditions. It consumes a
typed `LocalScene`, current measured ODOM, Prediction tags and declared
`LocalOccupancy` coverage (corridor-scoped in LANE and affirmative
drivable-region coverage in AREA); `PlanningInputBuilder::conditions()` publishes
admitted source/deadline tags, the measured start state, vehicle envelope,
dynamic-occupancy binding and either LANE reference/boundary geometry or AREA
free-space/curb/obstacle geometry. AREA contains no fabricated corridor or s/l
coordinates. Both modes require a clear measured vehicle footprint and
declared prediction coverage before conditions are marked executable.

Example lifecycle:

```cpp
PlanningInputBuilder builder(policy, requirements);
auto status = builder.BeginEpoch(stationary_odometry, now);
if (!status.ok()) return status;
status = builder.Build(scene, odometry, prediction, occupancy, now);
if (!status.ok()) return status;
const PlanningInputConditions* conditions = builder.conditions();
```

The caller supplies `InputPolicy` and `PlanningInputConfig` limits. A successful
conditions bundle is not a generated or collision-free path. Lane-follow
rollout, open-space search/optimization and speed planning remain downstream
algorithm ownership. `trajectory_continuity/` owns independent validation
and absolute-time alignment of accepted lane-follow trajectories. The
existing lane-follow simulation consumes the builder's admitted LANE cycle;
AREA geometry remains
available to a future open-space consumer while the lane-follow facade reports
that it cannot generate a trajectory in AREA mode.
`PlanScene` calls the builder and then runs three concrete Scenario Tasks
(stop decision, rollout, independent validation); direct `Plan(CycleInput)`
instead admits its provided cycle through Scenario's own gate. A successful
Scenario merely completed Tasks; only a validated `LocalTrajectory` is
executable. A failed cycle invalidates trajectory history. For AREA the facade
reports unavailability and clears its trajectory while retaining the builder's
usable geometry.
The direct and scene-based entries have separate admission frontiers. The
planner rejects switching between them within one epoch, clears history and
requires a new stationary epoch before changing entry paths.

```text
ODOM + LocalScene + Prediction
  -> planning_context/ (LANE or AREA conditions, measured ODOM origin)
  -> downstream lane-follow or future open-space generator
  -> trajectory_continuity/ (validate -> accept -> time-aligned next start)
  -> same-frame controller/supervisor
```

`reference_continuity` reports bounded overlap reuse and veto reasons without
claiming tangent or curvature continuity. `boundary_geometry` exports sampled
measured lines, not optimizer station-wise vehicle-envelope bounds; downstream
consumers use the vehicle-envelope/environment checks until a safe
vehicle-relative boundary constraint model is implemented.

Finite-view lane observations use the independent world-model temporal lane
map; optional navigation input selects only observed, permitted successors in
offline fixtures. This remains a low-speed synthetic-observation baseline, not
a production Cyber component or real-perception/control integration.

The authoritative design, ownership and implementation status are in
[`wheelos-service/context/modules/local_planning`](../../wheelos-service/context/modules/local_planning/README.md).

Build/run commands and simulation limits:
[`ODOM-only MuJoCo simulation`](../../wheelos-service/context/modules/local_planning/run/odom-only-mujoco-simulation.md).
