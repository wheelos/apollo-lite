# Planning Context

This package builds the admitted, geometric context consumed by planning
algorithms; it does not generate paths, choose speeds, or claim that a feasible
trajectory exists. Its public lifecycle is
`PlanningInputBuilder::BeginEpoch`, `Build`, and `conditions`. It depends on
world-model contracts and common mathematics, not Scenario or a planner.
The `executable_conditions` bit means only that the currently requested
input bundle passed admission, declared dynamic coverage and the measured
footprint query; it is not a validated trajectory or proof that any future
path is feasible.

| Module | Responsibility |
| --- | --- |
| `input_contract`, `input_gate` | ODOM/frame/clock/epoch admission, monotonic time and immutable source frontiers. |
| `corridor_selector` | Select the observed ego lane and permitted successors. Navigation cannot grant permission. |
| `reference_continuity` | Reuse bounded same-ODOM overlap, explain resets, and preserve current restrictions. Reusing one immutable scene retains its reference rather than repeatedly deforming it. |
| `reference_line` | Project current ODOM to reference s/l, validate resulting geometry, and extract the rear/front interval. |
| `boundary_geometry` | Sample current measured left/right boundaries at reference stations. These are not vehicle-center optimizer bounds. |
| `environment_geometry` | Vehicle-envelope and swept-region feasibility queries against affirmative free space, curbs and static obstacles. |
| `planning_geometry` | LANE reference geometry or AREA geometry without a fabricated reference. |
| `planning_input_builder` | Assemble admitted current ODOM start, geometric conditions, dynamic-occupancy binding, vehicle envelope and validity deadline; own accepted reference history. |

World-model observations are already transformed at measurement time:
`p_odom = T_odom_base(t_measurement) * p_base`. This package uses the current
ODOM state at the planning reference time. It does not transform old
observations again, re-estimate ODOM, or splice coordinate epochs.

Both modes require healthy current ODOM and declared dynamic coverage. LANE
provides an optional reference with stations, signed ego lateral offset and
current boundaries. AREA provides the same ODOM start and region constraints,
but no s/l coordinates. Losing required evidence clears the usable bundle.
Changing mode or epoch clears reference continuity. An admitted source remains
immutable even when geometric construction fails: the builder stores the full
accepted scene at admission and a failed geometry build does not roll back the
source frontier. A rejected source does not advance that frontier. The output
deadline is the minimum of the admitted source deadlines.

The current reference-continuity algorithm bounds position changes; it does
not promise tangent/curvature continuity. General vehicle-relative SL bounds,
multiple candidate corridors and open-space search remain future algorithms.
Time alignment of previously accepted execution trajectories is owned by the
separate `trajectory_continuity/` package. Unknown space is never made
drivable to manufacture a planner input.
`PlanScene` is the lane-follow facade's builder-backed entrypoint; its LANE
cycle is passed to private Scenario task execution without a second admission.
The older direct `Plan(CycleInput)` calls Scenario's separate `InputGate` and
does not enforce full-scene equality. The planner rejects switching between
these entrypoints within one epoch and clears cycle/trajectory state. Do not
use the direct entrypoint as a substitute for this builder when consuming
`LocalScene`.

Canonical architecture and extension status:
[`independent-local-planning-plan.md`](../../../wheelos-service/context/modules/local_planning/knowledge/independent-local-planning-plan.md).
