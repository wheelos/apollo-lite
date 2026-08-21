# Open Space Planning Architecture

## Public boundary

The stable C++ API is `//modules/open_space_planning:open_space_planning`.

```cpp
Status OpenSpacePlanner::Plan(
    const PlanningProblem& problem,
    PlanningResult* result);
```

`PlanningProblem` is an immutable planning-cycle snapshot. The component layer
must finish timestamp, frame, map-revision, and goal-revision alignment before
calling the planner.

`PlanningResult` is returned only after final safety validation. An OK status
therefore means the trajectory is eligible for publication, not merely that a
search algorithm found a path.

## Stage SPIs

### RoutePlanner

Input:

- grid-map snapshot,
- start and goal state,
- vehicle model,
- requested maximum candidate count.

Output:

- best-first, topologically distinct `RouteCandidate` values.

Required invariants:

- complete vehicle footprint is collision-free,
- path is kinematically connectable,
- corridor contains the skeleton,
- gear segments cover valid skeleton index ranges,
- map and goal revisions equal the request revisions.

### TrajectoryPlanner

Input:

- one validated planning problem,
- one route candidate.

Output:

- one time-parameterized physical trajectory.

Required invariants:

- source candidate ID is preserved,
- route topology and corridor are preserved,
- `relative_time` and geometric `s` are monotonic,
- gear and signed velocity agree,
- gear switches occur only through a legal near-stop transition,
- map and goal revisions equal the request revisions.

### TrajectoryValidator

The validator is independent from the two planners. It checks the selected
trajectory against the latest accepted snapshot and returns all detected
violations in a `ValidationReport`.

It must check:

- swept-footprint grid collision,
- dynamic-obstacle space-time collision,
- unknown/no-drive/out-of-map policy,
- curvature, steering, velocity, acceleration, and jerk,
- start continuity and inter-point continuity,
- gear/velocity and time/station contracts,
- stopping and terminal validity.

### FallbackPlanner

The fallback planner generates braking or hold trajectories only. It does not
perform route search. Its output passes through the same validator and revision
checks as a normal trajectory.

## Runtime sequence

```text
1. validate PlanningProblem
2. request best-first RouteCandidate list
3. for each current-revision candidate:
     a. generate PhysicalTrajectory
     b. reject empty/stale output
     c. run independent TrajectoryValidator
     d. commit first safe trajectory
4. when no candidate succeeds:
     a. generate fallback
     b. run the same revision checks and validator
     c. commit only when safe
5. otherwise return an explicit failure status
```

There is no success-shaped fallback and no unsafe historical trajectory reuse.

## Cyber component contract

The future `component/` adapter should be a timer-driven component because grid
maps and goals change independently from localization/chassis/prediction.

Readers:

- localization,
- chassis,
- prediction/perception obstacles,
- occupancy/cost grid,
- open-space goal.

Writers:

- dedicated open-space `ADCTrajectory`,
- open-space planning status/debug.

The adapter owns:

- input buffering and synchronization,
- protobuf/domain conversion,
- planner lifecycle and configuration,
- output protobuf conversion,
- publication and metrics.

It must not own search, trajectory generation, validation, candidate switching,
or fallback algorithms.

The public-road and open-space components must not concurrently write the
control-consumed topic. Deployment must use launch-time exclusion or an
external output arbiter.

## Implementation assignments

Independent agents may implement these packages in parallel:

| Work package | Stable integration point |
| --- | --- |
| Grid validation, transforms, distance field | `PlanningProblem::grid_map` |
| Route lattice and Top-K | `RoutePlanner` |
| Corridor construction/topology classification | `RouteCandidate` |
| Local path frame and trajectory lattice | `TrajectoryPlanner` |
| Dynamic obstacle projection | `PlanningProblem::dynamic_obstacles` |
| Hard safety checks | `TrajectoryValidator` |
| Brake/hold generation | `FallbackPlanner` |
| Cyber transport | `OpenSpacePlanner::Plan` |

Agents must not bypass these interfaces by adding cross-package mutable
singletons or depending on public-road planning runtime objects.

## Migration boundary

The original public-road planner remains operational while migration occurs.
New lattice implementations belong only in `modules/open_space_planning`.

Allowed temporary reuse:

- generic geometry/math,
- vehicle configuration,
- generic trajectory containers or utilities after API review.

Forbidden reuse:

- `modules/planning/lattice`,
- `Frame`,
- `ReferenceLineInfo`,
- HDMap/routing,
- planning tasks/scenarios,
- public-road planning gflags as algorithm input.

If both planning stacks later need a migrated kernel, move that kernel to
`modules/common/planning` in a separate change and make both stacks depend on
the neutral owner.

