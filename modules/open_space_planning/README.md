# Open Space Planning

`modules/open_space_planning` is the grid-map planning stack. It is independent
from the HDMap/reference-line planning stack in `modules/planning`.

## Pipeline

```text
PlanningProblem
  -> RoutePlanner             (topology and geometric corridor)
  -> TrajectoryPlanner        (time-parameterized physical trajectory)
  -> TrajectoryValidator      (independent hard safety gate)
  -> PlanningResult
```

If all route candidates fail, `FallbackPlanner` may produce a braking or hold
trajectory. The fallback must pass the same safety validator before it can be
returned.

## Package ownership

| Package | Responsibility |
| --- | --- |
| `common/` | Module-owned domain types and status |
| `lattice/` | Reusable lattice kernels; no runtime orchestration |
| `route/` | Layer-1 route-lattice SPI and implementations |
| `trajectory/` | Layer-2 physical-trajectory SPI and implementations |
| `safety/` | Final validation and fallback SPIs |
| `runtime/` | Pipeline orchestration and candidate lifecycle |
| `component/` | Cyber transport adapters only |

## Dependency direction

```text
component -> runtime -> route
                     -> trajectory
                     -> safety

route/trajectory implementations -> lattice -> common
```

Algorithm packages must not depend on `component/` or `runtime/`.

The module must not depend on HDMap, routing, `Frame`, `ReferenceLineInfo`,
`PlanningContext`, tasks, scenarios, or `PublicRoadPlanner`. During migration,
truly generic math/trajectory utilities may temporarily be referenced from
`modules/planning`, but no lattice implementation may be referenced there.

## Migration plan

The migration is intentionally staged so the public-road planner remains
functional while the open-space stack becomes the owning module for the
lattice implementation.

1. Freeze the public-road planner as the compatibility baseline.
2. Move reusable, map-independent lattice kernels into
   `modules/open_space_planning/lattice/`.
3. Move route-search and candidate-generation logic into
   `modules/open_space_planning/route/` and
   `modules/open_space_planning/planner/` only when the algorithm is no longer
   tied to `ReferenceLineInfo`, `Frame`, or HDMap routing.
4. Keep `modules/planning` as a temporary dependency only for generic geometry,
   vehicle model utilities, and shared non-map infrastructure.
5. Remove direct dependency on old lattice code from the new package before the
   first full integration test.

The immediate ownership target is:

```text
modules/open_space_planning/
  common/
  lattice/
  route/
  trajectory/
  runtime/
  safety/
```

The legacy `modules/planning/lattice` tree remains available only as a
compatibility source during migration; new code under this repo must not add
new imports to it.

## Extension rule

New algorithms implement an existing SPI. Do not add algorithm-specific fields
to `OpenSpacePlanner`; extend the request/result domain contracts only when the
data is meaningful to every implementation of that stage.

