# Open Space Planning

`modules/open_space_planning` provides mapless / grid-map open-space motion planning. It is independent from HDMap reference-line planning.

## Pipeline Architecture

```text
PlanningProblem
  -> RoutePlanner             (Route search & safe flight corridor: SkeletonCorridor / Hybrid A*)
  -> TrajectoryPlanner        (Curvature-constrained SQP smoother & piecewise-jerk speed planner)
  -> TrajectoryValidator      (Independent hard safety gate & swept-box collision checker)
  -> PlanningResult
```

## Package Structure & Responsibilities

| Package | Responsibility |
| --- | --- |
| `common/` | Domain types (`PlanningProblem`, `RouteCandidate`, `PhysicalTrajectory`), status, and problem validators. |
| `route/` | Route search SPI (`RoutePlanner`) and search paradigms (`SkeletonCorridorRoutePlanner`, `HybridAStarRoutePlanner`). |
| `trajectory/` | Trajectory generation SPI (`TrajectoryPlanner`) with SQP curvature-constrained path smoothing and piecewise-jerk speed profile optimization. |
| `safety/` | Independent trajectory validation (`TrajectoryValidator`) and emergency fallback (`FallbackPlanner`). |
| `runtime/` | Pipeline orchestration (`OpenSpacePlanner`) managing candidate execution, fallback, and validation. |

## Documentation

Comprehensive design specifications, algorithmic formulas, and validation strategies are maintained under `wheelos-service/context/modules/planning/knowledge/`.

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

