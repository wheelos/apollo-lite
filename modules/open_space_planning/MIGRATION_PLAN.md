# Open-space planning migration plan

## Objective

Create a parallel, grid-map/open-space planning module that owns its own route
lattice, trajectory lattice, validation, and runtime orchestration. The current
public-road planner remains intact while the new module grows and eventually
takes over open-space execution.

## Ownership boundary

- `modules/planning`: public-road / HDMap / reference-line planning baseline.
- `modules/open_space_planning`: grid-map / open-space planning stack.
- `modules/common/planning`: future neutral shared utilities once the migration
  is stable.

The open-space package must not depend on HDMap routing, `Frame`,
`ReferenceLineInfo`, or the old public-road lattice implementation.

## Phase 1: isolate the new planner domain

- Define the public contract (`PlanningProblem`, `RouteCandidate`,
  `PhysicalTrajectory`, `ValidationReport`, `PlanningResult`).
- Keep the current road-planning stack unchanged.
- Add the new module skeleton and package-level API boundaries.
- Validate that the new module owns only open-space inputs and outputs.

Acceptance criteria:

- No new open-space algorithm depends on public-road runtime objects.
- The package names reflect open-space ownership and not road-planning naming.

## Phase 2: move reusable lattice kernels

- Copy or extract map-independent lattice math and trajectory kernels into
  `modules/open_space_planning/lattice/`.
- Keep the code generic: no map, route, task, frame, or reference-line types.
- Rewire includes and BUILD labels to the new ownership path.

Acceptance criteria:

- `modules/open_space_planning/lattice/...` is the canonical source for new
  lattice kernels.
- `modules/planning/lattice/...` is referenced only for temporary compatibility
  and not by newly added code.

## Phase 3: separate route topology from physical trajectory

- `route/` owns topology search, corridor generation, and kinematic feasibility.
- `trajectory/` owns time-parameterized physical trajectories and local
  execution generation.
- `safety/` owns the final hard validation layer.
- `runtime/` owns candidate selection, fallback, and revision checks.

Acceptance criteria:

- Route generation never directly performs the final trajectory safety check.
- The trajectory layer never changes topology beyond the chosen corridor.

## Phase 4: candidate lifecycle and hysteresis

- Use a small top-K set: 2–3 route candidates, topology-distinct.
- Keep one active candidate and demote inactive ones to low-frequency checks.
- Apply hysteresis before candidate switching.
- Fallback to braking/hold only after the validator rejects all live candidates.

Acceptance criteria:

- No left/right oscillation under persistent obstacles.
- Candidate promotion is deterministic and testable.

## Phase 5: integration and simplification

- Replace temporary compatibility references with neutral common utilities when a
  kernel is truly shared.
- Remove direct roadmap references from the open-space package.
- Keep the package boundary strict until a real deployment target is ready.

Acceptance criteria:

- All new code under `modules/open_space_planning` is self-contained or depends
  only on neutral utilities.
- The public-road planner continues to run without the new module affecting its
  behavior.

## Recommended implementation order

1. module skeleton and public API
2. lattice kernel migration
3. route-candidate generation SPI
4. trajectory planner SPI
5. safety validator SPI
6. runtime orchestration plus fallback
7. final dependency cleanup and neutral utility extraction

This sequence keeps the work modular and reduces the chance of hidden coupling
between road planning and open-space planning.
