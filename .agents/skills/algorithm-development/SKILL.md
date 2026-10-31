---
name: algorithm-development
description: Develop and integrate autonomous-driving algorithms with explicit mathematical contracts, simple baselines, numerical checks, and reproducible evaluation. Use for algorithm development, optimization, state estimation, and perception, prediction, planning, and control implementation and evaluation.
---

# Algorithm Module Development

## Scope and boundaries

Use for algorithm modeling, implementation, replacement, and engineering
integration; not every algorithm task is a model-training task.
When the approach has not been selected, use `engineering-solution-design`;
for an existing anomaly, use `issue-localization` first; for resource-only
optimization, use `performance-optimization`.
For a clearly scoped small fix, use existing contracts rather than repeating
algorithm selection.

## Prerequisites

Read the owning module's rules, source, tests, and call sites, and identify
existing algorithms, math libraries, and evaluation entry points.
Get domain contracts from `ad_agents/context/`; do not infer this
repository's APIs from papers or external implementations.
See the [skill index](../README.md) for build, lint, and validation entry
points; load only the execution skills needed.

## Workflow

### 1. Specify the problem and contract

Clarify goals, inputs and outputs, state, domain of applicability, non-goals,
and expected benefit relative to the existing baseline.
Specify as needed:

- Coordinate frames, transform direction, units, angle conventions, time
  basis, sample period, and data freshness.
- State initialization, update order, lifecycle, reset, and ownership of
  cross-frame information.
- Objective function, hard constraints and soft costs, probability or noise
  assumptions, observability, and degradation conditions.
- Error, effectiveness, compute, and memory budgets; derive requirements
  from the task or evidence, and do not invent thresholds.
- Existing degradation contracts and behavior for missing input,
  infeasibility, non-convergence, non-finite values, or timeouts.

Not every algorithm needs every item; specify only what affects correctness
for this task.
Ask the user first when a missing budget, degradation policy, or key
assumption could change behavior.

### 2. Establish an interpretable baseline

Prefer the existing implementation, a simple analytical solution, a small
enumeration, or an independently verifiable reference algorithm.
Confirm the assumptions behind the reference result; do not use only the
implementation under test to generate its own “correct answer.”
Reuse mature mechanisms first. For customized parts, establish invariants and
module boundaries before implementation details.
Define evaluation criteria before tuning to avoid cherry-picking favorable
cases or repeatedly moving acceptance thresholds.

### 3. Implement the minimal working path

Follow existing module boundaries. Separate the numerical core from message,
configuration, and scheduling adapters, but do not create an unnecessary
framework or duplicate an existing adapter for a single change.
Make inputs, outputs, and failure states explicit; do not silently fall back
to zero values, previous-frame results, or empty trajectories.
If the existing contract permits reusing a previous frame, preserve its
validity period, reason, and status marker.

First connect a real input to an inspectable output. Unimplemented
capabilities in a framework stage must be identifiable. Mocks and idealized
inputs demonstrate only local mechanisms, not algorithm effectiveness in
real scenarios.
For real-time paths, check bounded computation, memory allocation, and
staleness of asynchronous results as needed; measure before performance
refactoring.

### 4. Validate correctness and numerical behavior

Add a small number of tests that directly cover new or fixed behavior,
preferably in existing test files.
Select tests according to algorithmic risk rather than mechanically
including every category:

- Known answers, small reference comparisons, and invariants such as
  conservation, bounds, and symmetry.
- Empty inputs, boundaries, degenerate cases, unusual scales, and failure
  states; check that outputs satisfy hard constraints.
- Distinguish absolute and relative error in floating-point comparisons and
  explain the basis for tolerances; do not require bit-for-bit cross-platform
  agreement.
- For derivatives, use finite differences with an appropriate step size when
  applicable. For solvers, check residuals, feasibility, and termination
  reasons rather than only “success” status.
- For stateful behavior, test initialization, sequential updates, reset, and
  discontinuous inputs, not just a single frame.

Use metamorphic or property-based tests only when the properties genuinely
hold; do not encode incorrect assumptions as tests.
For a bug fix, prove that the test detects the original defect. Do not
overwrite or revert another person's work to manufacture a failure.

### 5. Evaluate effectiveness and integrate

Fix code, dependency, configuration, data versions, and randomness; retain
representative scenarios and known failure cases.
Report algorithmic correctness, task effectiveness, resource budgets, and
remaining limitations separately; do not use one average score as a
substitute for all of them.
For learning-based algorithms, isolate training, tuning, and test data as
needed and check for scenario or temporal data leakage. Record model and
preprocessing versions and verify training/inference contracts.

Offline metrics do not replace closed-loop behavioral evidence. When outputs
affect downstream behavior, use `simulation-regression` to select the
smallest sufficient validation; do not assume whole-vehicle or road tests
are required.
For performance claims, use the controlled measurement workflow in
`performance-optimization`.

## Deliverables and acceptance

Report contracts and assumptions, baseline, implementation scope,
correctness and effectiveness evidence, failure boundaries, and unresolved
risks.
Base evidence on actual results; distinguish work that was not run,
simulation-only results, idealized-input results, and results still requiring
hardware validation.
If the expected benefit was not achieved, report that honestly. Do not claim
completion by changing metrics, relaxing constraints, or hiding failures.

## Failure handling

If unexplained numerical deviations, infeasible solutions, or regressions
occur, stop the related change and preserve the minimal input and state.
First check semantics, timing, coordinates, and configuration, then assess
the mathematical model and implementation. Choose the order based on
evidence; do not attribute every failure to parameters.
Obtain necessary confirmation before changing the problem definition,
constraints, or degradation behavior.
Do not automatically install a new solver, replace a math library, or expand
the experiment to hide a problem.

## Basis

Use the type-aware assertion approach from SciPy and the evidence acceptance
methods referenced by the [skill index](../README.md).
The algorithm modeling and autonomous-driving contract requirements here
are scenario-specific additions; they do not introduce an external assertion
API.
