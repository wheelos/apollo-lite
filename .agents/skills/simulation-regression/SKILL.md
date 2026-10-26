---
name: simulation-regression
description: Reproduce autonomous-driving behavior with the right replay or closed-loop harness, explicit assertions, frozen fixtures, and regression evidence. Use for replay reproduction, closed-loop simulation, scenario regression, first-failure-frame analysis, and converting issues into regressions.
---

# Simulation, Replay, and Scenario Regression

## Scope and prerequisites

Use this skill to reproduce existing issues and validate scenarios affected by
algorithm or engineering changes. Do not launch the full stack for every local
change.
First define the behavior to prove and its scope, then read the owning
module's contract and existing scenarios.
For commands, runtime environments, and evidence scope, see the authoritative
registries in the [skill index](../README.md). Before execution, verify the
actual harness and its help; do not invent scenario IDs, parameters,
assertions, or artifact paths.
For an unknown root cause, use `issue-localization` first; do not use extensive
simulation as a substitute for diagnosis.

## Workflow

### 1. Select an execution model for the problem

| --- | --- | --- |
| Model | Can demonstrate | Cannot demonstrate alone |
| --- | --- | --- |
| Local unit or component test | Mathematical properties, state transitions, and local interfaces | Whole-vehicle interactions and real sensor behavior |
| Open-loop log replay | First-failure reconstruction and module-output comparison for recorded inputs | Closed-loop outcomes after new decisions change the ego vehicle or environment |
| Closed-loop simulation | Behavior under decision, control, and environment feedback | Real-world safety outside the simulation model's coverage |
| HIL or road vehicle | Specific behavior coupled to hardware and the actual environment | General correctness across all scenarios |

Choose the smallest model that supports the current conclusion. Confirm which
modules the harness actually runs, which outputs come from the recording, and
which are recomputed; do not let old outputs mask the change under test.
Road tests and hardware operations require explicit authorization and
established safety procedures; ordinary regression must not escalate to them
automatically.

### 2. Freeze reproducibility conditions

Record code and build versions, configuration, map, route, vehicle
parameters, input package, model version, module set, random seed, and
clock/replay rate, trimming the list to the scenario.
Specify initial state, cleanup between runs, and recording-package integrity.
Use the harness's isolation mechanism; do not manually clear shared
directories or interrupt another person's process.
Reuse existing scenarios first. If a new scenario is necessary, retain only
the essential trigger conditions and use legitimate data sources.

### 3. Define decidable assertions before execution

Express expected behavior using input preconditions, observations, time
windows, thresholds, and evaluation methods.
Include proof that trigger conditions occurred, outputs are valid, and
behavioral constraints related to the change are met. Check collision,
feasibility, tracking error, completion conditions, or deadlines as relevant;
do not invent criteria.
Verify fixtures and runtime prerequisites first so a scenario that never
triggered cannot be reported as “passing.”

Distinguish environment or startup failures, invalid scenarios, failed
behavioral assertions, skips, and passes.
Missing required topics, probes, or metrics cannot count as a pass; a
successful process exit does not mean assertions passed.
Visualization can aid analysis but does not replace computable assertions.

### 4. Execute and analyze the first failure

Use original evidence to reconstruct the first failing frame, decision, or
state, and compare relevant inputs and outputs across versions.
Closed-loop trajectories may diverge quickly. Prioritize checking divergence
preconditions and the first causal difference; do not force alignment of
subsequent frames that have already diverged.
For performance claims, hand off to `performance-optimization`.

Preserve failure evidence first; do not rerun without bounds until a pass
appears. If instability is suspected, repeat a bounded number of times under
fixed conditions and record every result, seed, and failure rate. Choose the
number of repetitions based on the problem and cost.
Use numerical tolerances permitted by the contract. Do not require
bit-for-bit consistency from nondeterministic systems or hide instability by
loosening assertions.

### 5. Turn a stable reproduction into a regression

Once trigger conditions and correct outcomes are clear, reduce the real issue
to a minimal scenario executable by an existing harness. Preserve necessary
timing and interactions; do not copy the entire incident package.
A regression for a fix should detect the original defect and pass after the
fix; avoid snapshots of current output alone.
After minimization, verify that the scenario still triggers the same
mechanism; similar symptoms do not prove the mechanisms are the same.
Maintain scenario assertions, fixture versions, and sources. Archive a
separate knowledge document only when requested by the user.

## Planning sim-agent boundary

If using the existing Planning sim-agent, first read
`ad_agents/sim_agent/skills/planning-simulation/SKILL.md`，
and preserve its Plan validation, exact-plan approval, Worker, and
deterministic-evaluation boundaries.
This skill does not approve plans, replace that executor, or automatically
start an agent workflow.
Follow the authorization requirements of other existing harnesses; do not
invent a generic approval tool.

## Deliverables and acceptance

Report the behavioral question, execution model and limitations, frozen
conditions, scenario and assertions, actual execution status, first-failure
or comparison findings, artifact locations, and uncovered risks.
Link results to the relevant code and fixtures. Preserve raw logs, structured
results, and available JUnit/probe artifacts; use the actual harness outputs
as evidence and do not fabricate missing files.
A passing software simulation does not establish road or hardware validation.

## Failure handling and basis

When the environment or fixture is invalid, stop behavioral evaluation and
diagnose the environment first. Ask before changing configuration,
dependencies, data, or authorization boundaries. Report unavailable hardware
or data as evidence gaps.
Reuse the execution models and first-failure-frame methods from context, as
well as deterministic assertions from the existing Planning skill. See the
[skill index](../README.md) for evidence acceptance principles; do not copy
external execution or deployment procedures.
