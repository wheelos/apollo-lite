---
name: long-horizon-development
description: Deliver long-running, multi-stage features and integrations by staying anchored to the user goal, completing a minimal working path, establishing a baseline, then filling gaps and optimizing. Use when implementation spans multiple milestones or can stall on unavailable data or metrics.
---

# Long-Horizon Development

## Scope

Use for substantial implementation that spans multiple milestones, modules,
or dependencies and needs continued delivery without losing the user's goal.
The task must include implementation, not only research or design.

Do not use this skill for a small, bounded change, a root-cause investigation,
or a narrowly scoped algorithm, performance, build, or regression task. Route
those to the corresponding skill. This skill coordinates delivery; it does
not replace domain-specific contracts or validation.

## Prerequisites

- Read the repository's agent rules and the relevant module contracts, source,
  tests, and worktree status.
- Confirm the user-visible outcome, existing baseline, constraints, and
  boundaries. Keep established work and module ownership intact.
- Identify what is already known and what data, metrics, or decisions are
  unavailable. Ask only when the missing information changes safety,
  correctness, a hard contract, or the implementation boundary.

## Workflow

### 1. Keep the goal and blockers explicit

State the intended outcome in one sentence and identify the smallest
observable result that would demonstrate progress toward it.
Separate:

- **Hard gates:** Safety, correctness, required interfaces, legal constraints,
  and user-confirmed release or acceptance criteria.
- **Open evidence:** Data, measurements, or metrics that would improve
  confidence but are not required to build and verify the initial path.

Do not turn open evidence into a prerequisite by default. Do not invent a
target metric or claim evidence that has not been collected. If missing
evidence is a hard gate for a production claim or behavior, keep that boundary
explicit while continuing safe, independent work.

### 2. Deliver the minimal working path first

Choose the smallest vertical slice that exercises the real ownership and
interfaces from input to an observable result. Implement the framework and
minimum behavior needed for that path before broadening scope.

- Prefer existing module boundaries, interfaces, and test mechanisms.
- Keep unavailable data behind an explicit input boundary. Use a fixture or
  synthetic input only to verify the contract or plumbing, and label what it
  does not validate.
- Represent unsupported, unavailable, or diagnostic-only behavior explicitly.
  Do not use empty stubs, fabricated success, silent fallbacks, or mock output
  as evidence of production behavior.
- Validate the slice with the smallest repository-supported check that proves
  its stated behavior.

### 3. Establish and preserve a baseline

After the minimal path works, record its observable behavior and verification
evidence: relevant tests, outputs, configuration, input source, and known
limitations. Use real representative data when available; otherwise record
the fixture or synthetic input and its limited scope. Missing operational data
or product metrics must not stop work that can be implemented and checked
without them.

For a performance change, use `performance-optimization` to capture the
relevant pre-optimization measurements and compare like-for-like results.
Do not treat a functional baseline, fixture result, or proxy as a performance
or real-world effectiveness baseline.

### 4. Complete in goal-aligned increments

Turn the remaining work into small increments with observable outcomes and
clear acceptance checks. Prioritize by user value, dependency, and risk; keep
each increment independently reviewable and verifiable.

After each increment, check whether it still advances the stated outcome.
Defer polish, broad data collection, speculative abstractions, and
non-blocking metrics when they do not change correctness or the next decision.
Revisit deferred items when evidence becomes available or a hard gate depends
on them. Do not require a plan document, fixed milestone count, or agent
delegation unless the task or repository requires it.

### 5. Complete and optimize from evidence

Fill required behavior and integration gaps before expanding into optional
capabilities. Add representative data, metric coverage, and scenario
validation as they become available or necessary for a stated acceptance
claim.

Optimize only against a real requirement and an appropriate baseline. For
algorithm design or implementation, hand off the relevant work to
`algorithm-development`; for measured resource bottlenecks, use
`performance-optimization`; for behavior requiring replay or closed-loop
evidence, use `simulation-regression`. Keep the delivery goal, interfaces, and
deferred risks coherent across these handoffs.

## Acceptance

For work that must survive interruption, use the
[task checkpoint contract](../../../ad_agents/context/framework/validation/agent-task-checkpoints.md)
and [checkpoint tools](../../../scripts/agents/README.md). Store task state in
the session workspace, not in durable context. Capture every relevant checkout,
untracked input, configuration, and fixture in the explicit fingerprint scope.
On resume, compare fingerprints, recheck current ownership and outstanding
approvals, then reuse only evidence whose inputs are unchanged.
Do not treat checkpoint status as authorization or a test result.

- The user-visible goal and hard gates remain explicit throughout delivery.
- A working, inspectable minimum path exists before optional breadth or polish.
- Baselines and evidence identify their inputs and limitations; no proxy is
  presented as real-data, production, or performance validation.
- Data or metrics that are not hard gates do not block independent,
  verifiable implementation.
- Follow-on increments are scoped to the goal, and deferred items have a
  reason and a condition for revisiting them.
- Completion and optimization claims match the evidence actually obtained.

## Failure handling

Stop only the affected path when a hard gate fails, required interfaces
conflict, or missing evidence makes a correctness or safety claim impossible.
Preserve completed work and continue independent, safe increments where
possible.

If blocked, report the specific decision or evidence needed, what can proceed,
and the consequence of deferral. Do not silently weaken requirements, invent
data or metrics, broaden scope, or present a scaffold as a finished feature.

## Method reference

Adapt the small, independently testable task boundaries, explicit interfaces,
and checkable outcomes from
[Superpowers writing-plans](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/writing-plans/SKILL.md)
(reviewed 2026-10-26). This skill deliberately does not require a long
implementation plan, test-first workflow, commits, or subagents for every
task.
