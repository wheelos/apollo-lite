---
name: issue-localization
description: Search existing evidence and working code before tracing the first wrong state and testing root-cause hypotheses. Use for problem analysis, fault localization, unexpected behavior, test failures, and cross-module failures.
---

# Issue Localization: Search Before Tracing

## Scope and prerequisites

Use for errors, unexpected behavior, and failures with an unknown cause. The
goal is to narrow the fault and establish causal evidence, not to guess a fix.
Hand off performance-only issues to `performance-optimization`.
Read relevant source, tests, existing results, and recent related changes;
preserve the current worktree.
See the [skill index](../README.md) for domain knowledge and tool entry
points. Do not blindly scan the entire repository or preload every module.

## Workflow

### 1. Describe the symptom and inspect existing evidence first

Clarify expected and actual behavior, trigger conditions, affected versions,
first anomaly time or cycle, and inputs and outputs.
Read the relevant errors, stack traces, and existing logs in full, then
inspect current implementation, call relationships, configuration switches,
similar tests, and working paths. Do not ask again for information already
available in context.
Search only relevant history, known issues, and reference implementations; a
search hit is not root-cause evidence.

Prefer existing reproduction conditions. If the issue cannot be reproduced
reliably, preserve known events and identify missing evidence instead of
guessing a fix. Any new collection must distinguish specific hypotheses and
avoid sensitive data or unbounded logs.

### 2. Narrow the investigation to the first incorrect boundary

Trace inputs, state transitions, and call paths backward from the symptom,
comparing normal and failing examples layer by layer.
Find the earliest provable anomalous input, configuration, timing, state, or
output. Distinguish where an error is observed from where it is introduced;
do not patch only the final consumer that reports the error.
In multi-component systems, inspect only relevant boundaries and add minimal
instrumentation if needed; do not instrument everything.

For autonomous-driving issues, use evidence to check coordinates and units,
event time and receipt time, input freshness, reset, configuration, upstream
outputs, and downstream interpretation. For logs across clock domains,
confirm synchronization and offsets; do not infer causality only from log
print order.
For planning anomalies, distinguish input and decision boundaries,
optimization solving, trajectory stitching, and control response. Explore
only relevant areas; do not require checking every failure category.

### 3. Form and test necessary hypotheses

Separate confirmed facts, hypotheses to test, and eliminated causes.
For each candidate cause, state supporting evidence, falsification
conditions, and the smallest discriminating check. Prioritize low-cost
hypotheses that can narrow the scope most; the number depends on uncertainty,
not a mechanical requirement to list three.
Change one main variable per experiment when possible; predict the outcome
before running it.

Stop expanding the investigation when evidence is sufficient. Update
hypotheses when an experiment fails; do not stack patches or repeat
operations that add no information.
Correlation, temporal proximity, or a parameter change that “seems to help”
does not by itself prove a root cause.
If the cause cannot be confirmed, report the narrowed scope and next check,
and explicitly state that localization is incomplete.

### 4. Fix the root cause and prevent regression

For analysis-only tasks, stop at the root cause and evidence boundary; modify
implementation only when a fix is authorized.
State the chain “trigger condition → first incorrect input/state →
propagation path → visible symptom.”
Use a minimal reproduction or targeted test to demonstrate the original
problem, then implement the smallest fix and run regression coverage matching
the impact.
Do not hide unknown causes by widening tolerances, silently falling back,
extending timeouts, or adding retries.
If the contract requires fault tolerance, still explain the failure
mechanism, boundary, and how failures remain visible.

For scenario behavior, hand off to `simulation-regression`; do not
automatically launch full-stack replay for a local defect already covered by
evidence.
If repeated fixes are ineffective, stop and recheck assumptions and
boundaries. Ask the user before architectural redesign or scope expansion.

## Deliverables and acceptance

Report the symptom, reproduction status, first-failure boundary, key source
and runtime evidence, eliminated causes, causal chain, fix or next check, and
residual risks.
Distinguish “reproduced,” “scope narrowed,” “root cause confirmed,” and
“fixed and verified.” A code change or successful test-process exit does not
prove that the original symptom is resolved.

## Failure handling

If raw packages are missing, tools are unavailable, or timing is
incomparable, state how this affects the conclusion and choose safe
alternative evidence that tests the same hypothesis.
If evidence remains insufficient, report the gap and stop inferring.
Do not automatically restart shared services, clear data, update
dependencies, or replace the environment.

## Basis

Retain the first-failure-layer and causal-chain methods from context. Adapt
evidence gathering, comparison with working examples, and single-hypothesis
experiments from [Superpowers systematic-debugging](../README.md); do not
copy its fixed retry counts or requirement to read everything.
