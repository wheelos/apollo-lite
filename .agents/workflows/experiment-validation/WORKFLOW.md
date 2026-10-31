---
name: experiment-validation
description: Authorized reproducible experiments with independent verification.
---

# Experiment validation

## Trigger

Explicit experiment/benchmark/replay validation workflow request.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Pin dataset manifests, model/checkpoint, configurations, fixture hashes,
baseline conditions, thresholds, repetitions, and required trigger assertions.
GPU execution requires a provisioned shared lock path and resource identity.

## Stages

| Stage | Agent | Output |
| --- | --- | --- |
| preflight | algorithm-researcher | Frozen experiment and readiness; approval checkpoint |
| validation | algorithm-validator | Authorized experiment and hashed raw artifacts |
| independent-review | algorithm-reviewer | Independent conditions and result verification |
| report | experiment-reporter | Baseline comparison and uncertainty |

## Agents

Coordinator owns experiment approval. Validator uses `performance-optimization`
for measured comparisons or `simulation-regression` for replay/closed-loop proof,
only when applicable. No source writer is admitted.

## Handoffs

Follow [shared gates and result protocol](../README.md). Execute serially;
acquire the shared GPU lock before starting jobs, and establish termination
before releasing it. Queue contention through the external scheduler.

## Acceptance

Require executed test/metric checks, intact artifacts, comparable baselines,
and independent review. Missing scenario trigger, unavailable GPU/dataset, or
uncertain termination is INCONCLUSIVE, regardless of command exit status.
Failed declared criteria are FAIL. Do not rerun with relaxed thresholds.
