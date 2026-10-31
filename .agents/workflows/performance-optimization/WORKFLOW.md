---
name: performance-optimization
description: Measure baseline and profile before a minimal optimization and comparable candidate checks.
---

# Performance optimization

## Trigger

Explicit bottleneck analysis and performance implementation.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Declare workload/conditions, completion timing boundary, cold/warm policy,
metric rules and units, correctness/resource budgets, baseline and source scope.
Record variability and samples; do not invent a universal repetition count.

## Stages and agents

| Stage | Agent | Output |
| --- | --- | --- |
| research | algorithm-researcher | Frozen workload/budgets |
| baseline-profile | algorithm-validator | Baseline and targeted profile samples |
| design-review | algorithm-reviewer | Causal hypothesis and one approved change |
| implementation | algorithm-implementer | Minimal bottleneck change |
| validation | algorithm-validator | Comparable candidate and correctness/resources |
| independent-review | algorithm-reviewer | Original samples/units/noise verification |
| report | experiment-reporter | Observed gain or uncertainty and limitations |

## Handoffs and recovery

Follow [shared lifecycle](../README.md) and performance profile. Baseline evidence
is archived before source edits. Candidate checks must reference exact source
fingerprint; baseline checks are selected explicitly with check.stage.
GPU profiling/experiments require the existing approval and shared resource lock.

## Acceptance

Metric PASS requires deterministic rules and independent verification. No
reliable gain is a legitimate reported result. Missing measurements/noise proof
is INCONCLUSIVE; violated mandatory quality/resource/performance criteria are FAIL.
