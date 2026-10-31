---
name: stability-validation
description: Validate memory/resource ownership, repeated lifecycle, long-run and isolated recovery behavior.
---

# Stability and memory validation

## Trigger

Explicit resource, memory-growth, lifecycle, long-run, or stability validation.
This read-only workflow does not repair source; use development after attribution.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Declare realistic load/lifecycle, resource budgets, meaningful duration,
warm-up, observation windows, metrics/rules and approved isolated recovery.
Fault injection requires existing explicit authorization; never affect shared
users. Use existing memory investigation tools rather than installing profilers.

## Stages and agents

| Stage | Agent | Output |
| --- | --- | --- |
| preflight | algorithm-researcher | Workload, ownership hypotheses and approval |
| validation | algorithm-validator | Lifecycle/resource/teardown artifacts |
| independent-review | algorithm-reviewer | Allocation/growth/cleanup evidence audit |
| report | experiment-reporter | Classification, observed stability and limitations |

## Handoffs and recovery

Follow [shared lifecycle](../README.md) and memory-stability profile. Distinguish
RSS, live heap, allocator retention, fragmentation, caches/queues, CUDA memory,
FDs, threads, SHM and subscriptions. Separate sanitizer and normal-runtime runs.
All jobs must terminate; uncertain cleanup withholds further execution but still
allows review/reporting. Duration follows the actual lifecycle budget.

## Acceptance

PASS covers only declared load/duration/lifecycle and independently verified
checks. RSS growth alone is not a proven leak. Missing allocation/termination/
recovery evidence is INCONCLUSIVE; confirmed failed mandatory criteria are FAIL.
