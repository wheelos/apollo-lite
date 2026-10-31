---
name: solution-prototyping
description: Compare alternatives and validate the critical risk with a scoped runnable prototype.
---

# Solution prototyping

## Trigger

Explicit solution selection and prototype implementation, not read-only research.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Describe options including the existing baseline, hard constraints, critical
risk, source scope, a minimal vertical path, and declared feasibility checks.

## Stages and agents

| Stage | Agent | Output |
| --- | --- | --- |
| research | algorithm-researcher | Options, trade-offs, critical risk and prototype |
| design-review | algorithm-reviewer | Scoped design/approval |
| implementation | algorithm-implementer | Runnable minimum prototype |
| validation | algorithm-validator | Original feasibility/integration evidence |
| selection-review | algorithm-reviewer | Independently verified decision |
| report | experiment-reporter | Selected/rejected options and production gaps |

## Handoffs and recovery

Follow [shared lifecycle](../README.md). One writer; optional invocation-scoped
approval covers normal non-GPU implementation only. The reviewer may request a
bounded correction, then implementation, validation and review rerun.

## Acceptance

PASS covers declared prototype checks, not production readiness. Missing proof
is INCONCLUSIVE and failed mandatory checks are FAIL; reporting still runs.
