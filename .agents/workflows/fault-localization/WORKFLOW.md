---
name: fault-localization
description: Trace one causal chain, reproduce the first wrong state, and fix it with regression proof.
---

# Fault localization

## Trigger

Explicit multi-stage fault diagnosis and repair.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Supply actual symptoms/fixtures, baseline and source scope. Candidate acceptance
checks default to validation; use check.stage to explicitly select other evidence.
Expected reproducer failures are baseline observations, not candidate failures.

## Stages and agents

| Stage | Agent | Output |
| --- | --- | --- |
| research | algorithm-researcher | One causal chain and discriminating reproducer |
| reproduction | algorithm-validator | First wrong state and original baseline evidence |
| design-review | algorithm-reviewer | Root cause and approved minimal fix |
| implementation | algorithm-implementer | Fix and focused regression |
| validation | algorithm-validator | Candidate regression proof |
| independent-review | algorithm-reviewer | Causal/evidence verification |
| report | experiment-reporter | Cause, correction, proof and unresolved questions |

## Handoffs and recovery

Follow [shared lifecycle](../README.md). Do not spread one trace across parallel
agents or guess a fix without causal evidence. Missing reproducibility permits
read-only analysis/reporting, not speculative source modification.

## Acceptance

Candidate mandatory regressions and independent verification must pass.
Expected original failures remain preserved. A failed candidate check is FAIL;
an unavailable reproducer or causal proof is INCONCLUSIVE.
