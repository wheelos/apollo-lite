---
name: engineering-development
description: Minimal scoped engineering implementation and integration with independent evidence.
---

# Engineering development

## Trigger

Explicit collaborative software engineering implementation or module integration.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Declare current behavior, interfaces/configuration, dependent consumers, source
scope and local/integration checks. External checkout identities are optional
but missing identity limits reproducibility; writers stay in this repository.

## Stages and agents

| Stage | Agent | Output |
| --- | --- | --- |
| research | algorithm-researcher | Existing behavior and minimal integration path |
| design-review | algorithm-reviewer | Compatibility/lifecycle design and approval |
| implementation | algorithm-implementer | Unique scoped source writer |
| validation | algorithm-validator | Local and integration artifacts |
| independent-review | algorithm-reviewer | Check-level independent verification |
| report | experiment-reporter | Working path, evidence and limitations |

## Handoffs and recovery

Follow [shared lifecycle](../README.md), using the engineering or middleware
profile as relevant. Reviewer-directed correction reruns candidate checks;
missing optional context does not prevent implementation from source evidence.

## Acceptance

Require declared mandatory checks and independent evidence verification.
Unavailable proof is INCONCLUSIVE; failures remain FAIL. No automatic publication,
dependency checkout changes, or expansion into unrelated modules.
