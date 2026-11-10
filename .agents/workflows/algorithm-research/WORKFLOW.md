---
name: algorithm-research
description: Source-backed algorithm research and independent route review.
---

# Algorithm research

## Trigger

Explicit multi-agent technology research or route-selection request. No source
modification or experiment execution.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Declare source decision checks, current revision, alternatives, constraints,
and explicit source/model input files.

## Stages

| Stage | Agents | Output |
| --- | --- | --- |
| research | algorithm-researcher + algorithm-validator, parallel | Independent alternatives and feasibility |
| decision-review | algorithm-reviewer | Source-backed decision critique |
| report | experiment-reporter | Comparison, recommendation, uncertainty |

## Agents

The coordinator reconciles disagreements. Researcher uses
`engineering-solution-design`; validator inspects existing baselines and
compatibility, not new experiments. Reviewer independently audits claims.

## Handoffs

Follow [shared gates and result protocol](../README.md). Review waits for both
research results. Include original source references and hashed evidence.

## Acceptance

PASS is a source-backed decision only. Missing decision evidence is INCONCLUSIVE;
confirmed invalid assumptions are FAIL. Never claim accuracy or speed without
executed comparative measurements. Failure returns to the coordinator, not an
automatic implementation or retry.
