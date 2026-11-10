---
name: release-validation
description: Release candidate acceptance without publication or deployment.
---

# Release validation

## Trigger

Explicit release-candidate acceptance request, not an instruction to publish.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Pin candidate revision, delivery artifacts, compatibility consumers, relevant
regressions, and the release acceptance matrix.

## Stages

| Stage | Agent | Output |
| --- | --- | --- |
| preflight | algorithm-researcher | Candidate identity and evidence plan; approval checkpoint |
| validation | algorithm-validator | Executed acceptance checks and artifacts |
| independent-review | algorithm-reviewer | Independent release/consumer evidence audit |
| report | experiment-reporter | PASS / FAIL / INCONCLUSIVE and blockers |

## Agents

Coordinator owns acceptance scope and approval. Validator uses
`test-impact-analysis` and only relevant execution skills. Reviewer uses
`change-review` and interface-consumer contracts as needed. No source writer.

## Handoffs

Follow [shared gates and result protocol](../README.md). Stages are serial;
GPU checks require the shared lock. Separate checkouts retain their own rules.

## Acceptance

Require executed test/metric checks and independent evidence review. Missing
consumer proof is INCONCLUSIVE; failed criteria or critical/high defects are
FAIL. PASS authorizes no publication, deployment, hardware run, or commit.
