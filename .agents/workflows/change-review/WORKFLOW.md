---
name: change-review
description: Independent read-only change and evidence review.
---

# Change review

## Trigger

Explicit collaborative diff review. A small review stays single-agent.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Pin the current revision and supplied diff in file scope. Required checks may be
source-only; run builds/tests only when explicitly requested.

## Stages

| Stage | Agents | Output |
| --- | --- | --- |
| inspection | algorithm-reviewer + algorithm-validator, parallel | Independent source findings and impact-selected evidence |
| independent-review | algorithm-reviewer | Verified findings and evidence boundaries |
| report | experiment-reporter | Actionable findings and acceptance |

## Agents

Coordinator selects bounded independent scopes. Reviewer uses `change-review`,
validator uses `test-impact-analysis`. Neither fixes source. Validation commands
may produce session artifacts, not source/configuration edits.

## Handoffs

Follow [shared gates and result protocol](../README.md). The second review waits
for both inspection results. Preserve original evidence and source lines.

## Acceptance

Source-review PASS does not mean runtime correctness. Missing required evidence
is INCONCLUSIVE; failed checks or unresolved critical/high findings are FAIL.
No unsolicited fixes, build requirement for pure review, or automatic commits.
