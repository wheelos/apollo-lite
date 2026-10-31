---
name: algorithm-development
description: Approved algorithm changes with one writer and independent proof.
---

# Algorithm development

## Trigger

Explicit workflow request for substantial algorithm implementation.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Declare source scope, mathematical/interface contracts, target criteria,
baseline revision, fixtures, and mandatory test/metric checks before execution.

## Stages

| Stage | Agent | Output |
| --- | --- | --- |
| research | algorithm-researcher | Minimal design and contracts |
| design-review | algorithm-reviewer | Independent design review; approval checkpoint |
| implementation | algorithm-implementer | Approved diff, unique source writer |
| validation | algorithm-validator | Executed tests/metrics and original artifacts |
| independent-review | algorithm-reviewer | Actual diff and evidence audit |
| report | experiment-reporter | Change, evidence, limitations |

## Agents

Stages are serial. The coordinator owns approval, scope, and acceptance.
Implementer loads `algorithm-development`; validator loads `test-impact-analysis`
and only the relevant build, lint, numerical, model, or simulation skills.
Reviewer never modifies the diff.

## Handoffs

Follow [shared gates and result protocol](../README.md). Design approval binds
the run, reviewed design, and input fingerprint. Preserve the original dirty
worktree; modifications outside approved scope stop execution. GPU validation
requires the shared resource lock.

## Acceptance

All declared checks, at least one executed test/metric, and independent review
must pass. Missing proof is INCONCLUSIVE. A failed check or unresolved critical/
high finding is FAIL. Follow shared reviewer-directed bounded correction with
new candidate proof; retain original failed attempts and still generate reports.
No concurrent writers or blind repair/test loops.
