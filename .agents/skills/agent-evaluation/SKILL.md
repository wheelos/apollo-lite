---
name: agent-evaluation
description: Evaluate changes to agent instructions, skills, routing, and tools using frozen cases and reviewed evidence. Use for agent regression, prompt comparisons, recovery checks, and capability acceptance.
---

# Agent Evaluation

## Procedure

1. Read the
   [evaluation contract](../../../ad_agents/context/framework/validation/agent-evaluation.md)
   and select relevant cases from the
   [case set](../../../scripts/agents/evaluation-cases.json).
2. Freeze task inputs, code, context, skill/tool versions, model configuration,
   environment, and approvals for each comparison. Use separate worktrees or
   the existing isolated harness; never revert someone else's work.
3. Run only explicitly authorized tasks. Preserve actual transcripts, tool
   results, diffs, and evidence outside durable knowledge. No automatic
   road/HIL operations, publication, paid runs, or agent spawning.
4. Have a reviewer judge each case gate from the evidence. Record false or
   not-run for missing proof; an agent's own completion claim is insufficient.
5. Use the evaluation command to check report structure and aggregate gate
   results. Compare like-for-like runs; do not present schema validation as
   an agent benchmark or averages as permission to ignore hard-gate failures.

## Acceptance and failure handling

Report case coverage, hard-gate failures, correctness, unnecessary changes,
invalid operations, recovery, cost, and human intervention when measured.
Keep tuning and holdout cases distinct. Do not weaken gates after seeing a
failure; version intentional criterion changes and rerun comparable baselines.
