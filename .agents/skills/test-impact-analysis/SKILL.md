---
name: test-impact-analysis
description: Select required validation from a diff, affected contracts, and risk. Use for test planning, regression selection, and determining sufficient evidence before implementation.
---

# Test Impact Analysis

## Procedure

1. Identify changed behavior, direct consumers, state ownership, configuration,
   and failure/degradation contracts from source and existing tests.
2. Consult the
   [test strategy](../../../ad_agents/context/framework/validation/test-strategy-matrix.md)
   and verified command registry. Reuse existing tests before adding tooling.
3. Map each affected invariant to the smallest check that can actually refute
   an incorrect implementation. Record required evidence before editing.
   Formatting is not behavior evidence; open-loop output comparison is not
   closed-loop behavioral proof.
4. For fixes, select a case detecting the original defect and nearby relevant
   boundaries. For migrations, cover direct producers and consumers. Escalate
   only when local evidence cannot cover the affected contract.
5. Classify each required check as passed, failed, blocked, skipped, or not run.
   Record uncovered risks; never convert a missing fixture or non-triggering
   scenario into success.

## Acceptance and failure handling

Deliver a compact change-to-evidence mapping with verified targets, environment,
and any approval requirements. Stop when every required claim has sufficient
evidence, not when the cheapest check passes. Do not install tools, modify
assertions, or run road/HIL tests to resolve a planning gap without approval.
