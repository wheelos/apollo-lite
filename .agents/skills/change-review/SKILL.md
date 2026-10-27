---
name: change-review
description: Review diffs for behavioral bugs, contract violations, and missing regression evidence. Use for code review, pre-submit review, and cross-consumer change audits.
---

# Change Review

## Procedure

1. Establish the exact base and diff, preserve dirty work, and read applicable
   rules. Do not review unrelated history or modify code unless requested.
2. Read changed logic, its callers, existing tests, and the owning contract.
   Check state, timing, coordinates, failure paths, concurrency, and lifecycle
   only where relevant to the change.
3. Trace each candidate finding to a concrete trigger and incorrect observable
   result. Verify it against source; distinguish introduced defects from
   pre-existing issues and unsupported assumptions.
4. Use `test-impact-analysis` when proof scope is unclear. Do not require
   broad runtime execution merely to review a local diff.
5. Report actionable findings with file/line, trigger, impact, and evidence.
   Rank by actual consequence; separate unverified questions from findings.

## Acceptance and failure handling

No cosmetic complaints presented as bugs, invented runtime evidence, automatic
fixes, or automatic commits. State the reviewed scope and material blind spots.
If source or contracts are unavailable, narrow the claim rather than inventing
an API. Explicit vulnerability reviews use the runtime's security-review path.
