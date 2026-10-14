# Apollo Lite System-Wide Copilot Instructions

This repository follows a strict decoupled architecture optimized for modern large language models.
`ad_agents/context/` is the **ONLY** durable knowledge layer and **single source of truth** for repository knowledge.

## 0. ABSOLUTE LAWS (Never Violate)
1. **Reality > Docs:** Hierarchy of truth: `Compiler > Tests > Runtime > Source Code > Context Docs > README > Comments`. Never modify working code to fit stale documentation.
2. **Zero Hallucination:** NEVER invent APIs, commands, file paths, variables, or repo structures. If unknown, retrieve context or ask the user.
3. **Minimal Change:** Implement the smallest correct solution. NO unrelated refactoring, cosmetic cleanups, or symbol renaming.
4. **Fail Fast:** If any command, build, or validation fails -> STOP immediately. Read `ad_agents/context/framework/anti-patterns/README.md`. Explain the root cause and wait for user confirmation. NEVER blindly retry.
5. **Scope Containment:** Only execute the requested task. If out-of-scope work is required, STOP and ask for permission.
6. **Meta Rule (Conflict Resolution):** When multiple rules conflict, always choose the action that sequentially:
   1) Preserves correctness.
   2) Minimizes scope.
   3) Minimizes assumptions.
   4) Minimizes cost.

## 1. CONTEXT RETRIEVAL PROTOCOL
Keep the active context minimal. Do NOT scan the repo blindly. Stop immediately once sufficient info is found.
**Note:** The retrieval order defines priority, NOT mandatory sequential steps. Skip unnecessary levels when the required context is already available.
**Path resolution rule:** treat `ad_agents/context/` as the context root. Any shorthand such as `framework/...`, `modules/...`, or `skills/...` must be resolved under this root first (for example, `framework/build/build-and-test-command-registry.md` => `ad_agents/context/framework/build/build-and-test-command-registry.md`).
1. `ad_agents/context/context-catalog.json`
2. `ad_agents/context/README.md`
3. `ad_agents/context/framework/retrieval/ai-coding-session-entrypoints.md`
4. Target module `README.md`
5. Specific documents required for the task.
*Fallback:* If module ownership is unknown, consult `ad_agents/context/framework/retrieval/task-module-routing-matrix.md`. If a direct read fails, run a scoped lookup under `ad_agents/context/**/<filename>` and then read the matched canonical path.

## 2. EXECUTION WORKFLOW
Follow this exact sequence for every task:
- **[1] Classify & Route:** Determine task type (e.g., Debug, Refactor, Gen) and locate the owning module.
- **[2] Retrieve:** Gather minimum required context following the Protocol.
- **[3] Plan:** Formulate the smallest correct fix within architecture boundaries.
- **[4] Implement:** Write code following the "Anti-Blob" rule (single responsibility, cohesive modules). Do NOT bypass module ownership.
- **[5] Validate:** Prove correctness using the smallest sufficient method.
- **[6] Report:** Output results concisely.

## 3. VALIDATION & REGISTRIES
Perform validation **ONLY** when the task modifies executable artifacts (skip for pure design/review tasks). NEVER invent commands. Always use canonical registries:
- **Build/Test Commands:** `ad_agents/context/framework/build/build-and-test-command-registry.md`
- **Runtime Tools:** `ad_agents/context/framework/run/tool-and-artifact-registry.md`
- **Validation Strategy:** `ad_agents/context/framework/validation/test-strategy-matrix.md`

**Validation Escalation Path (Stop at the smallest successful step):**
`Formatter` -> `Static Analysis` -> `Unit Test` -> `Integration Test` -> `Replay Test` -> `Road Test`
*Runtime Note:* Prefer `whl start test`. Promote stable replay scenarios to integration tests.

## 4. COST AWARENESS
Always prefer the lowest-cost action that safely completes the task. You must actively minimize:
- **Token usage:** Keep context loading and outputs concise.
- **File modifications:** Avoid touching files outside the immediate scope.
- **Context retrieval:** Do not over-fetch documentation.
- **Validation cost:** Use the lightest test capable of proving correctness.

## 5. KNOWLEDGE MAINTENANCE
Execute this workflow **ONLY** when explicitly requested by the user. When durable engineering knowledge is created or updated:
- Ensure one topic per document using `kebab-case.md`.
- Update the nearest `README.md` and `context-catalog.json`.
- Strict execution of update loops:
  - `ad_agents/context/framework/skills/knowledge-skill-iteration.md`
  - `ad_agents/context/framework/validation/session-context-update-loop.md`
