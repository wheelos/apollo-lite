# Skills

This directory contains task procedures. The sole durable authority for
repository facts, module contracts, and commands remains
`ad_agents/context/`. Select an entry point for the current task; do not load
every skill at once.

## Engineering workflows

| Task | Skill | Deliverable boundary |
| --- | --- | --- |
| Long-running, multi-stage feature or integration | [long-horizon-development](long-horizon-development/SKILL.md) | Goal-aligned working path, baseline, and staged completion without blocking on non-gating data or metrics |
| Technology selection, engineering frameworks, and architecture design | [engineering-solution-design](engineering-solution-design/SKILL.md) | Evidence-based trade-offs, principles, boundaries, and a minimal end-to-end path |
| Algorithm modeling, implementation, and module integration | [algorithm-development](algorithm-development/SKILL.md) | Contracts, baselines, implementation, correctness, and effectiveness evidence |
| Latency, CPU, GPU, memory, or throughput issues | [performance-optimization](performance-optimization/SKILL.md) | Baseline, profiling, bottleneck attribution, optimization, and controlled remeasurement |
| Failures, unexpected behavior, or unknown causes | [issue-localization](issue-localization/SKILL.md) | Search and gather evidence before locating the root cause; require causal evidence for fixes |
| Replay, closed-loop simulation, and scenario regression | [simulation-regression](simulation-regression/SKILL.md) | Explicit execution model, deterministic assertions, and evidence package |
| Diff or change review | [change-review](change-review/SKILL.md) | Evidence-backed behavioral findings without unsolicited fixes |
| Selecting validation for a change | [test-impact-analysis](test-impact-analysis/SKILL.md) | Required evidence matched to affected contracts and risk |
| Cross-consumer interface changes | [interface-migration](interface-migration/SKILL.md) | Compatibility inventory, migration order, and consumer proof |
| Model, preprocessing, or inference engine integration | [model-artifact-integration](model-artifact-integration/SKILL.md) | Versioned inference contract and target-runtime evidence |
| Evaluating agent rules, skills, or tools | [agent-evaluation](agent-evaluation/SKILL.md) | Reviewed case results, hard-gate failures, and comparable run evidence |

Start with the current problem and load another skill only when a handoff is
needed; these skills are not a mandatory pipeline for every task.
For a clearly scoped small change, use the existing contract directly; do not
require renewed research, architecture work, or full-stack simulation.
`long-horizon-development` coordinates multi-stage implementation; use the
specialist skills for design selection, algorithm behavior, root-cause
analysis, measured performance work, and scenario validation rather than
duplicating their procedures.
When issue localization reveals a resource bottleneck, hand off to performance
analysis. When algorithm behavior changes, select regression coverage based on
the impact.

## Existing engineering entry points

- [build](build/SKILL.md): compilation and build issues.
- [lint](lint/SKILL.md): formatting and static checks.
- [container-build](container-build/SKILL.md): development container images.
- [multi-repo-dev](multi-repo-dev/SKILL.md): multi-repository dependencies and publication.
- [vendor](vendor/SKILL.md): offline source delivery.
- [repository-agent-setup](repository-agent-setup/SKILL.md): agent layout and skill maintenance.

Read commands from the authoritative entry points below as needed, and verify
them against the current source or tool help. If a referenced document is
missing, search for the same filename under `ad_agents/context/`; if it is still not
found, report the gap instead of inventing a command or installing a tool:

- `ad_agents/context/framework/build/build-and-test-command-registry.md`
- `ad_agents/context/framework/run/tool-and-artifact-registry.md`
- `ad_agents/context/framework/validation/test-strategy-matrix.md`

## Adaptation boundaries for existing context workflows

The diagnostic, performance, and regression skills in this directory reuse
methods from the following documents without duplicating domain knowledge.
Load domain details only as needed. These adaptations are intentional; they do
not require mechanical execution of every step in the older workflows:

- `ad_agents/context/framework/skills/issue-localization.md`: retain first-failure-layer and causal-chain analysis; form only the hypotheses needed by the evidence, without requiring exactly three hypotheses or checking every planning-failure category.
- `ad_agents/context/framework/skills/performance-optimization.md`: retain measure-before-optimize. Its referenced `performance-testing.md` is currently missing. This directory's workflow includes its own comparative measurement steps and uses the existing test strategy matrix; it does not depend on the missing file.
- `ad_agents/context/framework/skills/simulation-debugging.md`: retain execution-model selection, configuration freezing, and first-failure-frame analysis; add open-loop versus closed-loop evidence boundaries, assertions, and handling of instability.

## External method references

Reviewed on 2026-10-26. Only methods confirmed by reading the sources are
adapted and summarized here. External skills are not installed, their
instructions are not treated as repository rules, and their scripts,
dependencies, or automatic commit behavior are not introduced.

| Source | Adapted method | Not copied directly |
| --- | --- | --- |
| [Superpowers systematic-debugging](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/systematic-debugging/SKILL.md) | Gather evidence first, find working examples, trace data backward, and test one hypothesis at a time | Fixed retry counts, adding logs before deciding whether they are needed, or reading all unrelated implementation |
| [Superpowers verification-before-completion](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/verification-before-completion/SKILL.md) | Keep claims consistent with actual verification evidence | Re-running checks that have passed and remain valid just to repeat them in every report |
| [Superpowers brainstorming](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/brainstorming/SKILL.md) | Read the current state first, clarify information that materially affects the solution, and compare viable paths | Mandatory Q&A for every task, creating and committing design documents, or requiring a spawned agent |
| [Superpowers writing-plans](https://github.com/obra/superpowers/blob/2ce8aa15d05bd57213e6f746f953aafb7ded7a8b/skills/writing-plans/SKILL.md) | Small independently testable tasks, explicit interfaces, and checkable outcomes | Long mandatory plans, test-first for every task, commits, or mandatory subagents |
| [Google Benchmark user guide](https://github.com/google/benchmark/blob/0a78b5d203283b54f65590ecef23e0b835b4787c/docs/user_guide.md) | Warm-up, repeated samples, distinguish CPU from wall-clock time, and interleave comparisons | Automatically adding a benchmark library or changing shared-host frequency or permissions |
| [SciPy test tips](https://github.com/scipy/scipy/blob/e4ae5a6fb55455b06bbf7ada6cbfa947e6da5529/doc/source/dev/contributor/writing_test_tips.rst) | Distinguish approximate comparisons for floating-point values from exact comparisons for integers | Adding SciPy assertion APIs or imposing a universal floating-point tolerance |

The design-selection skill also cites original MADR, arc42, and OpenSSF
materials. Autonomous-driving algorithm contracts, replay boundaries, and
safety constraints are scenario-specific additions; the cited general-purpose
open-source skills are not claimed to provide autonomous-driving validation.
