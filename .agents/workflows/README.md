# Collaborative workflows

Workflows organize task lifecycles, not additional capabilities. Rules remain in
[AGENTS.md](../../AGENTS.md), procedures in [Skills](../skills/README.md), and
durable knowledge in [context](../../ad_agents/context/README.md). Do not create a
second knowledge tree. Temporary results belong in the session workspace.

| Task | Specification | Execution |
| --- | --- | --- |
| Research and route selection | [algorithm-research](algorithm-research/WORKFLOW.md) | Read-only research and independent evidence review |
| Algorithm implementation | [algorithm-development](algorithm-development/WORKFLOW.md) | Approved design, one writer, validation, independent review |
| Experiment assessment | [experiment-validation](experiment-validation/WORKFLOW.md) | Preflight, scheduled execution, independent verification |
| Diff review | [change-review](change-review/WORKFLOW.md) | Independent source review and impact-selected validation |
| Release acceptance | [release-validation](release-validation/WORKFLOW.md) | Acceptance only; no publication or deployment |
| Solution selection and prototype | [solution-prototyping](solution-prototyping/WORKFLOW.md) | Alternatives, risk prototype, experiment, decision |
| Software engineering/integration | [engineering-development](engineering-development/WORKFLOW.md) | Contracts, sole writer, integration and correction |
| Fault reproduction and repair | [fault-localization](fault-localization/WORKFLOW.md) | Causal evidence before a fix and regression |
| Performance optimization | [performance-optimization](performance-optimization/WORKFLOW.md) | Baseline/profile before a change and comparable measurements |
| Memory/resources/stability | [stability-validation](stability-validation/WORKFLOW.md) | Lifecycle, ownership, long-run and recovery evidence |
| Autonomous-driving scenarios | [scenario-regression](scenario-regression/WORKFLOW.md) | Trigger assertions, existing harness and evidence boundaries |

The foreground agent is the coordinator/Tech Lead, not another spawned role.
Use these workflows when the user requests them or a substantial lifecycle needs
repeatable gates. Do not auto-launch a fleet for a small fix or split one trace
among agents. Independent investigations may run concurrently; a shared design,
implementation, validation, and final acceptance require dependency barriers.

## Shared inputs and handoffs

[definitions.json](definitions.json) is the machine-readable stage manifest.
The [input schema](contracts/workflow-input.schema.json) binds task, repository
revision, file scope, fixtures, session storage, and required evidence.
Only task ID, description and source scope are mandatory; revision and artifact
storage default to the actual current HEAD and current session. Missing check
plans still run analysis/reporting but do not produce PASS.
[agent-result.schema.json](contracts/agent-result.schema.json) defines handoffs.
Each stage receives the task, exact input fingerprint, its specification, and
preceding stage results and revision history. Reports retain archived hashed artifact references;
do not put raw private logs, credentials, or personal data in handoffs.

Agents must read the repository guide and owning module's contracts, loading
only relevant skills. When knowledge is absent, use source/runtime evidence and
report the gap; do not invent commands or fetch/install dependencies implicitly.
Custom roles are defined in [.github/agents](../../.github/agents/README.md).
Select [domain profiles](profiles.json) for engineering, perception/model,
state estimation, planning/control, middleware, performance, memory/stability,
and scenarios. Profiles specify contracts and existing skills, not another
knowledge authority. The durable design and rollout rationale is in
[context](../../ad_agents/context/framework/validation/collaborative-engineering-workflows.md).

## Gates, ownership, and recovery

- Only `algorithm-implementer` may change source, and only in approved scope.
  Validators may write experiment artifacts, not source/configuration fixes.
  Tool permissions remain independent; these are cooperative constraints, not
  filesystem isolation.
- By default development pauses after design review, and execution workflows
  pause after preflight. Explicit `approval_mode: invocation` can authorize
  ordinary scoped non-GPU source work without another design pause. This does
  not approve resource-intensive experiments or bypass existing runtime gates.
  Checkpoint resume requires an operator-created approval receipt
  matching the run and checkpoint hash; resuming alone is not authorization.
- Runtime receipts bind arguments, source/configuration, explicit immutable
  inputs, declared external checkouts/environment, and archived artifacts.
  Do not put editable configuration in immutable inputs; include it in source
  scope instead. Missing environment/optional context warns, not blocks analysis.
  Changed inputs or interrupted side effects produce a report and reconciliation
  action; do not reuse cached proof.
- Failed or missing checks continue to independent review and reporting.
  A rejected/unavailable design skips source modification; unavailable source
  implementation skips candidate execution. The reviewer may explicitly request
  `next_action: revise` for actionable correction within approved scope.
  Default `max_revisions: 1` reruns writer, candidate validation and review once;
  `0` disables it. No blind test retries or concurrent writers.
- GPU work acquires one externally provisioned shared lock per resource before
  invoking the validator. All cooperating sessions must use the same absolute
  lock path; an external scheduler owns queueing and multi-host leases.
  Declare `uses_gpu: true` for GPU work. Without a lock, execution is skipped and
  the report is INCONCLUSIVE. Contention still permits review/reporting.
  On interruption or uncertain job
  termination, retain the lock until the operator verifies jobs have stopped.
  This does not reserve GPUs against unrelated processes.
- No automatic commits, publication, road/HIL execution, training, dependency
  installation, or runtime permission overrides. Approval covers only the named
  stage and existing permissions still apply.

## Acceptance

Stage completion is `COMPLETE`, not a test PASS. Final outcomes are computed by
the adapter, never by the reporter alone:

| Outcome | Meaning |
| --- | --- |
| PASS | All declared checks have intact evidence, independent review completed, and no unresolved critical/high finding |
| FAIL | A required check failed, a stage failed, or a critical/high defect remains |
| INCONCLUSIVE | Mandatory proof, independent verification, approval, execution, or job termination is unavailable |

Development/prototype/optimization, experiments, and release acceptance require at least one executed
test/metric check. Research can pass a source-backed decision, and a review can
pass source inspection without an unnecessary build; neither claims runtime
correctness. A successful command without its required scenario/assertion is
INCONCLUSIVE. Thresholds, repetitions, datasets, and baseline conditions must be
declared in required-check criteria and machine-readable metric `rules`, not
invented by the adapter. Metrics without rules stay INCONCLUSIVE, but their
analysis and report are retained. Optional checks (`mandatory: false`) warn
without determining acceptance.

Independent reviewers emit `verifications` binding check ID, candidate
fingerprint, original archived artifact hashes, status and rationale.
Critical/high findings receive IDs; only reviewer `resolutions` backed by
artifacts can close them. Earlier failed attempts remain in `history`; current
candidate proof and unresolved historical findings determine final acceptance.
Every ordinary completed path saves `report.json`, including FAIL and
INCONCLUSIVE. Operational interruption saves `failure.json` and checkpoint
references; cancellation/runtime failures still propagate.

Existing tools remain execution authority. Validators may supply
[harness reports](contracts/check-report.schema.json) through `check.result_file`;
the adapter parses these and applies declared rules instead of trusting the
agent's metric summaries. Manual tool records remain accepted during rollout
with explicit provenance limitations. Neither hashes nor JSON prove that a model
really ran a command; existing harness outputs and independent review are needed.

## Adapters and checks

[Copilot adapter](../../.github/extensions/autonomous-driving-workflows/README.md)
uses SDK Dynamic Workflows; `WORKFLOW.md` is not native executable configuration.
Codex or LangGraph may consume the same specifications and contracts, but cannot
execute Copilot's adapter as their own native workflow.

Configuration checks and deterministic mock regressions are not proof that a
model follows its instructions. Agent evaluation uses the existing
[reviewed-evidence tooling](../../scripts/agents/README.md); keep tuning and
holdout cases separate and record unexecuted behavioral cases as not run.
