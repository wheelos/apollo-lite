# Autonomous driving workflows: Copilot adapter

Implements the [shared specifications](../../../.agents/workflows/README.md)
using the experimental
[SDK workflow API](https://github.com/github/copilot-sdk/blob/main/nodejs/docs/workflows.md)
and [extension discovery](https://github.com/github/copilot-sdk/blob/main/nodejs/docs/extensions.md).
The CLI injects `@github/copilot-sdk`; no package installation is required.
GitHub Actions under `.github/workflows/` are unrelated.

## Registration and execution

`extension.mjs` registers all names from
[definitions.json](../../../.agents/workflows/definitions.json) with `joinSession`.
Importing the extension registers definitions only: it does not run agents,
experiments, commands, or workflows. Runtime modules use only Node built-ins.

Open a trusted repository session with `copilot --experimental`; review the
extension before granting load permission. On the installed CLI, extension
discovery happens at session start/replacement. Use `/env` to inspect extensions,
then ask for available Dynamic Workflows. If extension reload tools are provided
by your runtime, use their documented reload/list/inspect operations. This
session's workflow guide reported that extensions were not configured; writing
these files cannot hot-register them into that session. A separate installed CLI
process successfully loaded this extension with zero reported failures. All five
initial names were recognized with invalid `{}` arguments and rejected before execution;
no workflow agents or experiments were started by that registration check.

The input contract is shared by all names:

| Input | Structure |
| --- | --- |
| task_id / task_description | Nonempty task identifier and bounded task |
| baseline_revision | Optional full current HEAD SHA; defaults to actual HEAD, dirty content is separately fingerprinted |
| scope | Literal repository-relative source paths/directories; no globs or external checkout scopes |
| inputs | Optional explicit absolute fixture/model/configuration files, including otherwise ignored inputs; defaults to [] |
| artifact_dir | Optional absolute session directory; defaults to this run's session workspace files/workflow-artifacts |
| required_checks | Unique `{id, kind: source/test/metric, criteria}` entries; defaults to [] with INCONCLUSIVE acceptance and a useful report |
| gpu_lock | `null` for non-GPU execution; otherwise provisioned shared absolute resource lock path |
| uses_gpu | Explicit GPU intent; true without a shared lock skips execution, not reporting |
| approval_mode | `checkpoint` (default), or explicit `invocation` for ordinary scoped non-GPU implementation |
| max_revisions | Reviewer-directed corrections, default 1; 0 disables corrections |
| profiles | Relevant IDs from the shared profile manifest |
| environment / repositories | Optional runtime name/value identity and pinned absolute external checkout paths |

Only `task_id`, `task_description`, and `scope` are mandatory. Defaults are
resolved from the real current repository/session, never fabricated. A direct
CLI run need not predict the newly created session's artifact directory.

Paths, revisions, criteria, and resources must come from the actual task. Do not
turn placeholder inputs into a run. In prompt mode, retain narrowly scoped
permissions; never use all-permissions flags just to make a workflow run.

The installed CLI advertises:

```text
copilot --experimental workflow run algorithm-research --args @/absolute/task-input.json
```

Run only after explicit task approval and permission configuration. This command
is an execution example, not part of adapter validation. The CLI's run command
does not provide interactive permission approvals. Observe/resume using the
workflow facilities exposed by your runtime; this adapter does not start or
resume its own runs. It neither registers global user configuration nor changes
experimental settings.

## Stages and outputs

| Workflow | Agents in dependency order |
| --- | --- |
| algorithm-research | researcher + validator (parallel) -> reviewer -> reporter |
| algorithm-development | researcher -> reviewer/approval -> implementer -> validator -> independent reviewer -> reporter |
| experiment-validation | researcher/preflight/approval -> validator -> independent reviewer -> reporter |
| change-review | reviewer + validator (parallel) -> independent reviewer -> reporter |
| release-validation | researcher/preflight/approval -> validator -> independent reviewer -> reporter |
| solution-prototyping | researcher/options -> reviewer/design -> implementer/prototype -> validator -> selection reviewer -> reporter |
| engineering-development | researcher -> design reviewer -> implementer -> validator/integration -> independent reviewer -> reporter |
| fault-localization | researcher -> validator/reproducer -> design reviewer -> implementer -> validator/regression -> independent reviewer -> reporter |
| performance-optimization | researcher -> validator/baseline-profile -> design reviewer -> implementer -> validator/comparison -> independent reviewer -> reporter |
| stability-validation | researcher/preflight -> validator/lifecycle-resources -> independent reviewer -> reporter |
| scenario-regression | researcher/preflight -> validator/existing harness -> independent reviewer -> reporter |

Names above abbreviate `algorithm-*` roles and `experiment-reporter`.
Each handoff includes identity/fingerprint, `COMPLETE/FAIL/INCONCLUSIVE`, summary,
source-backed findings, checks (commands, actual execution/exit/assertion status,
metrics, hashed artifacts), limitations, and job termination status.

Final output includes `{workflow, task_id, run_id, status, results, history,
evaluation, revisions, warnings, fingerprint, checkpoint, blocker, next_action}`.
`status` is `PASS/FAIL/INCONCLUSIVE`; required executed
checks may only originate from the validator. The reporter cannot manufacture
acceptance. The adapter verifies artifact bytes and structured proof fields;
the independent reviewer must emit check-level `verifications` matching exact
candidate and archived artifacts. Required metric rules are evaluated by code.
This is not a machine-verifiable certificate that a model actually ran a command.

## Checkpoints, approval, and locks

Receipts live in `artifact_dir/run_id/checkpoint.json` with input/source hashes
and stage outputs. SDK `ctx.agent` supplies durable agent calls, `ctx.pause`
supplies operator checkpoints, and `ctx.step` journals the final report.
On resume, current source, configuration, explicit inputs, and referenced
artifact hashes are checked before downstream agents run. Interrupted stages
fail closed because journaled side effects are not exactly-once.

After design review/preflight the workflow logs an exact approval JSON shape
and pauses. The operator reviews the checkpoint, creates the approval file in
the run directory, then resumes. The receipt is an explicit operator attestation,
not cryptographic identity verification, and does not bypass tool permissions.
Changed designs or inputs require a new run, not editing approval hashes to
reuse stale proof.

One cooperative session/workflow lock prevents concurrent replay. An exclusive
Git-directory writer lock serializes implementation across cooperating sessions
in the same worktree; it is retained on interruption until side effects are
reconciled. Source scope fingerprints reject writes by non-writer stages and
out-of-scope writer changes. Use disjoint worktrees for independent source tasks;
this is not an OS write-access restriction.

GPU validators acquire the user-provisioned shared exclusive lock. Contention
returns INCONCLUSIVE without launching the agent; the external scheduler queues
tasks and decides when to start a new run. No guessed resource/credit limits
are registered. Interruption or unproven job termination retains the GPU lock.
Remove a stale lock only after inspecting its owner and proving all jobs stopped.
Locks require shared filesystem semantics; use a real scheduler for multi-host
work. Source/fixture changes, unsupported evidence, and missing approvals never
trigger blind retries. Ordinary gaps/failed checks still produce review/report
results. A reviewer can request one scoped correction by default; the writer,
validation and review rerun with separate round receipts. Finding resolutions
need reviewer rationale and evidence. Additional correction requires an explicit
`max_revisions` policy supplied by the caller.

Use optional `profiles`, `environment` and `repositories` to declare relevant
contracts and reproducibility conditions. Omitted identity is reported, not a
reason to abort analysis. Immutable `inputs` must exclude configuration being
edited; source scope fingerprints cover editable configuration.

Existing harnesses may emit
[check reports](../../../.agents/workflows/contracts/check-report.schema.json).
Set `check.result_file` to the original report under artifact_dir; the adapter
uses its candidate-bound execution/metric fields and archives report/evidence
bytes. A manual tool record is still usable, with a visible provenance warning.
The adapter never directly executes arbitrary test shell commands.
Harness reports may provide `status` to distinguish failed assertions from
unavailable triggers/measurements; nonzero executed exit codes remain FAIL.
Changing acceptance claims cannot override deterministic metric rules.

### Minimal practice inputs

Normal scoped non-GPU development can set `approval_mode: invocation`, avoiding
an extra design checkpoint while retaining runtime permissions. Experiments and
preflight approval still pause. `max_revisions: 1` permits a single explicit
reviewer correction; no repeated tests until they pass.

An invocation can start with this shape (replace the illustrative task with the
actual explicitly approved task):

```json
{
  "task_id": "scoped-prototype",
  "task_description": "Build the approved minimum prototype and report feasibility gaps",
  "scope": ["modules/example"],
  "approval_mode": "invocation",
  "max_revisions": 1,
  "profiles": ["engineering"],
  "required_checks": []
}
```

No fictional revision or session path is needed. The illustrative source scope
must be replaced by real approved paths. An empty evidence plan intentionally
cannot PASS; add the actual required source/test/metric checks for acceptance.

Metric checks declare rules such as:

```json
{
  "id": "latency",
  "kind": "metric",
  "criteria": "The declared workload meets its latency budget",
  "rules": [{"metric": "p99_ms", "unit": "ms", "operator": "le", "limit": 10}]
}
```

The number above is an illustrative task criterion, not a repository-wide
threshold. A measurement of 100 cannot pass it even if the agent claims PASS.
Source/test checks still require meaningful declared assertions and original
evidence; units/criteria missing from a metric report yield INCONCLUSIVE.

## Local checks and evidence limits

Run the existing portable configuration checker in the managed container:

```bash
bash docker/scripts/whl.sh exec dev -- python3 scripts/agents/check.py
```

`node --test .github/extensions/autonomous-driving-workflows/runtime.test.mjs`
exercises orchestration with mocked agents, not algorithm experiments or paid
model runs. The tests freeze an independent temporary Git checkout so concurrent
edits in a shared development worktree cannot invalidate mock evidence.
It needs an existing Node runtime. Live SDK registration was checked
separately as described above; actual agent dispatch, pause/resume against the
SDK, and instruction-effectiveness evaluation were not run. Do not label those
as passed from mock results. The repository's external context/evaluation
contract must be provisioned before full knowledge-link or agent benchmark
validation.
