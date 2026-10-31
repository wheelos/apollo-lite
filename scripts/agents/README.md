# Agent tooling

These standard-library tools check configuration, explicit input identity, and
reviewed evidence. They do not execute task commands, launch agents, fetch
dependencies, grant approval, or replace sim-agent. Durable contracts and command
authority remain in
[context](../../ad_agents/context/framework/validation/README.md).

## Configuration and preflight

Run local checks in the managed container as the mapped user:

```bash
bash docker/scripts/whl.sh exec dev -- python3 scripts/agents/check.py \
  --context-root /apollo/ad_agents/context
bash docker/scripts/whl.sh exec dev -- python3 scripts/agents/runtime.py \
  preflight --expected-uid "$(id -u)" --tool bazel
bash docker/scripts/whl.sh exec dev -- python3 -m unittest discover \
  -s scripts/agents/tests -v
```

`check.py` checks discoverable skills, required scalar metadata, entrypoint/index
coverage, local Markdown links and explicit backtick context paths. The current
repository uses only scalar `name` and `description` frontmatter; this checker
does not implement general YAML or the entire Agent Skills reference validator.
Extend it deliberately if richer metadata is introduced. It does not interpret
Markdown reference links, raw shell snippets, or template paths as executable
commands, and cannot detect arbitrary semantic contradictions between policies.

With `--context-root`, catalog paths and canonical routing documents are also
checked. Without it, external links are explicitly reported as `not_checked`;
this portable mode is used by CI without private checkout credentials.
After context changes, also use the existing inventory tool in its own checkout:

```bash
python3 ad_agents/context/tools/context_indexer.py sync
python3 ad_agents/context/tools/context_indexer.py check
```

Focused Python checks inside the managed container:

```bash
python3 -m black --check scripts/agents
python3 -m isort --check-only --profile black scripts/agents
python3 -m flake8 scripts/agents
```

`sync` rewrites only the generated inventory; review that diff. Inventory
validation and link validation are complementary. Preflight checks mapped UID,
the `/apollo` Docker workspace, declared tools, context presence, and existing
cache access. It cannot prove GPU, fixture, simulator, network, or runtime
readiness. Use the existing target-specific prerequisites for those claims.

CI is separately provisioned on GitHub-hosted runners and does not run Apollo
builds, fetch external repositories, or benchmark agent behavior.

## Versioning and checkpoints

[ad_agents.repos](../../ad_agents.repos) declares development checkouts tracking
`main`. Review it before using the existing vcs workflow; importing/updating is
not part of these tools. Record exact revisions and dirty content fingerprints
for runs instead of pretending `main` is an immutable delivery.
The checkout directory is ignored by the parent repository; review and commit
authorized context changes in the context repository independently.

Generate a fingerprint on stdout, then place it in a checkpoint in the session
workspace. The following is an example scope; add every actual checkout and
untracked/ignored input needed by the task:

```bash
python3 scripts/agents/runtime.py snapshot \
  --repo apollo=/apollo \
  --repo context=/apollo/ad_agents/context \
  --input /apollo/ad_agents.repos
```

Tracked checkout contents are hashed, not printed. Explicit extra inputs must
be absolute file paths. Untracked tools, new skills, configuration, fixtures,
and artifacts must be named with additional `--input` options until tracked.
The output contains local absolute paths: keep it in task storage, not public
logs or durable knowledge. Never include secret material or sensitive logs.

Checkpoint shape (replace placeholders with actual data, not fictional passes):

```json
{
  "schema_version": 1,
  "task_id": "<task ID>",
  "goal": "<observable outcome>",
  "scope": "<approved scope>",
  "status": "in_progress",
  "next_action": "<next bounded action>",
  "decisions": [],
  "blockers": [],
  "pending_approvals": [],
  "required_checks": ["<acceptance check>"],
  "fingerprint": {"repositories": {}, "inputs": []},
  "evidence": []
}
```

Replace `fingerprint` with the snapshot output. Each evidence item has `check`,
`command`, `status`, and `artifacts`; artifacts are objects containing absolute
`path` and actual file `sha256`. Complete status requires all required checks
passed with intact artifacts and no outstanding blockers/approvals.

```bash
python3 scripts/agents/runtime.py checkpoint /absolute/session/checkpoint.json
```

The checker recomputes input and artifact hashes before accepting the record.
It never resumes a workflow or executes the recorded command. A valid
in-progress checkpoint is not a completed task. See the
[checkpoint contract](../../ad_agents/context/framework/validation/agent-task-checkpoints.md).

## Agent evaluation

[evaluation-cases.json](evaluation-cases.json) is the initial rubric, NOT a
completed benchmark or a set of fully provisioned runtime fixtures. Before
running it, bind each selected setup to exact revisions, patches, inputs, and
permissions in an isolated workspace. No paid/model/hardware run is implicit.
Keep holdout cases out of tuning, even though their definitions are public.

An evaluation report has `schema_version: 1`, matching `suite_version`, non-empty
`run_id`, `configuration`, `reviewer`, and `cases`. Record actual model/runtime/
input versions in `configuration`, without secrets. For every completed case,
include its `id`, `status: "completed"`, boolean `gates` covering the exact case
definition, and hashed `artifacts`. A `not_run` case cannot claim gate results;
omitted cases remain not run.

```bash
python3 scripts/agents/runtime.py evaluate /absolute/session/evaluation.json
```

This aggregates reviewer-supplied judgments; it does not infer correctness from
transcripts or certify the reviewer. Missing cases and any false gate fail suite
acceptance, and hard-gate failures are separately reported. Follow the
[evaluation contract](../../ad_agents/context/framework/validation/agent-evaluation.md)
before making agent improvement claims.

Exit codes: `0` means the requested check/record passed its stated scope,
`1` means blocked or failed acceptance, and `2` means invalid inputs or an
operational error. JSON output always identifies the actual status.
