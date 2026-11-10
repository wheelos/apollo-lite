import assert from "node:assert/strict";
import { cp, mkdir, mkdtemp, readFile, rm, writeFile } from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { execFileSync } from "node:child_process";
import test, { after } from "node:test";
import { pathToFileURL } from "node:url";
import {
    acceptance, acquireLock, assertSchema, checkProven, definitions, digest,
    inputSchema, repository as sourceRepository, resultSchema, validateInput,
} from "./runtime.mjs";

const repository = await mkdtemp(path.join(os.tmpdir(), "apollo-workflow-repository-test-"));
after(() => rm(repository, { recursive: true, force: true }));
for (const relative of [".agents", ".github/agents",
    ".github/extensions/autonomous-driving-workflows"]) {
    await mkdir(path.dirname(path.join(repository, relative)), { recursive: true });
    await cp(path.join(sourceRepository, relative), path.join(repository, relative), { recursive: true });
}
await mkdir(path.join(repository, "modules"));
await writeFile(path.join(repository, "modules/fixture.txt"), "isolated source fixture");
await cp(path.join(sourceRepository, "AGENTS.md"), path.join(repository, "AGENTS.md"));
execFileSync("git", ["init", "--quiet", repository]);
execFileSync("git", ["add", "."], { cwd: repository });
execFileSync("git", ["-c", "user.name=Workflow Fixture", "-c", "user.email=fixture@example.invalid",
    "-c", "commit.gpgsign=false", "-c", "core.hooksPath=/dev/null",
    "commit", "--quiet", "-m", "Freeze isolated workflow fixture"], { cwd: repository });
const { runWorkflow } = await import(pathToFileURL(path.join(repository,
    ".github/extensions/autonomous-driving-workflows/runtime.mjs")));
const head = execFileSync("git", ["rev-parse", "HEAD"], { cwd: repository, encoding: "utf8" }).trim();
const check = (artifact, kind = "source") => ({
    id: "required", kind, status: "PASS", command: kind === "source" ? "" : "fixture-check",
    executed: kind !== "source", exit_code: kind === "source" ? null : 0,
    criteria_met: true, metrics: [], artifacts: [artifact],
});
const result = (identity, checks = []) => ({
    ...identity, status: "COMPLETE", summary: "Mock stage completed", findings: [],
    checks, limitations: [], jobs_stopped: true, next_action: "continue",
    verifications: [], resolutions: [],
});
const verifications = (handoffs) => handoffs.flatMap((item) => item.checks.map((evidence) => ({
    check_id: evidence.id, candidate_fingerprint: item.input_fingerprint,
    status: "VERIFIED", reason: "Read the frozen mock original artifact",
    artifacts: evidence.artifacts,
})));

async function fixture(t, name = "algorithm-research") {
    const root = await mkdtemp(path.join(os.tmpdir(), "apollo-workflow-test-"));
    t.after(() => rm(root, { recursive: true, force: true }));
    const proof = path.join(root, "proof.txt");
    await writeFile(proof, "frozen deterministic proof");
    const artifact = { path: proof, sha256: digest(await readFile(proof)) };
    const args = {
        task_id: "fixture", task_description: "Frozen mock orchestration, not an experiment",
        baseline_revision: head, scope: ["modules"], inputs: [proof], artifact_dir: root,
        required_checks: [{ id: "required", kind: name === "algorithm-research" ? "source" : "test",
            criteria: "Frozen mock proof only; no actual model or command execution is claimed" }],
        gpu_lock: null, approval_mode: "invocation",
    };
    const calls = [], phases = [], logs = [], paused = new Set();
    const ctx = {
        runId: "mock-run", args, session: { workspacePath: root },
        phase: (id) => phases.push(id), log: (line) => logs.push(line),
        parallel: async (thunks) => Promise.all(thunks.map(async (thunk) => {
            try { return await thunk(); } catch { return null; }
        })),
        pause: async (key) => {
            if (!paused.has(key)) { paused.add(key); throw Object.assign(new Error("MOCK_PAUSE"), { name: "AbortError" }); }
        },
        step: async (_key, producer) => producer(),
        agent: async (prompt, options) => {
            const identity = JSON.parse(prompt.match(/Identity: (.+)/)[1]);
            const history = JSON.parse(prompt.match(/Preceding handoffs and earlier revisions: (.+)/)[1]);
            calls.push({ ...options, identity, prompt });
            const output = result(identity, options.agent === "algorithm-validator"
                ? [check(artifact, args.required_checks[0]?.kind ?? "source")] : []);
            if (options.agent === "algorithm-reviewer") output.verifications = verifications(history);
            return output;
        },
    };
    const approve = async () => {
        const logged = logs.at(-1).match(/create (.+) with (\{.+\})\. Review/);
        await writeFile(logged[1], logged[2]);
    };
    return { root, proof, artifact, args, ctx, calls, phases, logs, approve };
}

test("all lifecycle definitions register without starting workflows and profiles/specs align", async () => {
    const entry = await readFile(new URL("./extension.mjs", import.meta.url), "utf8");
    const sdk = `export const defineWorkflow = x => x;
        export const joinSession = async x => { globalThis.__workflowRegistration = x; };`;
    const module = entry
        .replace('"@github/copilot-sdk/extension"', JSON.stringify(`data:text/javascript,${encodeURIComponent(sdk)}`))
        .replace('"./runtime.mjs"', JSON.stringify(new URL("./runtime.mjs", import.meta.url).href));
    await import(`data:text/javascript,${encodeURIComponent(module)}`);
    const registered = globalThis.__workflowRegistration.workflows;
    delete globalThis.__workflowRegistration;
    assert.equal(registered.length, 11);
    assert.deepEqual(registered.map(({ meta }) => meta.name), Object.keys(definitions));
    for (const { meta, run } of registered) {
        assert.equal(typeof run, "function");
        assert.deepEqual(meta.argsSchema, inputSchema);
        assert.equal(meta.limits, undefined);
        const spec = await readFile(path.join(repository, ".agents/workflows", meta.name, "WORKFLOW.md"), "utf8");
        for (const stage of definitions[meta.name].stages) {
            assert.ok(spec.includes(`| ${stage.id} |`), `${meta.name}: missing ${stage.id}`);
            for (const agent of stage.agents) {
                assert.ok(spec.includes(agent));
                const profile = await readFile(path.join(repository, ".github/agents", `${agent}.agent.md`), "utf8");
                assert.ok(profile.includes("Read root AGENTS.md"));
                assert.ok(profile.includes("include-custom-instructions: true"));
                assert.ok(!/^model:/m.test(profile));
            }
        }
    }
});

test("practice allows incomplete check plans but rejects unsafe input structure", async (t) => {
    const { args } = await fixture(t);
    validateInput("algorithm-research", args);
    validateInput("algorithm-development", { ...args, required_checks: [] });
    assert.throws(() => validateInput("algorithm-research", { ...args, baseline_revision: "main" }), /full Git SHA/);
    assert.throws(() => validateInput("algorithm-research", { ...args, scope: ["../other"] }), /scope/);
    assert.throws(() => validateInput("algorithm-research", { ...args, max_revisions: -1 }), /negative/);
    assert.throws(() => validateInput("algorithm-research", { ...args, profiles: ["invented"] }), /profile/);
    assert.throws(() => assertSchema({ status: "PASS" }, resultSchema), /missing/);
});

test("minimal invocation resolves actual revision and current-session storage without predicting a CLI session", async (t) => {
    const { ctx, calls, root } = await fixture(t);
    ctx.args = { task_id: "minimal", task_description: "Read-only frozen practice task", scope: ["modules"] };
    const output = await runWorkflow("algorithm-research", ctx);
    assert.equal(output.status, "INCONCLUSIVE");
    assert.equal(output.fingerprint.revision, head);
    assert.ok(output.checkpoint.startsWith(path.join(root, "files", "workflow-artifacts")));
    assert.ok(calls.some(({ agent }) => agent === "experiment-reporter"));
});

test("artifact storage outside the session is rejected before directory creation", async (t) => {
    const { args, ctx } = await fixture(t);
    args.artifact_dir = path.join(os.tmpdir(), `outside-${path.basename(ctx.session.workspacePath)}`);
    await assert.rejects(runWorkflow("algorithm-research", ctx), /inside the session workspace/);
    await assert.rejects(readFile(args.artifact_dir), { code: "ENOENT" });
});

test("declared metric limits cannot be overridden by PASS/criteria_met claims", () => {
    const artifact = { path: "/synthetic", sha256: "a".repeat(64) };
    const measured = { ...check(artifact, "metric"), metrics: [{ name: "latency", value: 100, unit: "ms" }] };
    const validator = result({ agent: "algorithm-validator", stage: "validation", input_fingerprint: "fp" }, [measured]);
    const reviewer = { ...result({ agent: "algorithm-reviewer", stage: "independent-review" }),
        verifications: verifications([validator]) };
    const args = { required_checks: [{ id: "required", kind: "metric", criteria: "latency <= 10 ms",
        rules: [{ metric: "latency", unit: "ms", operator: "le", limit: 10 }] }] };
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "FAIL");
    measured.metrics[0].value = 5;
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "PASS");
    measured.metrics[0].unit = "s";
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "INCONCLUSIVE");
    args.required_checks[0].rules = [];
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "INCONCLUSIVE");
});

test("independent verification binds candidate and every artifact, not just COMPLETE", () => {
    const validator = result({ agent: "algorithm-validator", stage: "validation", input_fingerprint: "fp" },
        [check({ path: "/fixture", sha256: "a".repeat(64) }, "test")]);
    const reviewer = result({ agent: "algorithm-reviewer", stage: "independent-review" });
    const args = { required_checks: [{ id: "required", kind: "test" }] };
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "INCONCLUSIVE");
    reviewer.verifications = verifications([validator]);
    reviewer.verifications[0].candidate_fingerprint = "wrong";
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "INCONCLUSIVE");
    reviewer.verifications = verifications([validator]);
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "PASS");
    reviewer.verifications[0].artifacts = [];
    assert.equal(acceptance("algorithm-development", args, [validator, reviewer], true), "INCONCLUSIVE");
});

test("unexecuted checks, missing assertions/metrics/artifacts never prove a pass", () => {
    const passed = check({ path: "/fixture", sha256: "a".repeat(64) }, "test");
    assert.equal(checkProven(passed), true);
    for (const change of [{ executed: false }, { criteria_met: false }, { exit_code: 1 },
        { command: "" }, { artifacts: [] }, { kind: "metric", metrics: [] }]) {
        assert.equal(checkProven({ ...passed, ...change }), false);
    }
});

test("shared locks reject collisions and preserve owner", async (t) => {
    const { root } = await fixture(t);
    const file = path.join(root, "gpu.lock");
    const release = await acquireLock(file, { run_id: "first" });
    assert.equal(await acquireLock(file, { run_id: "second" }), null);
    assert.equal(JSON.parse(await readFile(file)).run_id, "first");
    await release();
    const next = await acquireLock(file, { run_id: "second" });
    await next();
});

test("research barrier, independent proof, persistent report and receipt replay", async (t) => {
    const { ctx, calls, phases, root } = await fixture(t);
    const output = await runWorkflow("algorithm-research", ctx);
    assert.equal(output.status, "PASS", JSON.stringify(output));
    assert.deepEqual(phases, ["research", "decision-review", "report"]);
    assert.equal(new Set(calls.map(({ label }) => label)).size, calls.length);
    const saved = JSON.parse(await readFile(path.join(root, ctx.runId, "report.json")));
    assert.equal(saved.status, output.status);
    const before = calls.length;
    assert.equal((await runWorkflow("algorithm-research", ctx)).status, "PASS");
    assert.equal(calls.length, before);
});

test("missing agents/check plan still reaches independent review and report", async (t) => {
    const { ctx, calls, args } = await fixture(t);
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => options.agent === "algorithm-validator"
        ? null : original(prompt, options);
    args.required_checks = [];
    const output = await runWorkflow("algorithm-research", ctx);
    assert.equal(output.status, "INCONCLUSIVE");
    assert.ok(calls.some(({ agent }) => agent === "experiment-reporter"));
    assert.ok(output.warnings.some((warning) => warning.includes("No required checks")));
});

test("checkpoint approval precedes writer while invocation approval permits normal source flow", async (t) => {
    const { ctx, args, calls, approve } = await fixture(t, "algorithm-development");
    args.approval_mode = "checkpoint";
    await assert.rejects(runWorkflow("algorithm-development", ctx), /MOCK_PAUSE/);
    assert.ok(!calls.some(({ agent }) => agent === "algorithm-implementer"));
    await approve();
    assert.equal((await runWorkflow("algorithm-development", ctx)).status, "PASS");
    assert.equal(calls.filter(({ agent }) => agent === "algorithm-implementer").length, 1);
    const count = calls.length;
    assert.equal((await runWorkflow("algorithm-development", ctx)).status, "PASS");
    assert.equal(calls.length, count);
});

test("failed candidate still generates a final FAIL report with no blind retries", async (t) => {
    const { ctx, calls } = await fixture(t, "engineering-development");
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => {
        const output = await original(prompt, options);
        if (options.agent === "algorithm-validator") output.checks[0].exit_code = 1;
        return output;
    };
    const output = await runWorkflow("engineering-development", ctx);
    assert.equal(output.status, "FAIL");
    assert.ok(calls.some(({ agent }) => agent === "experiment-reporter"));
    assert.equal(calls.filter(({ agent }) => agent === "algorithm-validator").length, 1);
});

test("reviewer-directed correction reruns candidate proof and resolves historical finding", async (t) => {
    const { ctx, calls } = await fixture(t, "engineering-development");
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => {
        const output = await original(prompt, options);
        const round = Number(prompt.match(/Round: (\d+)/)[1]);
        if (output.stage === "validation" && round === 0) {
            output.checks[0].status = "FAIL";
            output.findings = [{ id: "defect", severity: "high", summary: "Frozen mock defect",
                source: "fixture:1", unresolved: true }];
        }
        if (output.stage === "independent-review") {
            if (round === 0) output.next_action = "revise";
            else output.resolutions = [{ finding_id: "defect", candidate_fingerprint: output.input_fingerprint,
                reason: "Read new frozen candidate proof",
                artifacts: output.verifications.at(-1).artifacts }];
        }
        return output;
    };
    const output = await runWorkflow("engineering-development", ctx);
    assert.equal(output.status, "PASS", JSON.stringify(output));
    assert.equal(output.revisions, 1);
    assert.equal(calls.filter(({ agent }) => agent === "algorithm-implementer").length, 2);
    assert.equal(calls.filter(({ agent }) => agent === "algorithm-validator").length, 2);
    assert.ok(output.history.some((item) => item.checks.some((item) => item.status === "FAIL")));
    const count = calls.length;
    assert.equal((await runWorkflow("engineering-development", ctx)).status, "PASS");
    assert.equal(calls.length, count, "resume must not rerun research or the corrected writer");
});

test("unfulfilled revision cannot PASS and the caller can disable correction", async (t) => {
    const { ctx, args, calls } = await fixture(t, "engineering-development");
    args.max_revisions = 0;
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => {
        const output = await original(prompt, options);
        if (output.stage === "independent-review") output.next_action = "revise";
        return output;
    };
    const output = await runWorkflow("engineering-development", ctx);
    assert.equal(output.status, "INCONCLUSIVE");
    assert.equal(calls.filter(({ agent }) => agent === "algorithm-implementer").length, 1);
});

test("mutable source proof is archived and changed immutable inputs produce a recoverable report", async (t) => {
    const { ctx, args, proof } = await fixture(t);
    args.inputs = [];
    const output = await runWorkflow("algorithm-research", ctx);
    const archived = output.history.find((item) => item.agent === "algorithm-validator").checks[0].artifacts[0].path;
    assert.notEqual(archived, proof);
    await writeFile(proof, "changed original");
    assert.equal((await runWorkflow("algorithm-research", ctx)).status, "PASS");
});

test("stale explicit inputs are not reused and operational interruptions retain a report", async (t) => {
    const { ctx, proof, root } = await fixture(t);
    await runWorkflow("algorithm-research", ctx);
    await writeFile(proof, "changed input");
    const blocked = await runWorkflow("algorithm-research", ctx);
    assert.equal(blocked.status, "INCONCLUSIVE");
    assert.match(blocked.blocker, /Checkpoint inputs/);
    assert.ok(JSON.parse(await readFile(path.join(root, ctx.runId, "failure.json"))).next_action);
});

test("stale evidence is inconclusive but review/report still complete", async (t) => {
    const { ctx } = await fixture(t);
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => {
        const output = await original(prompt, options);
        if (output.checks.length) output.checks[0].artifacts[0].sha256 = "0".repeat(64);
        return output;
    };
    const output = await runWorkflow("algorithm-research", ctx);
    assert.equal(output.status, "INCONCLUSIVE");
    assert.ok(output.results.some((item) => item.agent === "experiment-reporter"));
    assert.ok(output.warnings.some((warning) => warning.includes("Stale artifact")));
});

test("GPU contention launches no validator but still reports; uncertain jobs retain the GPU lock", async (t) => {
    const { ctx, args, calls, approve } = await fixture(t, "stability-validation");
    const gpuDirectory = await mkdtemp(path.join(os.tmpdir(), "apollo-workflow-gpu-test-"));
    t.after(() => rm(gpuDirectory, { recursive: true, force: true }));
    args.uses_gpu = true;
    args.gpu_lock = path.join(gpuDirectory, "resource.lock");
    await assert.rejects(runWorkflow("stability-validation", ctx), /MOCK_PAUSE/);
    await approve();
    const release = await acquireLock(args.gpu_lock, { run_id: "another-task" });
    const busy = await runWorkflow("stability-validation", ctx);
    assert.equal(busy.status, "INCONCLUSIVE");
    assert.ok(!calls.some(({ agent }) => agent === "algorithm-validator"));
    assert.ok(calls.some(({ agent }) => agent === "experiment-reporter"));
    await release();
    ctx.runId = "another-mock-run";
    await assert.rejects(runWorkflow("stability-validation", ctx), /MOCK_PAUSE/);
    await approve();
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => ({ ...await original(prompt, options),
        jobs_stopped: options.agent !== "algorithm-validator" });
    const uncertain = await runWorkflow("stability-validation", ctx);
    assert.equal(uncertain.status, "INCONCLUSIVE");
    assert.equal(JSON.parse(await readFile(args.gpu_lock)).run_id, ctx.runId);
});

test("harness report overrides agent metric claims and binds candidate identity", async (t) => {
    const { ctx, args, artifact, root } = await fixture(t, "engineering-development");
    args.required_checks[0] = { id: "required", kind: "metric", criteria: "latency budget",
        rules: [{ metric: "latency", unit: "ms", operator: "le", limit: 10 }] };
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => {
        const output = await original(prompt, options);
        if (options.agent === "algorithm-validator") {
            const file = path.join(root, "harness.json");
            await writeFile(file, JSON.stringify({ schema_version: 1, check_id: "required",
                candidate_fingerprint: output.input_fingerprint, executed: true,
                command: "frozen-mock-harness", exit_code: 0, criteria_met: true,
                metrics: [{ name: "latency", value: 100, unit: "ms" }], artifacts: [artifact] }));
            output.checks[0].metrics = [{ name: "latency", value: 1, unit: "ms" }];
            output.checks[0].result_file = file;
        }
        return output;
    };
    const output = await runWorkflow("engineering-development", ctx);
    assert.equal(output.status, "FAIL");
    assert.equal(output.evaluation.checks[0].status, "FAIL");
    assert.ok(output.evaluation.checks[0].reason.includes("100"));
});

test("optional unavailable evidence warns without defeating proven mandatory checks", async (t) => {
    const { ctx, args } = await fixture(t);
    args.required_checks.push({ id: "optional", kind: "source", criteria: "Optional coverage",
        mandatory: false });
    const output = await runWorkflow("algorithm-research", ctx);
    assert.equal(output.status, "PASS");
    assert.ok(output.warnings.some((warning) => warning.includes("optional")));
});

test("declared GPU intent without a resource lock skips execution and still produces a report", async (t) => {
    const { ctx, args, calls, approve } = await fixture(t, "scenario-regression");
    args.uses_gpu = true;
    await assert.rejects(runWorkflow("scenario-regression", ctx), /MOCK_PAUSE/);
    await approve();
    const output = await runWorkflow("scenario-regression", ctx);
    assert.equal(output.status, "INCONCLUSIVE");
    assert.ok(!calls.some(({ agent }) => agent === "algorithm-validator"));
    assert.ok(calls.some(({ agent }) => agent === "experiment-reporter"));
});

test("rejected design cannot admit source writes but still completes reporting", async (t) => {
    const { ctx, calls } = await fixture(t, "engineering-development");
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => {
        const output = await original(prompt, options);
        if (output.stage === "design-review") output.next_action = "stop";
        return output;
    };
    const output = await runWorkflow("engineering-development", ctx);
    assert.equal(output.status, "INCONCLUSIVE");
    assert.ok(!calls.some(({ agent }) => agent === "algorithm-implementer"));
    assert.ok(calls.some(({ agent }) => agent === "experiment-reporter"));
});

test("performance baseline precedes its sole writer and candidate validation", async (t) => {
    const { ctx, calls, phases } = await fixture(t, "performance-optimization");
    const output = await runWorkflow("performance-optimization", ctx);
    assert.equal(output.status, "PASS");
    assert.deepEqual(phases, ["research", "baseline-profile", "design-review", "implementation",
        "validation", "independent-review", "report"]);
    assert.ok(calls.find(({ identity }) => identity.stage === "baseline-profile").prompt.includes("performance"));
});

test("GPU performance baseline pauses for approval before profiling", async (t) => {
    const { ctx, args, calls } = await fixture(t, "performance-optimization");
    args.uses_gpu = true;
    await assert.rejects(runWorkflow("performance-optimization", ctx), /MOCK_PAUSE/);
    assert.ok(!calls.some(({ agent }) => agent === "algorithm-validator"));
});

test("a requested design revision cannot be bypassed by the later correction loop", async (t) => {
    const { ctx, calls } = await fixture(t, "engineering-development");
    const original = ctx.agent;
    ctx.agent = async (prompt, options) => {
        const output = await original(prompt, options);
        if (["design-review", "independent-review"].includes(output.stage)) output.next_action = "revise";
        return output;
    };
    const output = await runWorkflow("engineering-development", ctx);
    assert.equal(output.status, "INCONCLUSIVE");
    assert.equal(output.revisions, 0);
    assert.ok(!calls.some(({ agent }) => agent === "algorithm-implementer"));
    assert.ok(calls.some(({ agent }) => agent === "experiment-reporter"));
});
