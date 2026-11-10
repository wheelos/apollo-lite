import { createHash } from "node:crypto";
import { createReadStream } from "node:fs";
import { execFile } from "node:child_process";
import {
    copyFile, lstat, mkdir, open, readFile, readlink, realpath, rename, unlink,
} from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { promisify } from "node:util";
import { evaluateAcceptance } from "./evaluation.mjs";
export { checkProven } from "./evaluation.mjs";

const execute = promisify(execFile);
export const repository = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../../..");
const specs = path.join(repository, ".agents/workflows");
const json = async (file) => JSON.parse(await readFile(file, "utf8"));
export const definitions = await json(path.join(specs, "definitions.json"));
export const inputSchema = await json(path.join(specs, "contracts/workflow-input.schema.json"));
export const resultSchema = await json(path.join(specs, "contracts/agent-result.schema.json"));
const reportSchema = await json(path.join(specs, "contracts/check-report.schema.json"));
export const profiles = await json(path.join(specs, "profiles.json"));
export const digest = (value) => createHash("sha256").update(value).digest("hex");
const objectHash = (value) => digest(JSON.stringify(value));
const within = (root, file) => file === root || file.startsWith(`${root}${path.sep}`);
const nonempty = (value) => typeof value === "string" && value.trim().length > 0;
const runtimeProof = new Set(["algorithm-development", "engineering-development", "solution-prototyping",
    "fault-localization", "performance-optimization", "stability-validation", "scenario-regression",
    "experiment-validation", "release-validation"]);

export function assertSchema(value, schema, location = "result") {
    const types = Array.isArray(schema.type) ? schema.type : [schema.type];
    const actual = value === null ? "null" : Array.isArray(value) ? "array" : typeof value;
    if (!types.some((type) => type === actual || (type === "integer" && Number.isInteger(value)))) {
        throw new Error(`${location}: invalid type`);
    }
    if (schema.enum && !schema.enum.includes(value)) throw new Error(`${location}: invalid enum`);
    if (actual === "object") {
        for (const key of schema.required ?? []) {
            if (!Object.hasOwn(value, key)) throw new Error(`${location}: missing ${key}`);
        }
        for (const [key, child] of Object.entries(schema.properties ?? {})) {
            if (Object.hasOwn(value, key)) assertSchema(value[key], child, `${location}.${key}`);
        }
    }
    if (actual === "array") value.forEach((item, i) => assertSchema(item, schema.items, `${location}[${i}]`));
    if (actual === "number" && !Number.isFinite(value)) throw new Error(`${location}: nonfinite number`);
}

export function validateInput(name, args) {
    assertSchema(args, inputSchema, "args");
    for (const key of ["task_id", "task_description", "baseline_revision", "artifact_dir"]) {
        if (!nonempty(args[key])) throw new Error(`args: empty ${key}`);
    }
    if (!/^[a-zA-Z0-9][a-zA-Z0-9_-]*$/.test(args.task_id)) throw new Error("args: invalid task_id");
    if (!/^[a-f0-9]{40}$/.test(args.baseline_revision)) throw new Error("args: pin a full Git SHA");
    if (!args.scope.length || args.scope.some((file) =>
        !nonempty(file) || path.isAbsolute(file) || file.split("/").includes("..") ||
        file !== path.posix.normalize(file) || /[*?\[\]\\]/.test(file))) {
        throw new Error("args: scope must contain literal repository-relative paths");
    }
    if (!path.isAbsolute(args.artifact_dir) || within(repository, path.resolve(args.artifact_dir))) {
        throw new Error("args: artifact_dir must be absolute and outside the repository");
    }
    if (args.inputs.some((file) => !nonempty(file) || !path.isAbsolute(file))) {
        throw new Error("args: inputs must be explicit absolute file paths");
    }
    const ids = args.required_checks.map((check) => check.id);
    if (new Set(ids).size !== ids.length ||
        args.required_checks.some((check) => !nonempty(check.id) || !nonempty(check.criteria))) {
        throw new Error("args: declare unique nonempty required checks and criteria");
    }
    if (args.max_revisions !== undefined && args.max_revisions < 0) throw new Error("args: max_revisions cannot be negative");
    if (args.profiles?.some((profile) => !profiles[profile])) throw new Error("args: unknown domain profile");
    for (const check of args.required_checks) {
        if (check.stage && !definitions[name]?.stages.some((stage) => stage.id === check.stage)) {
            throw new Error(`args: unknown check stage ${check.stage}`);
        }
        if (check.rules?.some((rule) => !nonempty(rule.metric) || !nonempty(rule.unit))) {
            throw new Error("args: metric rules need names and units");
        }
    }
    if (args.repositories?.some((repo) => !path.isAbsolute(repo.path) || !/^[a-f0-9]{40}$/.test(repo.revision))) {
        throw new Error("args: external repositories need absolute paths and pinned revisions");
    }
    if (args.gpu_lock !== null &&
        (!path.isAbsolute(args.gpu_lock) || within(repository, path.resolve(args.gpu_lock)) ||
        within(path.resolve(args.artifact_dir), path.resolve(args.gpu_lock)))) {
        throw new Error("args: gpu_lock must be a provisioned shared path outside source/artifacts");
    }
}

async function gitAt(root, ...args) {
    const { stdout } = await execute("git", args, {
        cwd: root, encoding: "buffer", maxBuffer: 64 * 1024 * 1024,
    });
    return stdout;
}
const git = (...args) => gitAt(repository, ...args);

async function fileHash(file) {
    const stat = await lstat(file);
    if (stat.isSymbolicLink()) return digest(`symlink:${await readlink(file)}`);
    if (!stat.isFile()) throw new Error(`Expected a regular file: ${file}`);
    const hash = createHash("sha256");
    for await (const chunk of createReadStream(file)) hash.update(chunk);
    return hash.digest("hex");
}

async function snapshot(args) {
    const revision = (await git("rev-parse", "HEAD")).toString().trim();
    if (revision !== args.baseline_revision) throw new Error("Revision changed; start a newly pinned run");
    const files = [...new Set((await git("ls-files", "--cached", "--others", "--exclude-standard", "-z"))
        .toString().split("\0").filter(Boolean))].sort();
    const inside = [], outside = [], configuration = [];
    for (const file of files) {
        let hash, mode = null;
        try {
            hash = await fileHash(path.join(repository, file));
            mode = (await lstat(path.join(repository, file))).mode;
        } catch (error) {
            if (error.code !== "ENOENT") throw error;
            hash = "deleted";
        }
        const row = [file, hash, mode];
        const scoped = args.scope.some((scope) => scope === "." || file === scope || file.startsWith(`${scope}/`));
        (scoped ? inside : outside).push(row);
        if (file === "AGENTS.md" || file === ".github/copilot-instructions.md" ||
            file.startsWith(".agents/") || file.startsWith(".github/agents/") ||
            file.startsWith(".github/extensions/")) configuration.push(row);
    }
    const inputs = [];
    for (const file of args.inputs) inputs.push([file, await fileHash(await realpath(file))]);
    const dependencies = [];
    for (const repo of args.repositories ?? []) {
        const revision = (await gitAt(repo.path, "rev-parse", "HEAD")).toString().trim();
        if (revision !== repo.revision) throw new Error(`External revision changed: ${repo.path}`);
        const names = [...new Set((await gitAt(repo.path, "ls-files", "--cached", "--others", "--exclude-standard", "-z"))
            .toString().split("\0").filter(Boolean))].sort();
        const contents = [];
        for (const file of names) {
            try { contents.push([file, await fileHash(path.join(repo.path, file))]); } catch (error) {
                if (error.code !== "ENOENT") throw error;
                contents.push([file, "deleted"]);
            }
        }
        dependencies.push([repo.path, revision, objectHash(contents)]);
    }
    return { revision, index: digest(await git("diff", "--cached", "--binary", "--no-ext-diff", "--no-textconv")),
        inside: objectHash(inside), outside: objectHash(outside),
        configuration: objectHash(configuration), inputs: objectHash(inputs),
        dependencies: objectHash(dependencies), environment: objectHash(args.environment ?? []) };
}

async function save(file, value) {
    const temp = `${file}.tmp`;
    const handle = await open(temp, "w", 0o600);
    try { await handle.writeFile(JSON.stringify(value, null, 2)); } finally { await handle.close(); }
    await rename(temp, file);
}

export async function acquireLock(file, owner) {
    let handle;
    try {
        handle = await open(file, "wx", 0o600);
    } catch (error) {
        if (error.code === "EEXIST") return null;
        throw error;
    }
    try { await handle.writeFile(JSON.stringify(owner)); } finally { await handle.close(); }
    return async () => {
        if (objectHash(await json(file)) !== objectHash(owner)) throw new Error(`Lock owner changed: ${file}`);
        await unlink(file);
    };
}

export function acceptance(name, args, results, complete) {
    return evaluateAcceptance(name, args, results, complete, runtimeProof.has(name)).status;
}

async function artifactPath(artifact, args) {
    if (!path.isAbsolute(artifact.path) || !/^[a-f0-9]{64}$/.test(artifact.sha256)) {
        throw new Error("Artifact must have an absolute path and actual SHA-256");
    }
    const resolved = await realpath(artifact.path);
    const extra = [...args.inputs, ...(args.repositories ?? []).map((repo) => repo.path)];
    if (!within(repository, resolved) && !within(args.artifact_dir, resolved) &&
        !extra.some((root) => within(root, resolved))) throw new Error("Artifact lies outside declared evidence scope");
    if (await fileHash(resolved) !== artifact.sha256) throw new Error(`Stale artifact: ${artifact.path}`);
    return resolved;
}

async function archivedArtifact(artifact, args, directory) {
    const source = await artifactPath(artifact, args);
    const target = path.join(directory, "evidence", `${artifact.sha256}-${path.basename(source)}`);
    await mkdir(path.dirname(target), { recursive: true, mode: 0o700 });
    if (source !== target) await copyFile(source, target);
    if (await fileHash(target) !== artifact.sha256) throw new Error("Artifact changed while archiving");
    return { path: target, sha256: artifact.sha256 };
}

async function validateResult(result, identity, args, directory, archive = false) {
    if (result === null) return {
        ...identity, status: "INCONCLUSIVE", summary: "Agent returned no usable result",
        findings: [], checks: [], limitations: ["Agent or structured output unavailable"], jobs_stopped: false,
    };
    assertSchema(result, resultSchema);
    for (const [key, value] of Object.entries(identity)) {
        if (result[key] !== value) throw new Error(`Handoff ${key} does not match assigned identity`);
    }
    if (!nonempty(result.summary)) throw new Error("Handoff summary is empty");
    for (const finding of result.findings) {
        if (!nonempty(finding.summary) || !nonempty(finding.source)) throw new Error("Finding lacks source evidence");
        finding.id ??= digest(`${identity.agent}:${finding.source}:${finding.summary}`).slice(0, 16);
    }
    for (const check of result.checks) {
        if (!nonempty(check.id)) throw new Error("Check identity is empty");
        if (check.executed && (result.agent !== "algorithm-validator" ||
            !definitions[result.workflow].stages.find((stage) => stage.id === result.stage)?.validation)) {
            throw new Error("Executed checks may only originate from an assigned validation stage");
        }
        try {
            if (archive && check.result_file) {
                const file = await realpath(check.result_file);
                if (!within(args.artifact_dir, file)) throw new Error("Harness report must lie under artifact_dir");
                const raw = await json(file);
                assertSchema(raw, reportSchema, "check report");
                if (raw.check_id !== check.id || raw.candidate_fingerprint !== result.input_fingerprint) {
                    throw new Error("Harness report does not match assigned check/candidate");
                }
                Object.assign(check, {
                    executed: raw.executed, command: raw.command, exit_code: raw.exit_code,
                    criteria_met: raw.criteria_met, metrics: raw.metrics, artifacts: raw.artifacts,
                    status: raw.executed && raw.exit_code !== null && raw.exit_code !== 0
                        ? "FAIL" : raw.status ?? (raw.criteria_met ? "PASS" : "INCONCLUSIVE"),
                    evidence_origin: "harness-report",
                });
                check.artifacts.push({ path: file, sha256: await fileHash(file) });
            } else if (archive && check.kind !== "source") {
                check.evidence_origin = "manual-tool-record";
                result.limitations.push(`${check.id}: manual execution provenance; no harness report supplied`);
            }
            const artifacts = [];
            for (const artifact of check.artifacts) {
                artifacts.push(archive ? await archivedArtifact(artifact, args, directory)
                    : (await artifactPath(artifact, args), artifact));
            }
            check.artifacts = artifacts;
        } catch (error) {
            if (error.name === "AbortError") throw error;
            check.evidence_invalid = error.message;
            result.limitations.push(`${check.id}: ${error.message}`);
        }
    }
    for (const item of [...result.verifications ?? [], ...result.resolutions ?? []]) {
        if (result.agent !== "algorithm-reviewer") throw new Error("Only independent reviewers may verify/resolve evidence");
        if (!nonempty(item.reason)) throw new Error("Verification/resolution lacks rationale");
        try {
            for (const artifact of item.artifacts) await artifactPath(artifact, args);
        } catch (error) {
            if (error.name === "AbortError") throw error;
            if (Object.hasOwn(item, "status")) item.status = "INCONCLUSIVE";
            item.artifacts = [];
            result.limitations.push(`Independent evidence unavailable: ${error.message}`);
        }
    }
    return result;
}

export async function runWorkflow(name, ctx) {
    const args = structuredClone(ctx.args);
    const workspace = ctx.session.workspacePath;
    if (!workspace) throw new Error("Session workspace unavailable; durable execution cannot proceed");
    args.baseline_revision ??= (await git("rev-parse", "HEAD")).toString().trim();
    args.artifact_dir ??= path.join(workspace, "files", "workflow-artifacts");
    args.inputs ??= [];
    args.required_checks ??= [];
    args.gpu_lock ??= null;
    validateInput(name, args);
    const definition = definitions[name];
    if (!definition) throw new Error(`Unknown workflow: ${name}`);
    const sessionRoot = await realpath(workspace);
    if (!within(sessionRoot, path.resolve(args.artifact_dir))) {
        throw new Error("artifact_dir must be inside the session workspace");
    }
    await mkdir(args.artifact_dir, { recursive: true, mode: 0o700 });
    args.artifact_dir = await realpath(args.artifact_dir);
    if (!within(sessionRoot, args.artifact_dir)) throw new Error("artifact_dir must be inside the session workspace");
    if (!/^[a-zA-Z0-9_-]+$/.test(ctx.runId)) throw new Error("Invalid runtime run ID");
    const directory = path.join(args.artifact_dir, ctx.runId);
    await mkdir(directory, { recursive: true, mode: 0o700 });
    const stateFile = path.join(directory, "checkpoint.json");
    const lock = await acquireLock(path.join(sessionRoot, `workflow-${name}.lock`), { run_id: ctx.runId });
    if (!lock) {
        const report = { workflow: name, task_id: args.task_id, run_id: ctx.runId,
            status: "INCONCLUSIVE", results: [], warnings: ["Same workflow already active in this session"],
            next_action: "Observe the current run or use an independent session/worktree" };
        await save(path.join(directory, `contention-${process.pid}.json`), report);
        return report;
    }
    try {
        return await executeStages(name, ctx, args, definition, directory, stateFile);
    } catch (error) {
        if (ctx.signal?.aborted || ["AbortError", "ResponseError", "ConnectionError"].includes(error.name)) throw error;
        let history = [];
        const warnings = [error.message];
        try {
            const saved = await json(stateFile);
            history = saved.stages?.flatMap((stage) => stage.outputs) ?? [];
        } catch (readError) {
            warnings.push(`Checkpoint unavailable for reporting: ${readError.message}`);
        }
        const report = { workflow: name, task_id: args.task_id, run_id: ctx.runId,
            status: "INCONCLUSIVE", results: history, history, checkpoint: stateFile,
            blocker: error.message, warnings,
            next_action: "Reconcile partial jobs/edits and use a newly pinned run; no side effects were retried" };
        ctx.log(`Execution stopped safely: ${error.message}`);
        await save(path.join(directory, "failure.json"), report);
        return report;
    } finally {
        await lock();
    }
}

async function executeStages(name, ctx, args, definition, directory, stateFile) {
    const current = await snapshot(args);
    let state;
    try { state = await json(stateFile); } catch (error) {
        if (error.code !== "ENOENT") throw error;
        state = { version: 2, workflow: name, run_id: ctx.runId, round: 0, warnings: [],
            arguments_hash: objectHash(args), fingerprint: current,
            stages: [], in_flight: null };
        await save(stateFile, state);
    }
    if (state.version !== 2 || state.workflow !== name || state.run_id !== ctx.runId ||
        state.arguments_hash !== objectHash(args) ||
        objectHash(state.fingerprint) !== objectHash(current)) {
        throw new Error("Checkpoint inputs/source changed; reconcile side effects and start a new run");
    }
    if (state.in_flight) throw new Error(`Interrupted ${state.in_flight}; reconcile partial jobs/edits before a new run`);
    const history = () => state.stages.flatMap((item) => item.outputs);
    const active = () => state.stages.filter((item) => item.round === state.round).flatMap((item) => item.outputs);
    const final = async (complete, blocker = null) => {
        const results = active();
        const baselineResults = state.stages.filter((item) => item.round < state.round &&
            args.required_checks.some((check) => check.stage === item.id) &&
            !state.stages.some((activeStage) => activeStage.round === state.round && activeStage.id === item.id))
            .flatMap((item) => item.outputs);
        const historicalFindings = state.stages.filter((item) => item.round < state.round)
            .flatMap((item) => item.outputs.flatMap((result) => result.findings));
        const evaluation = evaluateAcceptance(name, args, [
            ...baselineResults, ...results, { agent: "history", stage: "history", status: "COMPLETE",
                checks: [], findings: historicalFindings },
        ], complete && !results.some((result) =>
            result.agent === "algorithm-reviewer" && result.next_action === "revise"), runtimeProof.has(name));
        const report = { workflow: name, task_id: args.task_id, run_id: ctx.runId,
            status: evaluation.status, evaluation, results, history: history(), revisions: state.round,
            fingerprint: state.fingerprint, checkpoint: stateFile, blocker,
            warnings: [...new Set([...state.warnings, ...evaluation.warnings,
                ...history().flatMap((result) => result.limitations)])],
            next_action: evaluation.status === "PASS" ? "completed" : "Review evidence and the reported gaps before the next run",
        };
        await save(path.join(directory, "report.json"), report);
        return report;
    };
    if (!args.environment?.length) state.warnings.push("Runtime environment identity not supplied; comparability is limited");
    if (!args.required_checks.length) state.warnings.push("No required checks declared; report cannot claim PASS");
    const selectedProfiles = [...new Set([...(definition.profiles ?? []), ...(args.profiles ?? [])])];
    const writerIndex = definition.stages.findIndex((stage) => stage.writer);
    const limit = args.max_revisions ?? 1;
    const reviewStages = new Set(["independent-review", "selection-review"]);
    for (let index = state.round > 0 && writerIndex >= 0 ? writerIndex : 0;
        index < definition.stages.length; index++) {
        const stage = definition.stages[index];
        ctx.phase(stage.id);
        const key = `${state.round}:${stage.id}`;
        let receipt = state.stages.find((item) => item.key === key);
        if (!receipt) {
            if (objectHash(await snapshot(args)) !== objectHash(state.fingerprint)) {
                throw new Error("Inputs changed between stages; no cached proof may be reused");
            }
            const before = state.fingerprint;
            const fingerprint = objectHash(before);
            const specification = await readFile(path.join(specs, name, "WORKFLOW.md"), "utf8");
            let releaseGpu = null;
            let releaseWriter = null;
            const prior = active();
            const designRejected = state.design_withheld || prior.some((result) =>
                result.next_action === "stop" ||
                (result.stage === "design-review" && (result.status !== "COMPLETE" ||
                    result.next_action === "revise" ||
                    result.findings.some((finding) => finding.unresolved && ["critical", "high"].includes(finding.severity)))));
            const validationBlocked = prior.some((result) =>
                (result.agent === "algorithm-implementer" || result.stage === "preflight") &&
                (result.status !== "COMPLETE" || result.next_action === "stop"));
            const missingGpuLock = stage.validation && args.uses_gpu === true && args.gpu_lock === null;
            if (missingGpuLock) state.warnings.push(`${stage.id}: GPU execution needs a declared shared lock`);
            const skip = (stage.writer && designRejected) || (stage.validation && validationBlocked) ||
                missingGpuLock || ((stage.writer || stage.validation) && state.jobs_unknown);
            if (stage.writer) {
                if (!skip) {
                    const lockPath = (await git("rev-parse", "--git-path", "autonomous-driving-writer.lock"))
                        .toString().trim();
                    releaseWriter = await acquireLock(path.resolve(repository, lockPath), {
                        run_id: ctx.runId, task_id: args.task_id,
                    });
                    if (!releaseWriter) {
                        state.warnings.push("Source writer busy; use a separate worktree or serialize writes");
                    }
                }
            }
            if (!skip && stage.validation && args.gpu_lock !== null) {
                releaseGpu = await acquireLock(args.gpu_lock, { run_id: ctx.runId, task_id: args.task_id });
                if (!releaseGpu) state.warnings.push(`GPU resource busy: ${args.gpu_lock}; no job was launched`);
            }
            const blocked = skip || (stage.writer && !releaseWriter) ||
                (stage.validation && args.gpu_lock !== null && !releaseGpu);
            state.in_flight = key;
            await save(stateFile, state);
            const calls = stage.agents.map((agent) => async () => {
                const identity = { workflow: name, task_id: args.task_id, stage: stage.id,
                    agent, input_fingerprint: fingerprint };
                const prompt = [
                    "Read AGENTS.md and the owning module contracts. Do not delegate.",
                    "Use only relevant existing skills; knowledge authority is ad_agents/context.",
                    `Relevant entry skills: ${definition.skills.join(", ")}.`,
                    `Domain profiles: ${JSON.stringify(selectedProfiles.map((id) => ({ id, ...profiles[id] })))}`,
                    `Specification:\n${specification}`,
                    `Assigned stage: ${stage.instruction}`,
                    stage.writer ? "You are the unique approved source writer." : "Do not modify source/configuration.",
                    agent === "algorithm-validator" && stage.validation
                        ? "Execute only declared checks/approved experiments; retain original artifacts."
                        : "Do not execute tests or experiments in this stage.",
                    `GPU lock held: ${releaseGpu !== null}. All jobs must terminate before returning.`,
                    "Return the assigned schema. COMPLETE is stage completion, never a fabricated test PASS.",
                    "Include next_action=continue/revise/stop. Reviewer revisions need actionable causal feedback.",
                    "Independent reviewers must emit verifications per check_id, original archived artifacts and candidate_fingerprint. Resolve finding IDs only with original evidence.",
                    "Metric acceptance uses declared rules. Prefer an existing harness JSON report with check.result_file following .agents/workflows/contracts/check-report.schema.json. Manual tool records remain allowed but their provenance is reported.",
                    `Round: ${state.round}; remaining approved corrections: ${limit - state.round}.`,
                    `Identity: ${JSON.stringify(identity)}`,
                    `Inputs: ${JSON.stringify(args)}`,
                    `Write experiment artifacts only under: ${directory}`,
                    `Preceding handoffs and earlier revisions: ${JSON.stringify(history())}`,
                ].join("\n\n");
                return blocked ? {
                    ...identity, status: "INCONCLUSIVE", summary: "Execution withheld; no agent/job was launched",
                    findings: [], checks: [], limitations: ["Design, implementation, resource or job state prevents this execution"],
                    jobs_stopped: true,
                } : ctx.agent(prompt, {
                    agent, schema: resultSchema,
                    label: `${ctx.runId}:${state.round}:${stage.id}:${agent}:${fingerprint}`,
                });
            });
            const raw = calls.length === 1 ? [await calls[0]()] : await ctx.parallel(calls);
            const outputs = [];
            for (let i = 0; i < raw.length; i++) {
                outputs.push(await validateResult(raw[i], { workflow: name, task_id: args.task_id,
                    stage: stage.id, agent: stage.agents[i], input_fingerprint: fingerprint }, args, directory, true));
            }
            const after = await snapshot(args);
            if (after.index !== before.index || after.outside !== before.outside ||
                after.configuration !== before.configuration ||
                after.inputs !== before.inputs || after.dependencies !== before.dependencies ||
                (!stage.writer && after.inside !== before.inside)) {
                throw new Error("Stage changed unapproved source/configuration/inputs; execution stopped");
            }
            const validator = outputs.find((result) => result.agent === "algorithm-validator");
            const jobsUnknown = !blocked && stage.validation && !validator?.jobs_stopped;
            if (jobsUnknown) {
                state.warnings.push("Launched-job termination not proven; later execution is withheld, review/report continue");
                state.jobs_unknown = true;
                if (releaseGpu) state.warnings.push(`GPU lock retained: ${args.gpu_lock}; reconcile job owner before reuse`);
            } else if (releaseGpu) {
                await releaseGpu();
            }
            receipt = { key, id: stage.id, round: state.round, fingerprint, after, outputs };
            state.fingerprint = after;
            state.stages.push(receipt);
            state.in_flight = null;
            await save(stateFile, state);
            if (releaseWriter) await releaseWriter();
        }
        if (receipt.outputs.length !== stage.agents.length) throw new Error("Incomplete checkpoint stage");
        for (const [index, result] of receipt.outputs.entries()) {
            if (result !== null) {
                await validateResult(result, { workflow: name, task_id: args.task_id,
                    stage: stage.id, agent: stage.agents[index], input_fingerprint: receipt.fingerprint }, args, directory);
            }
        }
        if (receipt.outputs.some((result) => result.status !== "COMPLETE")) {
            state.warnings.push(`${key}: incomplete/failed stage; analysis and reporting continue`);
        }
        if (stage.id === "design-review") {
            state.design_withheld = receipt.outputs.some((result) =>
                result.status !== "COMPLETE" || ["stop", "revise"].includes(result.next_action) ||
                result.findings.some((finding) => finding.unresolved && ["critical", "high"].includes(finding.severity)));
        }
        const approvedByInvocation = args.approval_mode === "invocation" &&
            stage.id === "design-review" && args.gpu_lock === null && args.uses_gpu !== true;
        const resourceApproval = stage.resource_approval && (args.uses_gpu === true || args.gpu_lock !== null);
        if ((stage.approval || resourceApproval) && !approvedByInvocation &&
            !(stage.id === "design-review" && state.design_withheld) &&
            receipt.outputs.every((result) => result.status === "COMPLETE" && result.next_action !== "stop")) {
            const checkpointHash = objectHash({ run_id: ctx.runId, args, fingerprint: receipt.after, receipt });
            const approvalFile = path.join(directory, `approval-${stage.id}.json`);
            ctx.log(`Approval required: create ${approvalFile} with ${JSON.stringify({
                run_id: ctx.runId, task_id: args.task_id, checkpoint_hash: checkpointHash, approved: true,
            })}. Review the checkpoint first; runtime permissions remain separate.`);
            await ctx.pause(`approval-v1:${stage.id}:${checkpointHash}`);
            let approval;
            try { approval = await json(approvalFile); } catch (error) {
                if (error.code !== "ENOENT") throw error;
                return final(false, `Missing operator approval: ${approvalFile}`);
            }
            if (approval.approved !== true || approval.run_id !== ctx.runId ||
                approval.task_id !== args.task_id || approval.checkpoint_hash !== checkpointHash) {
                return final(false, `Approval does not match reviewed checkpoint: ${approvalFile}`);
            }
        }
        if (reviewStages.has(stage.id) && receipt.outputs.some((result) =>
            result.agent === "algorithm-reviewer" && result.next_action === "revise")) {
            if (writerIndex >= 0 && state.round < limit && !state.jobs_unknown && !state.design_withheld) {
                state.round++;
                await save(stateFile, state);
                ctx.log(`Reviewer-directed correction ${state.round}/${limit}; rerun writer, validation, and independent review`);
                index = writerIndex - 1;
            } else {
                state.warnings.push("Reviewer requested revision; budget/scope/job state does not permit more automatic writes");
            }
        }
    }
    if (objectHash(await snapshot(args)) !== objectHash(state.fingerprint)) {
        throw new Error("Inputs changed before final acceptance");
    }
    for (const result of active()) {
        await validateResult(result, { workflow: name, task_id: args.task_id }, args, directory);
    }
    if (state.jobs_unknown) return final(false, "Reconcile launched jobs; evidence/report retained");
    await save(stateFile, state);
    return final(true);
}
