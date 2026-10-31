const nonempty = (value) => typeof value === "string" && value.trim().length > 0;
const compare = {
    lt: (a, b) => a < b, le: (a, b) => a <= b, eq: (a, b) => a === b,
    ge: (a, b) => a >= b, gt: (a, b) => a > b,
};

export function checkProven(check) {
    return check.status === "PASS" && check.criteria_met && check.artifacts.length > 0 &&
        (check.kind === "source" || (check.executed && nonempty(check.command) && check.exit_code === 0 &&
            (check.kind !== "metric" || check.metrics.length > 0)));
}

export function evaluateCheck(required, result, check) {
    const detail = { id: required.id, requirement_id: required.requirement_id ?? required.id,
        mandatory: required.mandatory !== false, status: "INCONCLUSIVE",
        candidate_fingerprint: result?.input_fingerprint ?? null, reason: "No matching evidence" };
    if (!check) return detail;
    if (check.evidence_invalid) return { ...detail, reason: check.evidence_invalid };
    if (check.status === "FAIL" || (check.executed && check.exit_code !== null && check.exit_code !== 0)) {
        return { ...detail, status: "FAIL", reason: "Check execution failed" };
    }
    if (!checkProven(check)) return { ...detail, reason: "Execution, assertions, or artifacts are incomplete" };
    if (required.kind === "metric" && !required.rules?.length) {
        return { ...detail, reason: "Metric acceptance has no declared deterministic rules" };
    }
    for (const rule of required.rules ?? []) {
        const metrics = check.metrics.filter((metric) => metric.name === rule.metric && metric.unit === rule.unit);
        if (metrics.length !== 1) return { ...detail, reason: `Missing or ambiguous metric ${rule.metric} (${rule.unit})` };
        if (!compare[rule.operator](metrics[0].value, rule.limit)) {
            return { ...detail, status: "FAIL", reason: `${rule.metric}=${metrics[0].value} violates ${rule.operator} ${rule.limit} ${rule.unit}` };
        }
    }
    return { ...detail, status: "PASS", reason: "Declared checks and metric rules satisfied" };
}

export function evaluateAcceptance(name, args, results, complete, needsRuntime = false) {
    const evaluations = [];
    const reviewers = results.filter((result) => result.agent === "algorithm-reviewer" &&
        ["independent-review", "decision-review", "selection-review"].includes(result.stage));
    for (const required of args.required_checks) {
        let selected;
        for (const result of results) {
            if (result.agent === "experiment-reporter") continue;
            if (required.stage && result.stage !== required.stage) continue;
            if (required.kind !== "source" && result.agent !== "algorithm-validator") continue;
            const matches = result.checks.filter((check) => check.id === required.id && check.kind === required.kind);
            if (matches.length > 1) {
                selected = { result, check: { ...matches[0], evidence_invalid: "Duplicate check results" } };
            } else if (matches.length === 1) selected = { result, check: matches[0] };
        }
        let detail = evaluateCheck(required, selected?.result, selected?.check);
        if (detail.status === "PASS") {
            const verification = reviewers.flatMap((reviewer) => reviewer.verifications ?? [])
                .filter((item) => item.check_id === required.id &&
                    item.candidate_fingerprint === selected.result.input_fingerprint).at(-1);
            const original = selected.check.artifacts;
            const verified = verification?.artifacts ?? [];
            if (verification?.status === "REJECTED") {
                detail = { ...detail, status: "FAIL", reason: verification.reason };
            } else if (verification?.status !== "VERIFIED" || !nonempty(verification.reason) ||
                !original.every((artifact) => verified.some((item) =>
                    item.path === artifact.path && item.sha256 === artifact.sha256))) {
                detail = { ...detail, status: "INCONCLUSIVE", reason: "Independent verification does not cover candidate and original artifacts" };
            }
        }
        evaluations.push(detail);
    }
    const resolutions = reviewers.flatMap((reviewer) => (reviewer.resolutions ?? [])
        .filter((item) => item.candidate_fingerprint === reviewer.input_fingerprint &&
            nonempty(item.reason) && item.artifacts.length > 0));
    const findings = results.flatMap((result) => result.findings).filter((finding) =>
        finding.unresolved && !resolutions.some((item) => item.finding_id === finding.id));
    const mandatory = evaluations.filter((item) => item.mandatory);
    const failed = mandatory.some((item) => item.status === "FAIL") ||
        findings.some((item) => ["critical", "high"].includes(item.severity)) ||
        results.some((result) => result.status === "FAIL");
    const runtimeCheck = evaluations.some((item) => item.status === "PASS" &&
        args.required_checks.some((check) => check.id === item.id && check.kind !== "source"));
    const missing = !complete || !mandatory.length || mandatory.some((item) => item.status !== "PASS") ||
        results.some((result) => result.status === "INCONCLUSIVE") || (needsRuntime && !runtimeCheck);
    return { status: failed ? "FAIL" : missing ? "INCONCLUSIVE" : "PASS", checks: evaluations,
        unresolved_findings: findings,
        warnings: evaluations.filter((item) => item.status !== "PASS").map((item) => `${item.id}: ${item.reason}`) };
}
