---
name: engineering-solution-design
description: Design engineering solutions through industry practices, open-source research, reuse-versus-build comparison, and architecture decisions. Use for technology selection, solution comparison, open-source adoption, engineering patterns, architecture, and minimal framework design.
---

# Engineering Solution Selection and Design

## Scope

Use for designing new capabilities, technology selection, architecture
evolution, and adopting engineering patterns from open-source projects.
The goal is a minimal, correct solution that is evidence-based, trade-off
aware, and implementable in stages—not a collection of technology buzzwords.
The default deliverable is a design; modify code or build a framework only
when implementation is explicitly part of the task.

## Task routing

This skill addresses what to select, why, and where to draw boundaries; it
does not take over algorithm implementation or fault investigation.
Hand off algorithm implementation to `algorithm-development`, unknown
anomalies to `issue-localization`, resource bottlenecks to
`performance-optimization`, and scenario validation to `simulation-regression`.
See the [skill index](../README.md) for entry points and method references.
Hand off only when needed for the current task.
When an existing solution meets the needs of a clearly scoped small change,
reuse it and record only new constraints and necessary trade-offs; do not
require a new candidate matrix, document, or approval process.

## Prerequisites

- Follow the repository's agent rules and read relevant source, tests, and
  necessary context for the target module.
- Confirm existing capabilities, module ownership, dependency boundaries, and
  worktree changes; prefer reusing existing repository mechanisms.
- Clarify goals, non-goals, use cases, delivery stages, and non-negotiable
  constraints.
- Ask first when missing information could change the selection, system
  boundary, or key behavior; explicitly label other assumptions.
- Use only public, non-sensitive questions for external research. Do not
  upload internal code, logs, credentials, or business data.

## Core principles

- **Look for existing answers first.** Prefer verified implementations,
  interfaces, and engineering patterns for mature, general problems. Do not
  reinvent them to demonstrate design skills, and do not copy an entire
  architecture just because a project is well known.
- **Adopt mechanisms, not scale.** Distinguish direct dependencies, protocol
  compatibility, local adaptation, conceptual borrowing, forks, and
  in-house development. Each has different licensing, maintenance, and upgrade
  costs.
- **For unknown problems, establish principles first.** For niche, unresolved,
  or highly customized problems, define invariants and trade-offs before
  deciding system boundaries, module responsibilities, and key contracts;
  address algorithm and implementation details last.
- **Prefer a working framework, not an empty shell.** Some local details may
  be deferred, but do not skip feasibility risks that affect the architecture
  or substitute empty methods, swallowed errors, or false success for a
  working minimal path.
- **Match depth to risk.** Use brief comparisons for simple, reversible
  decisions and stronger evidence for costly, irreversible, or high-risk
  decisions. Stop researching once the decision is supported; do not aim to
  exhaustively list projects.

## Workflow

### 1. Define the problem and selection criteria

Briefly state business goals, the current baseline, inputs and outputs,
non-goals, and success criteria.
Separate hard constraints from tradeable requirements. Performance
requirements should include the scenario, scale, and evaluation method; do not
invent metrics.

First classify the problem, assessing modules separately when necessary:

| Problem type | Default path |
| --- | --- |
| Mature, general problem | Find established patterns and implementations; compare reuse options and satisfy constraints with minimal adaptation |
| Mature core with a customized boundary | Reuse the general core, isolate customized policies, and define interface and upgrade boundaries |
| Niche, unresolved, or highly customized problem | Conduct focused research, establish principles, design a framework, and validate key assumptions with risk experiments |

“Not found yet” does not mean “does not exist in the industry.” State the
research scope and missing evidence; do not claim that no solution exists.

### 2. Research established practices and open-source implementations

Gather evidence from official documentation, original papers or standards,
project source, tests, release records, and maintainer discussions.
Use blogs and rankings only to discover leads; stars, badges, and vendor claims
alone do not demonstrate suitability.
Start with a small set of candidates and usually compare two or three
representative options in depth. State when evidence is insufficient; do not
invent candidates to fill a quota. Include the existing implementation or
adding no dependency as a baseline.

For candidates that affect the conclusion, inspect actual critical paths and
representative tests, not only the README.
Record source links, access date, and version or commit. For source evidence,
use version-pinned paths and symbols, adding reliable line numbers when
available. Distinguish:

- **Verified facts:** What the source actually demonstrates; source code being
  present does not mean it has been run and validated.
- **Applicability assessment:** Which scenarios match this project and which
  assumptions differ.
- **Unverified assumptions:** Capabilities, costs, or limitations that still
  require an experiment or user confirmation.

For each reference project, state which mechanism is borrowed, what problem it
solves, what is not adopted, and what adaptation is required.
Compare total costs, including integration, deployment, debugging,
long-term maintenance, upstream upgrades, fork patches, and replacement.
Check licenses, transitive dependencies, and distribution obligations. Do not
assume code may be copied when its license is unclear.
Do not generalize findings about one version into claims that apply to the
project forever.

### 3. Compare options and make a selection

Filter against hard constraints first, then weigh trade-offs. Eliminate
options that fail a hard constraint. If a hard constraint is unknown, treat
the option only as conditional; other advantages cannot “average out” the
uncertainty.

Use dimensions relevant to the task; avoid an unnecessarily large comparison
table:

| Candidate and reuse approach | Evidence and applicable scenarios | Hard constraints | Engineering and maintenance costs | Risks/unknowns | Conclusion |
| --- | --- | --- | --- | --- | --- |
| Existing baseline | Actual behavior and shortcomings | Met/not met/unknown | Cost of keeping the status quo | Missing evidence | Retain/reject and why |
| Candidate | Source, version, and key mechanism | Met/not met/unknown | Integration, upgrade, and exit costs | Validation method | Recommend/alternate/reject and why |

Compare direct reuse, thin adaptation, and in-house development when relevant,
but do not require every task to include all approaches.
Use weighted scores only when weights and scoring criteria are clear; do not
use fabricated precision to disguise judgment.

State the recommended option, key reasons for rejecting alternatives,
accepted disadvantages, and conditions that would trigger reassessment.
When evidence is insufficient, provide a conditional recommendation and
minimal validation plan; do not present selection as complete.
Obtain necessary confirmation before implementing significant behavioral
trade-offs or changes to dependencies, environments, or deployment.

### 4. Establish principles, then refine the architecture in layers

For customized parts, first write a small number of verifiable principles.
Each principle should include an invariant, rationale, design impact, and
verification method. Principles must constrain the design; avoid vague
slogans such as “high performance” or “high extensibility.”
For mature solutions, also state whether they are compatible with these
principles.

Develop the design in the following order, going only as deep as the current
decision requires:

1. **System boundaries:** Users, external systems, trust boundaries,
   inputs and outputs, and deployment constraints.
2. **High-level structure:** Main data and control flows, state ownership,
   dependency direction, and failure paths.
3. **Modules:** Single responsibilities, interface contracts, state
   ownership, reuse sources, and customization boundaries.
4. **Key details:** Expand only details that affect correctness, feasibility,
   performance thresholds, or compatibility.

At minimum, contracts must specify input/output semantics and error behavior;
add timing, units, coordinate frames, concurrency, and lifecycle details as
needed. Do not directly import another project's deployment scale or
dependency ecosystem into this project.
A simple diagram and a necessary module table are sufficient; do not require
filling out an entire architecture-document template.

Classify unresolved details as “must validate first” or “can defer,” and
state the reason for deferral, constraint boundary, and revisit conditions.
Do not label a risk that determines whether the solution is viable as an
unrelated detail.
For autonomous-driving modules, specify coordinate-transform direction,
time basis, data freshness, reset and relocalization behavior, real-time
budgets, and failure/degradation contracts as applicable; do not assume every
module needs every concept.

### 5. Plan a minimal end-to-end path and evolution

First define a minimal vertical path from input to an observable result rather
than building every module at once.
Typical stages are a critical-risk experiment, a minimal end-to-end path, and
capability additions driven by need; do not mechanically require a fixed
number of stages.
For each stage, state deliverables, dependencies, acceptance criteria, and
stop or rollback conditions.

If the user requests only framework design, deliver module boundaries, key
contracts, and implementation order.
If implementation is authorized, prioritize completing this path and reuse
existing engineering and test mechanisms:

- Report unimplemented capabilities explicitly; do not return fabricated
  success results.
- State the scope of mocks, stubs, and diagnostic inputs; they are not
  evidence of production correctness.
- Use representative loads and a baseline to validate specific risk
  hypotheses; do not report a plan as a measurement.
- Run only the minimum validation needed to cover the change, using
  repository-verified commands and environments.
- Identify which known changes motivate extension points; do not prebuild a
  generic plugin platform or unnecessary abstraction layers.
- Limit handoff information to the selected solution, constraints, key
  contracts, unresolved risks, and acceptance criteria. Do not require the
  next stage to repeat research that is complete and still valid.

## Deliverable format

Scale the following content to the problem; a few paragraphs are sufficient
for small problems, and creating a document is not mandatory:

1. **Conclusion:** Recommended approach, applicable assumptions, and selection status (proposed, pending validation, or confirmed).
2. **Constraints and principles:** Goals, non-goals, hard gates, and design invariants.
3. **Evidence and comparison:** Sources, candidates, scope of reuse, trade-offs, and reasons for rejection.
4. **Framework and modules:** Critical path, ownership, interfaces, failure behavior, and deferred details.
5. **Implementation path:** Minimal end-to-end path, acceptance criteria, risk experiments, exit conditions, and reassessment conditions.

For significant decisions, record context, drivers, options, decisions,
consequences, and confirmation in an ADR-like format; do not create a separate
ADR for every local implementation choice. Persist knowledge only when
requested by the user, following the maintenance rules for
`ad_agents/context/` and updating its index. Do not establish a competing
knowledge authority.

## Acceptance

- The recommendation has traceable evidence and identifies what does and does
  not match this project.
- The comparison includes a viable baseline; rejected options have reasons;
  unknowns are not presented as facts.
- A customized solution defines principles, boundaries, state ownership, key
  contracts, and a minimal end-to-end path.
- Deferred details do not undermine the current architecture; key risks have
  validation and revisit conditions.
- A design deliverable does not claim implementation was validated; an
  implementation deliverable does not treat a shell as complete.
- Only authorized code, dependencies, and persistent documents are changed.

## Failure handling

If an external source is unavailable, use accessible public primary sources or
existing versioned evidence; do not bypass access restrictions or fabricate
citations. If key evidence is still missing, lower confidence and state the
gap.
Stop the affected work when constraints conflict, a key experiment fails, or
a significant trade-off remains. Explain the reason and options, then ask the
user to decide; do not silently relax gates, introduce dependencies, or expand
scope.

## Method references

The following references provide methods, not fixed technology selections
for all projects. Links are pinned to the reviewed versions.
This workflow synthesizes them for the task; it does not require copying any
template in full.

- [MADR decision template](https://github.com/adr/madr/blob/ba75bb1b20d42af5746b246ad348c202419ae681/template/adr-template.md):
  context, drivers, candidates, consequences, and confirmation for traceable
  selection records.
- [arc42 architecture template](https://github.com/arc42/arc42-template/blob/9057fe1f32ce543e546f76971515c537f0dd3f63/EN/arc42-template.adoc):
  goals, constraints, context, strategy, building blocks, runtime views,
  quality, and risks for organizing a layered design.
- [OpenSSF Best Practices](https://github.com/ossf/best-practices-badge/blob/2ea9de20f655c91a200bf3d9fe377455cc404f68/README.md)：
  licensing, release, testing, and maintenance practices for open-source
  projects; a self-certified badge is only a lead, not a substitute for
  validating suitability.

Sources last reviewed: 2026-10-26.
