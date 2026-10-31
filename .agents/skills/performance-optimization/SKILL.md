---
name: performance-optimization
description: Measure before optimizing; collect profiles, identify the dominant bottleneck, and verify controlled improvements. Use for performance analysis, hotspot profiling, latency, CPU/GPU, memory growth, throughput, lock contention, and scheduler waits.
---

# Evidence-Driven Performance Optimization

## Scope and prerequisites

Use for resource-consumption or real-time-budget issues. Analyze, measure, and
attribute the bottleneck before optimizing; do not guess hotspots from code
appearance.
If the primary symptom is a functional error, start with
`issue-localization`.
Read the module contract, existing performance tools, and test entry points;
see the [skill index](../README.md) for authoritative command references.
Use approved environments and permissions. Do not automatically install a
profiler or change shared-machine runtime settings.

## Workflow

### 1. Define metrics and freeze conditions

Define metrics, measurement intervals, the current problem, and target
budgets, based on user-visible behavior or contractual requirements.
Distinguish end-to-end latency, per-stage time, CPU time, throughput, memory
peak, and long-term growth. Select mean, tail latency, deadline misses, or
frame-drop rate as appropriate; they are not interchangeable.

Fix code and dependency versions, build mode, hardware, container, data,
configuration, concurrency, and replay rate.
Record warm-up, cache state, input volume, other loads, and measurement
overhead. If a shared environment cannot be isolated, report the limitation.
Do not stop another person's process or automatically change CPU frequency,
affinity, or GPU power settings.

### 2. Measure a baseline, then localize the bottleneck layer by layer

Use low-cost counters and per-stage timing to locate the area first, then
select call-stack sampling, tracing, memory, lock, or GPU tools based on
evidence. Do not enable every heavyweight profiler before understanding the
problem.
Save the profiling scope, tool parameters, and raw results; do not repeat
collection when existing evidence is sufficient.

Distinguish bottleneck types:

- Compute: CPU/GPU core work, repeated computation, or growth in problem size.
- Data: Copying, allocation, serialization, I/O, cache, or memory bandwidth.
- Waiting: Locks, queues, scheduling, inter-process communication, and
  downstream backpressure.
- Memory: Peak, retention, fragmentation, and continued growth; an increase
  in RSS does not by itself mean a leak.

Explain hotspots using call counts, input size, inclusive/self time, and the
critical path.
A CPU hotspot does not necessarily cause wall-clock latency. Asynchronous GPU
submission time does not mean the kernel has completed; measure at the
appropriate completion boundary and state how synchronization perturbs
production execution.
Check queue waits, upstream supply, and consumer rate limits so that waiting
is not misdiagnosed as slow computation.

### 3. Form an optimization hypothesis

State the chain “evidence → dominant bottleneck → cause → minimal change →
expected observable change.”
Prioritize the modifiable cause that contributes most to the target metric,
estimate the upper bound of its overall benefit, and avoid optimizing
expensive functions that are not on the critical path.
Test one primary causal hypothesis at a time; do not combine algorithm,
thread-count, precision, and cache-policy changes in one experiment.

Before implementation, check correctness, thread safety, data freshness,
memory, and worst-case latency constraints.
Reducing precision, scenario coverage, or data; increasing caches; or changing
behavior is not a free optimization. Make the trade-off explicit and obtain
necessary confirmation.

### 4. Validate a comparison under equivalent conditions

Compare baseline and change using the same input, build mode, metric
definition, and timing boundaries.
Warm up for steady-state metrics. Keep separate cold-start measurements for
cold-start issues; do not use warm-up to hide the cost.
Repeat samples based on variability and required decision precision. When
useful, interleave baseline and candidate runs to reduce environmental drift.
Preserve every sample, sample count, and distribution; do not present a
microbenchmark's repeated mean as per-frame P99.

Report absolute values, relative changes, variability, and whether
pre-agreed thresholds were met.
If the difference falls within noise, state that no reliable improvement has
been demonstrated; do not cherry-pick the best run.
Microbenchmarks validate local mechanisms; end-to-end performance claims
still require evidence under the relevant workload.
Use a comparison without heavyweight profiling to confirm that an apparent
gain is not merely a profiler artifact.

Run correctness regressions covering the change. Confirm that cost was not
shifted to other threads, the GPU, memory, or downstream components and that
algorithm quality and tail latency were not sacrificed. Stop once evidence
is sufficient.

## Deliverables and acceptance

Report workload and environment, metrics and budgets, baseline, profiling
evidence, bottleneck attribution, changes, comparison samples, correctness
results, and remaining limitations. Performance conclusions must be
traceable to raw results.
If only analysis was completed, report the confirmed bottleneck and next
discriminating experiment; do not claim optimization.

## Failure handling

If a tool is missing, permissions are insufficient, or profiling overhead is
too high, stop that collection and use existing low-cost evidence.
If the alternative cannot support a conclusion, state the blocker. Ask before
installing tools or changing the environment.
If no hotspot is found, continue narrowing the scope rather than optimizing
randomly. Recheck the hypothesis when gains are unstable or functional
regressions occur. Revert only experimental changes that are clearly
separable and yours; preserve others' work.

## Basis

Reuse the performance-layering approach from existing context; comparative
measurement is informed by [Google Benchmark](../README.md).
This workflow includes its own validation steps and does not depend on the
currently missing `performance-testing.md` or require a new benchmark library.
