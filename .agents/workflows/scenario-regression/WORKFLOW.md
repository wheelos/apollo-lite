---
name: scenario-regression
description: Autonomous-driving trigger and behavioral assertions through the smallest existing suitable harness.
---

# Autonomous-driving scenario regression

## Trigger

Explicit replay or closed-loop scenario validation. No road/HIL execution.

## Inputs

Use the [shared input contract](../contracts/workflow-input.schema.json).
Bind requirements, ODD/scenario, map/vehicle/model/fixture identity, recomputed
modules, random seed and time basis. Define trigger and behavioral assertions
before execution. Existing sim-agent exact-plan approval remains mandatory.

## Stages and agents

| Stage | Agent | Output |
| --- | --- | --- |
| preflight | algorithm-researcher | Execution model, scenario/trigger contract and approval |
| validation | algorithm-validator | Existing harness/evaluator and original artifacts |
| independent-review | algorithm-reviewer | Trigger, first divergence and coverage audit |
| report | experiment-reporter | Behavior, acceptance and uncovered ODD risks |

## Handoffs and recovery

Follow [shared lifecycle](../README.md) and scenario profile. Use existing
scenario assets and executors. Analysis can finish with missing fixtures, but no
behavioral PASS is inferred. Select the smallest model supporting the conclusion.

## Acceptance

Mandatory triggers/assertions and independent verification must pass. A
non-triggering scenario is INCONCLUSIVE even if the process exits zero.
Open-loop evidence does not prove changed closed-loop outcomes; simulation
does not establish road safety or standards conformity.
