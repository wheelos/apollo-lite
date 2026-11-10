---
name: model-artifact-integration
description: Integrate or replace inference models and engines with verified tensor, preprocessing, runtime, and artifact contracts. Use for ONNX, TensorRT, model assets, precision changes, and target-device deployment.
---

# Model Artifact Integration

## Procedure

1. Read the owning evaluator/adapter and existing model tests. Identify the
   model's legitimate source, license, checksum, revision, and target runtime.
   Do not fetch or distribute artifacts without authorization.
2. Specify tensor names, shapes, dynamic dimensions, layout, dtype, units,
   coordinate frames, normalization, preprocessing, and output decoding.
   Compare training/export semantics with the actual inference implementation.
3. Record runtime/compiler, device, precision, optimization profiles, plugins,
   calibration inputs, and engine build parameters. Do not assume a serialized
   engine is portable across device or runtime versions.
4. Compare a small reference fixture against independently verified outputs.
   Check invalid shapes, non-finite results, empty inputs, and failure signaling.
5. Validate the real target runtime for deployment claims. Use
   `performance-optimization` for measured resource claims and
   `simulation-regression` for downstream behavioral claims.

## Acceptance and failure handling

Separate contract correctness, model quality, target compatibility, and latency
evidence. Synthetic plumbing, CPU inference, or engine construction alone does
not prove target GPU behavior. Missing device, calibration data, or model rights
is a stated boundary, not permission to change precision or substitute assets.
