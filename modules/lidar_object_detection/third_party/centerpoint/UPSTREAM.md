# CenterPoint upstream provenance

- Upstream: [NVIDIA-AI-IOT/Lidar_AI_Solution, CUDA-CenterPoint](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-CenterPoint)
- Revision: `7c1623f234d5a9794fd862c9456f68f87ae7b900`
- License: MIT; see [`LICENSE.md`](LICENSE.md) and the retained per-file SPDX notices.

This directory contains the upstream CenterPoint implementation and its CUDA
kernels, with compatibility adaptations for the repository's CUDA/TensorRT
toolchain. It is not a pristine source export. Keep Apollo's C ABI wrapper and
resource lifecycle code under `modules/lidar_object_detection/inference/`;
keep shared serialized-model import and schemas under
`pkgs/common/perception/model_io/onnx` and the reusable spconv library build
in the registry module `spconv@2.3.8.bcr.3`.

The sparse ONNX parser and its tests were extracted into
`wheelos_common//perception/model_io/onnx`; they are not NVIDIA upstream files.
The model-neutral sparse graph API and CUDA executor live in
`pkgs/common/perception/ops/spconv`. `CenterPoint` maps the parsed graph into that
public API, whose sole backend compiles pinned spconv 2.3.8/cumm generated
sources as described in [`SPCONV.md`](SPCONV.md). The NVIDIA sparse-engine
wrapper, imported DSO, headers and parser are no longer used. NVIDIA
provenance still applies to the retained voxelizer, TensorRT integration and
postprocessing.

Before updating the pinned revision, compare the proposed source against the
upstream revision, review and document compatibility changes, preserve upstream
notices, then run the CenterPoint GPU build and focused module tests. TensorRT
engines and model files are separate deployment artifacts and are not part of
this source snapshot.
