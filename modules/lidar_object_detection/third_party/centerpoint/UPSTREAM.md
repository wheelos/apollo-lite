# CenterPoint upstream provenance

- Upstream: [NVIDIA-AI-IOT/Lidar_AI_Solution, CUDA-CenterPoint](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-CenterPoint)
- Revision: `7c1623f234d5a9794fd862c9456f68f87ae7b900`
- License: MIT; see [`LICENSE.md`](LICENSE.md) and the retained per-file SPDX notices.

This directory contains the upstream CenterPoint implementation and its CUDA
kernels, with compatibility adaptations for the repository's CUDA/TensorRT
toolchain. It is not a pristine source export. Keep Apollo's C ABI wrapper and
resource lifecycle code under `modules/lidar_object_detection/inference/`;
keep the reusable spconv headers, ONNX parser, and schemas under the single
repository-level `third_party/spconv/` copy.

Before updating the pinned revision, compare the proposed source against the
upstream revision, review and document compatibility changes, preserve upstream
notices, then run the CenterPoint GPU build and focused module tests. TensorRT
engines and model files are separate deployment artifacts and are not part of
this source snapshot.
