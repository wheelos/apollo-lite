# Upstream source

This directory contains the inference-only core from NVIDIA
`CUDA-BEVFusion` at `7c1623f234d5a9794fd862c9456f68f87ae7b900`.
The upstream project is distributed under MIT; see `LICENSE.md`.

The retained files are the BEVFusion camera normalization/geometry/backbone,
LiDAR voxelization and sparse backbone, BEV pooling, feature fusion, bounding
box head, and their CUDA/TensorRT helpers. The standalone demo, image
visualization, Python bindings, QAT/PTQ, training and export tools, cuOSD,
STB, copied spconv parser, and duplicate ONNX protobuf schemas are not included.
The sparse ONNX graph executes through the shared CenterPoint sparse-backbone
adapter and the Bzlmod `@spconv//:spconv` target; BEVFusion does not carry a
second sparse-convolution implementation or NVIDIA's former parser library.

The first model contract is the upstream ResNet50 FP16 layout: six RGB camera
images at 1600x900, resized by the upstream normalizer to 704x256, and packed
LiDAR rows with five FP16 model features at runtime. The runner accepts packed
float32 LiDAR rows and converts them to FP16 after strict input validation.
No model weights, ONNX files, or TensorRT engines are included.
