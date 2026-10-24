# NVIDIA CUDA-PointPillars

- Repository: https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars
- Revision: `092affc36c72d7b8f7530685d4c0f538d987a94b`
- Parent Lidar_AI_Solution revision: `7c1623f234d5a9794fd862c9456f68f87ae7b900`
- License: Apache-2.0; retained LICENSE, NOTICE and per-file notices.

Only CUDA preprocessing, scatter plugin, decoding and NMS sources are imported.
Apollo owns engine loading/building and resource management; the upstream
TensorRT 8 runner, training, data and model files are not copied.

The supplied ONNX has a 10000-pillar input while upstream params.h permits
40000. The adapter validates the actual engine shape and rejects input beyond
the engine's capacity before execution.

Local changes: bound pillar writes at MAX_VOXELS (the adapter reports overflow);
use the atomic object counter instead of the racy output count; correct plugin
serialization size and reject invalid serialized dimensions. The fixed model
is KITTI XYZI, single sweep, with the constants in params.h. GPU voxel point
selection remains nondeterministic as documented upstream.
