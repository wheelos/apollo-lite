# LiDAR semantic segmentation

This module is the neural LiDAR semantic segmentation entrypoint. It follows the
standalone module shape used by `modules/lane`:

- `component/`: Cyber component boundary.
- `conf/`: component runtime configuration.
- `dag/`: launch DAG.
- `inference/`: RangeRet model contract, preprocessing, and decoding.
- `proto/`: component config and output messages.
- `types/`: C++ data structures shared inside the module.

## Model boundary

RangeRet lives outside this repository at `/home/humble/01code/RangeRet` and is
treated as the training/export source. Runtime code should not import Python from
that repository. Export the model to a stable inference artifact
(`.engine`, TorchScript, or ONNX) and configure that artifact through
`LidarSemanticSegmentationComponentConfig.engine_path`.

The Apollo-side contract mirrors RangeRet's dataset parser:

1. Convert `apollo.drivers.PointCloud` to a 5-channel range image:
   `range, x, y, z, intensity`.
2. Normalize each channel with configured means/stds.
3. Run RangeRet and decode logits shaped as `[1, H, W, num_classes]`.
4. Re-project labels to the original point order and publish
   `LidarSemanticSegmentationResult`.

## Data model recommendation

Keep semantic labels separate from raw point clouds and obstacle objects. The
output message carries one `PointSemanticLabel` per source point, preserving the
original point order and the projected range-image coordinate. Downstream modules
can join by `point_index` without mutating driver messages or overloading legacy
`LidarFrame` obstacle fields.
