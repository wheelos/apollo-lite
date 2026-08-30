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

RangeRet lives outside this repository and is treated as the training/export source. The checked-in component is a TensorRT runtime and therefore requires a serialized `.engine`; a PyTorch `.pt` file or ONNX file cannot be configured directly.

The Apollo-side contract mirrors RangeRet's dataset parser:

1. Convert `apollo.drivers.PointCloud` to a 5-channel range image:
   `range, x, y, z, intensity`.
2. Scale Apollo's integer intensity into the checkpoint's remission domain,
   then normalize each channel with configured means/stds.
3. Run RangeRet and decode logits shaped as `[1, H, W, num_classes]`.
4. Re-project labels to the original point order and publish
   `LidarSemanticSegmentationResult`.

The SemanticKITTI config uses `intensity_scale = 1 / 255`, because Apollo
Velodyne points carry 8-bit intensity while the RangeRet checkpoint was trained
with remission values in `[0, 1]`.

## Prepare the TensorRT engine

Export the available checkpoint from the RangeRet environment:

```sh
python3 wheelos-service/context/framework/skills/rangeret-engine-conversion/export_rangeret_onnx.py \
  --source-dir <path-to-RangeRet-repo> \
  --checkpoint <path-to-models>/rangeret-kitti-657.pt \
  --config <path-to-RangeRet-repo>/config/RangeRet-semantickitti.yaml \
  --output <path-to-models>/rangeret-kitti-657.onnx

python3 wheelos-service/context/framework/skills/rangeret-engine-conversion/build_rangeret_engine.py \
  --onnx=<path-to-models>/rangeret-kitti-657.onnx \
  --engine=<path-to-models>/rangeret-kitti-657.engine

```

TensorRT engines are GPU architecture and TensorRT-version specific. Generate
the engine in the same container/runtime family that will execute the
component. The default config points to the engine path above.

## Build and replay

```sh
bazel build //modules/lidar_semantic_segmentation:liblidar_semantic_segmentation_component_plugin.so
mainboard -d modules/lidar_semantic_segmentation/dag/rangeret_semantic_kitti.dag
cyber_recorder play -f <path-to-records>/demo_3.5.record

```

The demo record publishes
`/apollo/sensor/lidar128/compensator/PointCloud2`, which is the reader channel
in the checked-in DAG and config. The checkpoint is trained on SemanticKITTI
HDL-64 data, so Apollo lidar128 replay proves integration but not production
segmentation quality. A sensor-matched fine-tuned checkpoint and labeled
benchmark remain necessary for quality acceptance.

## Data model recommendation

Keep semantic labels separate from raw point clouds and obstacle objects. The
output message carries one `PointSemanticLabel` per source point, preserving the
original point order and the projected range-image coordinate. Downstream modules
can join by `point_index` without mutating driver messages or overloading legacy
`LidarFrame` obstacle fields.
