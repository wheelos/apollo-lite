# Camera semantic segmentation

This module is the neural camera image semantic segmentation entrypoint using SegFormer. It follows the standalone module shape used by `modules/lidar_semantic_segmentation` and `modules/lane`:

- `component/`: Cyber component boundary (`CameraSemanticSegmentationComponent`).
- `conf/`: component runtime configuration (`segformer_cityscapes.pb.txt`).
- `dag/`: launch DAG (`segformer_cityscapes.dag`).
- `inference/`: SegFormer model contract, image preprocessing, decoding, and TensorRT executor.
- `proto/`: component config and output messages.
- `types/`: C++ image frame structures and semantic types.
- `tools/`: ONNX export and TensorRT engine conversion scripts for SegFormer.

## Model boundary

SegFormer (e.g. `/home/humble/01code/SegFormer`) is treated as the training and export source. The checked-in component is a TensorRT runtime and therefore requires a serialized `.engine`.

The Apollo-side pipeline:

1. Subscribes to `apollo.drivers.Image` (RGB/BGR encoding).
2. Resizes and normalizes the image to SegFormer's input resolution (e.g., $512 \times 1024$ or $1024 \times 1024$) with standard ImageNet/Cityscapes mean `[123.675, 116.28, 103.53]` and std `[58.395, 57.12, 57.375]`.
3. Runs SegFormer TensorRT engine and decodes logits shaped as `[1, num_classes, H, W]` (NCHW) or `[1, H, W, num_classes]` (NHWC).
4. Decodes argmax class labels (e.g. 19 Cityscapes classes: road, sidewalk, building, vehicle, person, etc.) and softmax confidences.
5. Publishes `CameraSemanticSegmentationResult` carrying the compact mask bytes and per-pixel labels/confidences.

## Prepare the TensorRT engine

Export the model checkpoint from the SegFormer environment:

```sh
python3 modules/camera_semantic_segmentation/tools/export_segformer_onnx.py \
  --source-dir /home/humble/01code/SegFormer \
  --config /home/humble/01code/SegFormer/local_configs/segformer/B0/segformer.b0.512x1024.city.160k.py \
  --checkpoint <path-to-models>/segformer.b0.512x1024.city.160k.pth \
  --output modules/camera_semantic_segmentation/data/segformer.b0.512x1024.city.160k.onnx \
  --height 512 \
  --width 1024

python3 modules/camera_semantic_segmentation/tools/build_segformer_engine.py \
  --onnx=modules/camera_semantic_segmentation/data/segformer.b0.512x1024.city.160k.onnx \
  --engine=/apollo/modules/camera_semantic_segmentation/data/segformer.b0.512x1024.city.160k.engine \
  --fp16
```

## Build and test

Build the component plugin and execute unit tests:

```sh
bazel build //modules/camera_semantic_segmentation:libcamera_semantic_segmentation_component_plugin.so
bazel test //modules/camera_semantic_segmentation/...
```

Launch the component with Cyber RT:

```sh
mainboard -d modules/camera_semantic_segmentation/dag/segformer_cityscapes.dag
```
