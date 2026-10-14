# LiDAR Object Detection

This is an isolated NVIDIA CenterPoint offline detection path for A/B
experiments. It does not modify the existing perception pipeline and does not
implement tracking or sensor fusion.

## Pipeline

```text
PCD directory
  -> PCL loader and configured five-feature adapter
  -> NVIDIA CUDA-CenterPoint executable
       CUDA voxelization -> NV spconv SCN -> TensorRT RPN/CenterHead
       -> CUDA decode/NMS
  -> predictions.jsonl + per-frame timings.csv + raw NVIDIA text results
```

`types/model_input.*` adapts each Apollo point cloud to packed five-float rows.
The first three channels must be `x, y, z`; the other sources and scales are
specified in `NvidiaCenterPointConfig`. Confirm intensity and time-lag units
against the selected model export before running. Zero point timestamps map to
a zero time lag. Frames with zero points or more than NVIDIA's 300,000-point
input limit are rejected.

## Offline run

Build `//modules/lidar_object_detection:pcd_to_nvidia_bin` in Apollo's managed
development container. The input config is a protobuf text file containing
exactly five `input_feature` entries, plus the SCN ONNX and RPN/CenterHead
engine paths. The paths in the config must match the paths passed to the
runner. For example, the feature list has this shape; set scales to the
checkpoint's actual training/export units:

```text
input_feature { source: POINT_FEATURE_X }
input_feature { source: POINT_FEATURE_Y }
input_feature { source: POINT_FEATURE_Z }
input_feature { source: POINT_FEATURE_INTENSITY scale: 1.0 }
input_feature { source: POINT_FEATURE_TIME_OFFSET_SECONDS scale: 1.0 }
```

No feature defaults are provided because incorrect units make inference
results invalid.

Build the upstream `centerpoint` executable from the pinned NVIDIA source
revision, with its required CUDA, TensorRT, and NV spconv dependencies. Then
run:

```sh
python3 modules/lidar_object_detection/tools/run_offline.py \
  --pcd_dir=<directory-with-pcd-files> \
  --output_dir=<new-results-directory> \
  --config=<nvidia-centerpoint-config.pb.txt> \
  --converter_bin=<path-to-pcd_to_nvidia_bin> \
  --centerpoint_bin=<path-to-upstream-centerpoint-executable> \
  --scn_onnx=<path-to-centerpoint.scn.onnx> \
  --rpn_centerhead_engine=<path-to-rpn_centerhead_sim.plan>
```

The wrapper stages inputs and model links in a temporary directory to match
the upstream executable's relative model/output paths. It writes one raw
NVIDIA result text file per frame, `predictions.jsonl`, `timings.csv`, and the
captured upstream log. The JSONL output is prediction data, not nuScenes
evaluation JSON. Official nuScenes metrics require the dataset metadata and
sample tokens, which are not available for the current PCD input.

## Compatibility status

The selected validation target is RTX 4060 (SM89), CUDA 12.8, and TensorRT
10.9. NVIDIA documents CUDA-CenterPoint for SM80/SM86 and SM87, and its checked
in TensorRT wrapper uses the older binding API. The selected runtime therefore
requires an experimental upstream build/compatibility adaptation; successful
build or execution on SM89 is not established here. The module does not bundle
the large NV spconv binary, ONNX model, or TensorRT engine.

Upstream reference: [NVIDIA-AI-IOT/Lidar_AI_Solution, CUDA-CenterPoint](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-CenterPoint).
