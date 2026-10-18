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
specified in `CenterPointModelConfig`. Confirm intensity and time-lag units
against the selected model export before running. Zero point timestamps map to
a zero time lag. Frames with zero points or more than NVIDIA's 300,000-point
input limit are rejected.

`types/sweep_fusion.*` provides the real-time data-path primitive: it keeps a
bounded history, transforms prior sweeps into the current LiDAR frame using
the supplied sensor-to-world poses, filters close points from historical
sweeps, and encodes positive time lag for `ModelInputBuilder`. It rejects
out-of-order timestamps, invalid poses, and fused clouds over the configured
point limit. `CenterPointComponent` uses it to fuse incoming Cyber
PointCloud messages before inference.

For nuScenes, `prepare_nuscenes_manifest.py` reads the official info pickle,
validates the LiDAR files and creates JSONL records containing sample tokens,
source paths, and sensor transforms. For each frame it selects the nine nearest
historical sweeps, transforms them into the keyframe LiDAR coordinate system,
removes points within 1 m of the sensor as the upstream validation pipeline
does, and supplies positive time-lag features. At scene starts it repeats the
last available sweep, matching NVIDIA's CenterPoint config behavior. The
current sweep retains XYZ/intensity and uses zero time lag; the nuScenes
ring-ID field is not passed to the model.

A legacy manifest without sweep records remains single-sweep input and is not
comparable to the upstream 10-sweep validation configuration.

## Offline run

From `/apollo` in the managed development container, build with
`./apollo.sh build --config=dev lidar_object_detection`. The `dev` Bazel
configuration enables the local module overrides in `.bazelrc.dev`. The input
config is a protobuf text file containing
exactly five `input_feature` entries and paths to the SCN ONNX and RPN/CenterHead
engine. The paths in the config are resolved relative to the config file when
they are not absolute, and must resolve to the same files passed to the runner;
the converter rejects mismatches. For example, the feature list has this shape;
set scales to the checkpoint's actual training/export units:

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
  --config=<centerpoint-model-config.pb.txt> \
  --converter_bin=<path-to-pcd_to_centerpoint_input> \
  --centerpoint_bin=<path-to-upstream-centerpoint-executable> \
  --scn_onnx=<path-to-centerpoint.scn.onnx> \
  --rpn_centerhead_engine=<path-to-rpn_centerhead_sim.plan>
```

The wrapper stages inputs and model links in a temporary directory to match
the upstream executable's relative model/output paths, and passes the input
directory with a trailing separator required by upstream path concatenation.
It writes one raw
NVIDIA result text file per frame, `predictions.jsonl`, `timings.csv`, and the
captured upstream log. The JSONL output is prediction data, not nuScenes
evaluation JSON. Official nuScenes metrics require the dataset metadata and
sample tokens, which are not available for the current PCD input.

## nuScenes validation

Prepare the full validation split from the OpenDataLab nuScenes layout:

```sh
python3 modules/lidar_object_detection/tools/prepare_nuscenes_manifest.py \
  --data_root=/mnt/synology/nuScenes/OpenDataLab___nuScenes/raw/Trainval/train \
  --info_pickle=/mnt/synology/nuScenes/OpenDataLab___nuScenes/raw/Trainval/train/nuscenes_infos_val.pkl \
  --output=/tmp/nuscenes_val_manifest.jsonl
```

The data root and manifest source paths must be visible from the environment
running the converter and NVIDIA executable. Run all manifest entries with:

```sh
python3 modules/lidar_object_detection/tools/run_offline.py \
  --nuscenes_manifest=/tmp/nuscenes_val_manifest.jsonl \
  --output_dir=/tmp/nuscenes_centerpoint_val \
  --config=<centerpoint-model-config.pb.txt> \
  --converter_bin=<path-to-pcd_to_centerpoint_input> \
  --centerpoint_bin=<path-to-upstream-centerpoint-executable> \
  --scn_onnx=<path-to-centerpoint.scn.onnx> \
  --rpn_centerhead_engine=<path-to-rpn_centerhead_sim.plan>
```

Convert the token-aligned predictions and run the official nuScenes detection
evaluator. Install the isolated evaluation dependencies into the Python
environment used for the tools:

```sh
python3 -m pip install -r modules/lidar_object_detection/tools/requirements-nuscenes-tools.txt
python3 modules/lidar_object_detection/tools/evaluate_nuscenes.py \
  --manifest=/tmp/nuscenes_val_manifest.jsonl \
  --predictions=/tmp/nuscenes_centerpoint_val/predictions.jsonl \
  --data_root=/mnt/synology/nuScenes/OpenDataLab___nuScenes/raw/Trainval/train \
  --output_dir=/tmp/nuscenes_centerpoint_val/evaluation
```

The official result JSON is `nuscenes_results.json`; the devkit writes
`metrics/metrics_summary.json`, including mAP and NDS. `--format-only` writes
the result JSON without running the evaluator. Detection dimensions preserve
the runner's `[width, length, height]` order required by nuScenes. Do not use a
limited manifest for official validation metrics.

## Apollo runtime build and release status

The NVIDIA runtime is a Bazel target. It compiles the pinned CenterPoint
sources and CUDA kernels, and links against Apollo's CUDA, TensorRT, and
Protobuf repositories. NV spconv is a local shared-library dependency; set
`APOLLO_NVIDIA_SPCONV_LIBRARY` to the compatible `libspconv.so` and forward it
to Bazel with `--repo_env`:

```sh
export APOLLO_NVIDIA_SPCONV_LIBRARY=<path-to-compatible-libspconv.so>
bazel run --config=dev \
  --repo_env=APOLLO_NVIDIA_SPCONV_LIBRARY="${APOLLO_NVIDIA_SPCONV_LIBRARY}" \
  //modules/lidar_object_detection:install -- /apollo/modules
```

The install target stages the Cyber component and runtime DSO under
`lidar_object_detection-dev/local/lib` for the dev package and installs the
runtime's Bazel-managed shared dependencies under `/apollo/modules/lib`. Make
those installed dependency directories available to the dynamic loader in the
target environment. The installed DAG and component config use that package
prefix. Copy the matching `centerpoint.scn.onnx` and
`rpn_centerhead_sim.plan` into
`/apollo/modules/lidar_object_detection-dev/local/model`; these model artifacts
are not checked into the repository or bundled by the Bazel install target.
CUDA driver support must be available in the target NVIDIA software image.
The ONNX model and TensorRT engine are separate deployment artifacts; the
component config must point to the exact model files used to build the engine.

The pinned Lidar_AI_Solution checkout supplies spconv headers and a prebuilt
`libspconv.so`, but not the library implementation sources or a build system.
This integration therefore requires a compatible prebuilt spconv library; it
cannot rebuild spconv itself from that checkout.

The nuScenes Python dependencies remain evaluation-only. The validated
experiment used RTX 4060 (SM89), CUDA 12.8, and TensorRT 10.9 with local NVIDIA
compatibility adaptations. NVIDIA's upstream CUDA-CenterPoint documentation
lists SM80/SM86 and SM87, not SM89; the adaptations and resulting dataset score
are experimental and do not qualify the deployment as production-ready.
Production qualification still requires an end-to-end run of the installed
Cyber component, target-hardware compatibility and latency measurements,
stability/resource testing, and reproducible model and dependency provenance.

Upstream reference: [NVIDIA-AI-IOT/Lidar_AI_Solution, CUDA-CenterPoint](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-CenterPoint).
