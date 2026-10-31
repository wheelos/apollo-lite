# LiDAR Object Detection

## NVIDIA PointPillars backend

The module also provides NVIDIA CUDA-PointPillars at pinned revision
`092affc36c72d7b8f7530685d4c0f538d987a94b`. It uses single-sweep KITTI XYZI
inputs, not the CenterPoint five-feature/10-sweep input contract. CenterPoint
and PointPillars share one Cyber component, one DAG, one component config, and
one output message. Their model-specific preprocessing, runtime operators, and
postprocessing remain separate.

In the managed non-root development container, build the runtime and shared
component:

```sh
bazel build --config=dev --config=gpu \
  //modules/lidar_object_detection:libpointpillars_runtime.so \
  //modules/lidar_object_detection:liblidar_object_detection_component.so
bazel test --config=dev //modules/lidar_object_detection:pointpillars_input_test
```

Build a TensorRT FP16 engine for the target GPU using the ONNX from the pinned
upstream model, then run packed little-endian float32 XYZI frames:

```sh
python3 modules/lidar_object_detection/tools/run_pointpillars.py \
  --runtime=bazel-bin/modules/lidar_object_detection/libpointpillars_runtime.so \
  --build-onnx=/path/to/pointpillar.onnx --engine=/path/to/new-pointpillar.plan
python3 modules/lidar_object_detection/tools/run_pointpillars.py \
  --runtime=bazel-bin/modules/lidar_object_detection/libpointpillars_runtime.so \
  --engine=/path/to/new-pointpillar.plan \
  --input=/path/to/frame.bin --output=/path/to/new-predictions.jsonl
```

The runner refuses to overwrite an existing engine or output. Input must
contain 1..300000 finite XYZI rows. Exceeding the engine's occupied-pillar
capacity (10000 for the pinned ONNX, up to 40000 supported by the kernels) is
an explicit error, not silent memory corruption or clipping. Point selection
within a pillar follows the upstream nondeterministic GPU implementation. Output boxes
are geometric centers, sizes along input x/y/z, yaw in radians about +z, and
zero-based KITTI classes Car/Pedestrian/Cyclist. No velocity is inferred.
JSONL is prediction data, not an official KITTI evaluation submission.

The versioned runtime ABI checks detection size and converts runtime failures
to explicit errors. The shared Cyber component dynamically loads this DSO,
requires an exact sensor frame match and explicitly configured positive
intensity scale, and publishes `LidarObjectDetectionFrameResult`. Select
intensity scale from the checkpoint's training units; there is no default
guessed scale. The single config is a protobuf `oneof`: replace its `centerpoint`
block with a `pointpillars` block to switch models. Configure the model paths,
revision, sensor frame, output channel, and the DAG reader channel before launch.
The checked-in example uses the `left_front` sensor frame and matching
`/apollo/sensor/vanjeelidar/left_front/PointCloud2` topic from the `ad_agents`
calibration example; the selected vehicle calibration remains authoritative,
so update both when using another sensor.
No production perception flow is enabled automatically.

Install both model runtimes and the shared component with
`//modules/lidar_object_detection:install_dir`; supply `--destdir <prefix>`
to the official rules_pkg installer. Model files and TensorRT engines are
separate deployment artifacts and are not bundled.
The imported Apache-2.0 sources and changes are recorded under
`third_party/pointpillars/UPSTREAM.md`. Building does not establish dataset
accuracy, target latency, long-run resource stability or vehicle readiness.

The offline input tests require only Python's standard library. GPU/ABI tests
are opt-in and require the runtime and target-built engine:

```sh
cd modules/lidar_object_detection/tools
POINTPILLARS_RUNTIME=/path/to/libpointpillars_runtime.so \
POINTPILLARS_ENGINE=/path/to/pointpillar.plan \
POINTPILLARS_SAMPLE=/path/to/kitti-frame.bin \
python3 -m unittest test_run_pointpillars test_pointpillars_runtime -v
```

The GPU tests exercise engine-capacity overflow rejection, valid frames after
a rejection, and nonfinite input handling. `POINTPILLARS_SAMPLE` is optional;
without it the test uses a synthetic in-range point, not an accuracy dataset.
`lidar_object_detection_component_test` is an explicit/manual Cyber integration
target; pass the same runtime/engine paths via Bazel `--test_env`. It initializes
the shared component with the PointPillars backend, processes input, rejects
the wrong sensor frame, and reloads the runtime. TensorRT plugin code remains
resident until process exit because its plugin registry retains creator
pointers; inference GPU buffers are still
released when each runtime is destroyed.
For direct test-binary execution against a local core checkout, set
`CYBER_PATH` to that checkout's `cyber` directory so Cyber can find its runtime
configuration.

The isolated migration's model artifacts are deployed under
`/mnt/synology/models/pointpillars/nvidia-092affc/`: `pointpillar.onnx`,
`pointpillar-sm89-fp16.plan`, a KITTI smoke sample, and `manifest.json` with
source revisions and SHA-256 checksums. The engine was built for RTX 4060
(SM89), CUDA 12.8 and TensorRT 10.9; rebuild it for other environments. These
paths do not configure vehicle channels or enable a default component.

## Existing CenterPoint backend

This is an experimental CenterPoint LiDAR detection module with an offline
inference/evaluation path and an optional Cyber component. It does not replace
the existing perception pipeline and does not implement tracking or
cross-sensor/object-level fusion. The Cyber component does perform temporal
single-LiDAR sweep accumulation.

## Pipeline

```text
PCD directory
  -> PCL loader and configured five-feature adapter
  -> NVIDIA CUDA-CenterPoint executable (offline)
       CUDA voxelization -> shared spconv SCN adapter -> TensorRT RPN/CenterHead
       -> CUDA decode/NMS
  -> predictions.jsonl + per-frame timings.csv + raw NVIDIA text results

Cyber PointCloud
  -> shared component input/frame validation and result publication
       CenterPoint: pose-aligned temporal sweep fusion -> five-feature adapter
         -> CenterPoint CUDA/TensorRT runtime -> model-specific postprocessing
       PointPillars: XYZI adapter -> PointPillars CUDA/TensorRT runtime
         -> model-specific postprocessing
  -> LidarObjectDetectionFrameResult
```

Both backends consume XYZ in the configured incoming PointCloud frame.
PointPillars uses the current XYZI sweep; CenterPoint builds its configured
five-feature input after aligning temporal sweeps into the current sensor frame.
The shared result records that frame and the selected backend, with common box
fields, but class IDs retain each model's taxonomy and PointPillars has no
velocity output. A common message schema does not by itself establish equivalent
predictions or accuracy.

`types/model_input.*` adapts each Apollo point cloud to packed five-float rows.
The first three channels must be `x, y, z`; the other sources and scales are
specified in `CenterPointModelConfig`. Confirm intensity and time-lag units
against the selected model export before running. Zero point timestamps map to
a zero time lag. Frames with zero points or more than NVIDIA's 300,000-point
input limit are rejected.
Voxelization also rejects frames with no in-range voxels or a voxel count
above capacity, returning an explicit runtime error instead of launching an
invalid CUDA grid.

`types/sweep_fusion.*` provides the real-time data-path primitive: it keeps a
bounded history, transforms prior sweeps into the current LiDAR frame using
the supplied sensor-to-world poses, filters close points from historical
sweeps, and encodes positive time lag for `ModelInputBuilder`. It rejects
out-of-order timestamps, invalid poses, and fused clouds over the configured
point limit. The shared component uses it for CenterPoint and bypasses sweep
fusion for PointPillars.
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

The Apollo runtime adapter is maintained under `inference/`; the pinned
upstream CenterPoint implementation is isolated under
`third_party/centerpoint/` with its license and provenance record. The adapter
exposes a versioned C ABI from the runtime DSO and checks its ABI version and
struct sizes before inference. It compiles the upstream CenterPoint sources
and CUDA kernels, and links against Apollo's CUDA, TensorRT, and Protobuf
repositories. The dependency boundary is:

| Dependency | Source / management | Runtime requirement |
| --- | --- | --- |
| CenterPoint CUDA/TensorRT implementation | Pinned, MIT-licensed source snapshot in `third_party/centerpoint/` | CUDA toolkit/driver and a compatible TensorRT installation |
| ONNX schemas and model loader | `@wheelos_common//perception/model_io/onnx:sparse_model`, including its owned schema snapshot | Protobuf, no CUDA headers or NVIDIA sparse-engine parser |
| spconv 2.3.8 | Registry module `spconv@2.3.8.bcr.3`, official archive plus Bazel execution-phase CUDA 12.8 / SM89 generation | Sole sparse backbone, compiled into the runtime; no imported spconv DSO |
| Model artifacts | Supplied by the deployment; not checked in | Matching SCN ONNX and TensorRT engine |

Use the registry-managed source generation as described in
[`SPCONV.md`](third_party/centerpoint/SPCONV.md), then build/install with CUDA.
No local source preparation or `SPCONV_SOURCE_ROOT` is needed:

```sh
bazel run --config=dev --config=gpu --config=opt --lockfile_mode=off \
  --@rules_cuda//cuda:archs=sm_89 \
  //modules/lidar_object_detection:install_dir -- --destdir /apollo/modules
```

The official mapping stages the Cyber component and runtime DSO under
`lidar_object_detection/lib`. Dependency collection is owned by product runtime
assembly, not inferred by the module installer. Make the required shared
dependencies available to the target's dynamic loader and adjust DAG/config
paths to the chosen prefix. Copy the matching `centerpoint.scn.onnx` and
`rpn_centerhead_sim.plan` into
`/apollo/modules/lidar_object_detection-dev/local/model`; these model artifacts
are not checked into the repository or bundled by the Bazel install target.
CUDA driver support must be available in the target NVIDIA software image.
For backend-specific deployment, use `:centerpoint_dir` or
`:pointpillars_dir` instead of the aggregate `:install_dir` target.
Each stages the shared component and selected runtime; PointPillars-only
installation does not build or install the CenterPoint/spconv runtime.
The ONNX model and TensorRT engine are separate deployment artifacts; the
component config must point to the exact model files used to build the engine.

The runtime uses the shared
`@wheelos_common//perception/ops/spconv:spconv_backbone` target, backed by
upstream spconv 2.3.8, as its only sparse implementation. The model-neutral
CUDA executor lives under `pkgs/common/perception/ops/spconv`; the shared
ONNX importer and schemas live under `pkgs/common/perception/model_io/onnx`.
CenterPoint owns only its model adapter and detector integration. The former NVIDIA spconv
import and backend selection have been removed. This replaces only the sparse
backbone; it retains the existing ONNX model, voxelizer, TensorRT head and
postprocessing. The install target packages spconv/cumm licenses but no NVIDIA
spconv DSO. See [`SPCONV.md`](third_party/centerpoint/SPCONV.md) for source
preparation, stream/lifetime contracts and the validation harness. This
implementation choice does not establish full accuracy or deployment
acceptance.

The nuScenes Python dependencies remain evaluation-only. The validated
experiment used RTX 4060 (SM89), CUDA 12.8, and TensorRT 10.9 with local NVIDIA
compatibility adaptations. NVIDIA's upstream CUDA-CenterPoint documentation
lists SM80/SM86 and SM87, not SM89; the adaptations and resulting dataset score
are experimental and do not qualify the deployment as production-ready.
Production qualification still requires an end-to-end run of the installed
Cyber component, target-hardware compatibility and latency measurements,
stability/resource testing, and reproducible model and dependency provenance.

Upstream reference: [NVIDIA-AI-IOT/Lidar_AI_Solution, CUDA-CenterPoint](https://github.com/NVIDIA-AI-IOT/Lidar_AI_Solution/tree/7c1623f234d5a9794fd862c9456f68f87ae7b900/CUDA-CenterPoint).
