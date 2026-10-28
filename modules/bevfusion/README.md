# BEVFusion

This is an independent, optional NVIDIA CUDA-BEVFusion inference module. It
does not modify the existing CenterPoint module. The migrated core is pinned
to NVIDIA `Lidar_AI_Solution` revision
`7c1623f234d5a9794fd862c9456f68f87ae7b900`; its MIT license and source
selection are recorded under `third_party/bevfusion/`.

The first supported model is ResNet50 FP16. No models, TensorRT engines, datasets, training/QAT/PTQ tools, visualizer, demo
assets, or spconv/ONNX schemas are included. Its sparse backbone uses the
public `@wheelos_common//perception/ops/spconv:spconv_backbone` target; the
common module directly depends on Bzlmod `spconv@2.3.8.bcr.3`, so
CenterPoint and BEVFusion do not maintain separate sparse-convolution
implementations. The shared custom sparse-ONNX parser is owned by
`@wheelos_common//perception/model_io/onnx:sparse_model`;
operator execution itself has no dependency on ONNX or detector modules.
Both packages belong to the single `wheelos_common` module.
BEVFusion no longer depends on CenterPoint
source or its BUILD targets.
TensorRT engines and the SCN ONNX must be generated for the target
GPU/software stack and are not assumed portable.
The local registry revision generates CUDA 12.8 / SM89 sources in Bazel;
consumers no longer need a local `SPCONV_SOURCE_ROOT` or Python codegen step.
Other GPU architectures require separately validated generated sources.
The sparse ONNX graph and initializer tensors must be FP16. Legacy node
`precision`/`output_precision` values of `int8` are engine-builder hints; the
shared executor intentionally forces FP16 execution, matching the previous
BEVFusion integration. Its sparse graph path includes XYZ `ScatterDense`,
dense `Transpose`, and `Reshape`; it does not execute INT8 kernels.

## Input contract

The API and offline manifest require one complete frame:

* Exactly six cameras in this order: `FRONT`, `FRONT_RIGHT`, `FRONT_LEFT`,
  `BACK`, `BACK_LEFT`, `BACK_RIGHT`.
* RGB8, interleaved HWC images, 1600x900, tightly packed 4800-byte rows;
  buffers must contain exactly 4,320,000 bytes. JPEG/other encodings, strides,
  dimensions, missing frames, and stale/unsynchronized timestamps are rejected.
* A LiDAR reference timestamp matching the frame timestamp, nonempty packed
  float32 rows in exactly five model features, and no more than the configured
  maximum (at most 300,000). For offline input, the file is already
  model-feature ordered. Input points are converted to FP16 only after
  validation and FP16 range checks; over-limit or unrepresentable input is
  never clipped.
* Four row-major camera-major arrays of six 4x4 matrices:
  `camera_to_lidar`, `camera_intrinsics`, `lidar_to_image`, and
  `image_augmentation`. Every value must be finite; matrices inverted by the
  upstream geometry path must also be nonsingular.
* Every camera timestamp must be within the configured nanosecond tolerance
  of the LiDAR reference timestamp.

`types/BoundedSynchronizer` provides a bounded six-camera/LiDAR packet queue.
Queue overflow, out-of-order timestamps, missing cameras after the wait bound,
and samples outside the timestamp tolerance are explicit rejections. The
optional Cyber component consumes six configured `apollo::drivers::Image`
channels plus one configured `PointCloud` channel; the first camera channel is
the component's required primary DAG reader, and the component creates readers
for the other five cameras and LiDAR. It copies/validates RGB8 image payloads,
checks each configured camera/LiDAR frame ID, converts each cloud to packed
`x,y,z,intensity,point_time_offset_seconds` rows with five explicit configured
scales, synchronizes complete bundles, and runs the same strict input contract.
No vehicle topics, frame names, calibration values, or tolerances are assumed.
Calibration is an explicit static snapshot in the component configuration:
all four row-major camera-major matrices and a nonempty calibration revision
must be supplied alongside expected camera and LiDAR frame IDs. This does not
query or replace a runtime calibration/TF authority.

**Sensor/model contract limitation:** this adapter supports only one incoming
raw `PointCloud` per frame, configured as
`SINGLE_SWEEP_EXPERIMENTAL`, with the feature declaration
`XYZ_INTENSITY_POINT_TIME_OFFSET_SECONDS`. It requires nonzero nanosecond
timestamps on every point, computes signed seconds relative to the cloud's
measurement time, enforces a configured absolute per-point time-offset bound,
and applies five explicit feature scales. It does not accumulate sweeps,
perform pose alignment, infer intensity units, or convert arbitrary point
timestamps. These settings must match the selected export exactly. In
particular, this single-sweep adapter is **not** the upstream nuScenes
multi-sweep validation path and is not claimed to reproduce its trained input
distribution. Do not enable it for model inference until the model's feature
layout, intensity units, timestamp units, and sweep policy have been verified.

## Offline inference

Build the GPU targets in the managed development container. Configure
`conf/bevfusion_model.template.json` with the five compatible upstream model
artifacts, runtime library path, and nonempty `model_revision`; blank paths and a null threshold are
intentional and are rejected until a deployment is configured. The JSON
manifest has a root `max_timestamp_skew_ns` and a nonempty `frames` array.
Each frame contains `timestamp_ns`, an ordered `cameras` array (each with
`name`, `timestamp_ns`, `format: "RGB8_HWC"`, `width`, `height`,
`stride_bytes`, and raw RGB file `path`), a `lidar` object with timestamp,
`point_count`, `feature_count: 5`, and raw packed float32 `path`, and a
`calibration` object containing the four 96-float matrices listed above.
The calibration object also requires a nonempty `revision` identifying the
snapshot used for that frame.
Relative data paths resolve from the manifest directory; relative model paths
resolve from the model-config directory.

```sh
./bazel-bin/modules/bevfusion/bevfusion_offline \
  --config modules/bevfusion/conf/bevfusion_model.json \
  --manifest /path/to/manifest.json \
  --output /path/to/results.jsonl
```

The runner publishes results only after all frames succeed. Each output row
records per-frame inference and total elapsed time, the pinned upstream
revision, configured model and calibration revisions, model artifact paths,
and unmodified upstream detection semantics:
input LiDAR coordinate frame, x/y box center, bottom-center z, dimensions in
upstream x/y/z order, yaw from `atan2(rot[0], rot[1])`, and upstream x/y
velocity. No transform into a vehicle/world frame is implied.

## Optional Cyber component

`libbevfusion_component_plugin.so` subscribes to the configured six image
channels and LiDAR channel and publishes `BevFusionFrameResult`. The supplied
`conf/bevfusion_component.pb.txt` and DAG are intentionally unset: configure
the actual channels, frame IDs, static calibration snapshot/revision, model
artifacts, feature/sweep declarations and scales, point-time bound, and
measured nonzero synchronization tolerance/wait bound before use. Set
`lidar_feature_contract: XYZ_INTENSITY_POINT_TIME_OFFSET_SECONDS` and
`lidar_sweep_policy: SINGLE_SWEEP_EXPERIMENTAL` explicitly; the component
rejects any other/unspecified policies. The feature layout is a migration
adapter declaration, not evidence that an unavailable model export has those
semantics. Verify intensity/time units, input sweeps, and scale values against
the selected export before enabling inference. The DAG's primary reader
channel must match `camera_channel[0]`. Rejected inputs and runtime failures
are logged and produce no result message.

The `install` target stages the optional plugin/runtime under
`/apollo/modules/bevfusion-dev/local/lib` and configuration under
`/apollo/modules/bevfusion-dev/local/conf` in the default dev setup. If using a
different install prefix, update the absolute library and config paths in the
DAG to match that prefix. The DAG is not enabled by any default pipeline.

## Acceptance status

CPU adapter/synchronization tests cover contract acceptance and invalid
camera order, layout, timestamp, feature count, calibration, point count, queue
overflow, ordering, timeout, and skew cases. CUDA/TensorRT compilation and
runtime/model accuracy acceptance require the configured target environment.
The official pinned NVIDIA Box model archive and sample-data archive were
retrieved and checksum-recorded in
`/mnt/synology/models/bevfusion/nvidia-7c1623f/artifact-manifest.json`;
compatible ResNet50 FP16 ONNX exports and a prepared official example frame
are staged there, not copied into this source module. The MMDetection3D
`.pth` checkpoint in the pre-existing `/mnt/synology/models/bevfusion`
directory is a different export and is not used by this runtime. TensorRT
engines were built for the CUDA 12.8 / TensorRT 10.9 RTX 4060 target stack.
The pinned NVIDIA official example completed one GPU inference and five
repeated frames, each producing 33 detections. This is a positive runtime
smoke only: the example uses synthetic equal timestamps, and no dataset
accuracy, vehicle timing, general latency, or production-readiness acceptance
is claimed.
