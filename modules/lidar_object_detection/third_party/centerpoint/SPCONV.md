# CenterPoint sparse convolution

The reusable sparse-convolution graph executor is in the public
`wheelos_common//perception/ops/spconv` package at `pkgs/common/perception/ops/spconv`. Bazel
builds the generated spconv-backed operator target once for both consumers.
The common `perception/model_io/onnx` package owns the ONNX graph parser.
CenterPoint maps its validated graph into the model-neutral API. BEVFusion uses that parser target
and the same operator target; neither module carries a separate sparse engine.
No prebuilt `libspconv.so` or backend-selection flag is needed.

## Registry dependency

Apollo declares `spconv@2.3.8.bcr.3` in `MODULE.bazel`. With `--config=dev`,
the entry in `pkgs/bazel-central-registry` supplies the fixed official upstream
archive and pinned execution-phase generation tools with content-checked outputs.
Version pins, source-tree validation, generated-code BUILD rules and licenses
belong to that module; Apollo does not carry a second implementation of them.

See the registry module's
[`README.bazel.md`](../../../../pkgs/bazel-central-registry/modules/spconv/2.3.8.bcr.3/overlay/README.bazel.md)
for source provenance and packaging constraints. Bazel invokes the official
generator before native compilation; no generated-source supplement or
precompiled runtime library is used. It targets CUDA 12.8 / SM89 only.

Upstream spconv provides code-generated operators, not a loader for NVIDIA's
custom ONNX graph. The Apollo executor implements `SparseConvolution`, sparse
`Add`/`Relu`, `ScatterDense`, dense `Transpose` and `Reshape` for batch-one FP16 tensors and
weights. The NVIDIA sparse graph may also carry `int8` precision hints for
its former engine builder; as in the previous BEVFusion integration, this
runtime deliberately forces FP16 execution and validates the underlying graph
and initializers as FP16. Unknown precision hints, unsupported operators,
shapes and weights are rejected. The operator package directly depends on
`@spconv//:spconv`; model modules depend on
`@wheelos_common//perception/ops/spconv:spconv_backbone`. The executor does not
own voxelization, model weights, TensorRT heads or decode/NMS.

As the mapped non-root user in the managed dev container, build a precise
target through `.bazelrc.dev` and the local registry. No Python codegen setup
or `SPCONV_SOURCE_ROOT` is required:

```sh
bazel build --config=dev --config=gpu --config=opt \
  --lockfile_mode=off \
  --@rules_cuda//cuda:archs=sm_89 --jobs=4 \
  //modules/lidar_object_detection:libcenterpoint_runtime.so \
  //modules/bevfusion:libbevfusion_runtime.so
```

Codegen architecture and Bazel architecture must match the target GPU.
`@spconv//:spconv` supplies the generated C++/CUDA library, without a spconv
shared library. The common importer uses its own
`//perception/model_io/onnx/schema:onnx_proto_cc` schemas inside common. ONNX/Protobuf parsing uses the
host C++ compiler, not nvcc. The module installer stages the spconv and cumm licenses under
`lidar_object_detection/share/licenses/spconv`.
All CenterPoint runtime builds require CUDA; generation inputs are fetched
through the registry rather than a mutable host directory.

## Runtime contract and validation

The adapter receives FP16 `[N,5]` features and int32 `[N,4]` batch/ZYX indices,
uses the caller's CUDA stream, and returns owned FP16 `[1,256,180,180]` output.
Inputs must come from the existing voxelizer: unique in-bounds coordinates,
batch zero, and 1..160,000 active voxels. Each instance belongs to one CUDA
device and is not a concurrent inference context. Forward completes its
stream before returning; output remains valid until the next call or
destruction. Persistent tuning and capacity-retaining workspaces are per
instance; SubM index relations are cached only within one frame. The retained
workspace cap is 1 GiB, excluding weights and upstream sorting scratch.

From the `pkgs/common` module root,
`//perception/model_io/onnx:sparse_model_test` checks supported and malformed ONNX graphs without
requiring a GPU or model file.
`//perception/ops/spconv:spconv_backbone_test` checks numerical
execution against a small CPU reference, coordinate ordering, transpose,
repeated inputs, invalid count/grid recovery and nondefault streams.
Each package's `test_module` separately
checks public consumption without depending on those tests.
The test-only `sparse_backbone_benchmark` binary accepts:

```text
sparse_backbone_benchmark MODEL FIXTURE_LIST NEW_OUTPUT_DIR PASSES
```

Each list line is an absolute fixture path. A fixture contains four native
little-endian uint32 values `[count,z,y,x]`, FP16 `[count,5]`, then int32
`[count,4]`. Use at least two different frames and 2..9 passes. The benchmark
alternates two nondefault nonblocking streams, revisits the first frame after
shape changes, rejects nonfinite or changed repeated outputs, and writes
complete-forward wall timings and first-pass dense FP16 outputs. Timings
include sparse graph, dense scatter and stream completion, but exclude
uploads, downloads and file I/O. Compare warm passes separately from tuning
and capacity growth. Previously recorded NVIDIA comparison results are
historical evidence, not an available backend in the current build.

Selecting spconv 2.3.8 as the sole implementation is not production or
release acceptance: require same-input dense/detection comparisons, full
6,019-frame ten-sweep nuScenes mAP/NDS, matched latency measurements and
target-device stability before production deployment.
