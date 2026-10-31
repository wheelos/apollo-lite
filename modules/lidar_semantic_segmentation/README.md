# LiDAR semantic segmentation

This module is the neural LiDAR semantic segmentation entrypoint. It follows the
standalone module shape used by `modules/lane`:

- `component/`: Cyber component boundary.
- `conf/`: component runtime configuration.
- `dag/`: launch DAG.
- `inference/`: RangeRet and Cylinder3D inference and decoding.
- `proto/`: component config and output messages.
- `types/`: C++ data structures shared inside the module.

## Cylinder3D / PVKD (native C++)

`Cylinder3DComponent` is an alternative to RangeRet, not a replacement.
All online preprocessing, point MLP/max aggregation, asymmetric sparse
encoder/decoder (including saved-index inverse convolution), reconstruction,
flip voting and decoding execute in C++ with LibTorch. No Python interpreter,
PyTorch Python package, `spconv` Python package, or custom TorchScript sparse
operator is used online. CPU coordinate indexing plus ATen indexed matrix
products provides an FP32 implementation on CPU or CUDA;
it is **not a real-time-qualified sparse CUDA implementation**.

The implementation follows the local `Cylinder3D` and
`Cylinder3D-updated-CUDA` sources and
[Codes-for-PVKD](https://github.com/cardwing/Codes-for-PVKD).
The inspected revisions are Cylinder3D
`30a0abb2ca4c657a821a5e9a343934b0789b2365`, updated-CUDA
`129037162e63c2735f8adcacd9c030ee97132c22`, and PVKD
`8cb20e1048b3ff36bece846edc909488355bfd05`. Cylinder3D is Apache-2.0;
PVKD is MIT. Upstream code and weights are not vendored here.
PVKD's configurable-width inference and optional four-flip logit summation
are supported; upstream does not publish its full distillation training
implementation, so no distillation-training claim is made. Multi-scan input,
other datasets, stochastic/multi-scale TTA and training are not supported.

### Input, weights and output contract

Input coordinates are sensor-local Cartesian metres, without any world-frame
transform. The artifact owns the trained grid and cylindrical bounds (normally
`480x360x32`, `[0,-pi,-4]` to `[50,pi,2]`). Training uses
`floor((clip(polar)-minimum)/((maximum-minimum)/(grid-1)))`, **not** a
`range/grid` voxel size. Out-of-volume points retain their unclipped features,
matching training; only their grid indices are clipped. Nine features are
`polar-voxel_center, rho, phi, z, x, y, remission`. Apollo integer intensity
defaults to `1/255` scaling; configure it to match the actual sensor and
checkpoint. Non-finite/zero-origin points remain unknown with zero confidence.
Oversized clouds fail without truncation or publication.

The artifact stores FP32 tensors in `[k0,k1,k2,in,out]` sparse-kernel order and
metadata with checkpoint/config checksums and source revision. Every expected
tensor name, shape and value is validated before inference. Missing,
extra, incompatible or non-finite parameters fail initialization; there is no
partial checkpoint loading or randomly initialized fallback.
Artifacts explicitly distinguish updated-CUDA independent indices (default)
from original/PVKD spconv 1.2.1 shared `indice_key` semantics. Select
`--indice-semantics spconv1-shared --weight-layout spconv1` **only** for the
original shared-key network. That runtime reuses the first convolution's
coordinate pairs and iterates their kernel count, even when later declared
kernels differ. The exporter reshapes active planes to the effective geometry
and records discarded, never-executed planes (including their non-finite count).
It never replaces NaNs with zeros; any non-finite **active** value still fails.
Weight layout alone does not identify this distinct network computation.
The output uses the existing `LidarSemanticSegmentationResult` and its
SemanticKITTI **learning IDs 0..19**, not raw dataset IDs. Original point order,
header, source topic and frame are preserved. Range-image dimensions are
unset and point range-image coordinates remain `-1`.
For four-flip TTA, each vote starts from the original cloud, point logits are
summed like PVKD, then argmax/softmax confidence is computed.

### Export and launch

The evaluated baseline weights are preserved outside Git at
`/mnt/synology/models/cylinder3d/apollo-baseline-20261029/`, including the
original checkpoints, exact evaluated runtime archives, conversion configs,
label mapping, SHA256 manifest and compact evaluation evidence.
The sample config points to its `runtime/pvkd-half.ts`, retaining CPU as the
safe default; opt into the measured GPU backend with `device_id: 0` and
`use_spconv: true` after the explicit spconv build below.
Canonical provenance, exact reconversion commands and current design live in
`ad_agents/context/modules/lidar_semantic_segmentation/`:
`cylinder3d-model-artifacts.md` and `cylinder3d-native-baseline.md`.
The existing MMDetection3D `.pth` alongside this versioned directory is a
different model contract and is not substituted for these weights.

Only the offline weight conversion uses Python (`torch`, `PyYAML`).
Use a trusted checkpoint and its exact model YAML/label mapping; no model is
downloaded automatically. The exporter does not import the upstream model or
require its old CUDA environment. Explicitly select the checkpoint layout:
`spconv1` is `[k0,k1,k2,in,out]`; `spconv2` is `[out,k0,k1,k2,in]`.
Other legacy layouts are not inferred. Export with a PyTorch version compatible
with the deployment LibTorch (the validation environment uses 2.10).

```sh
python3 modules/lidar_semantic_segmentation/tools/export_cylinder3d.py \
  --source-dir /path/to/Cylinder3D-updated-CUDA \
  --config /path/to/Cylinder3D-updated-CUDA/config/semantickitti.yaml \
  --checkpoint /path/to/trusted-checkpoint.pt \
  --checkpoint-sha256 <expected-sha256> \
  --weight-layout spconv2 \
  --output /mnt/synology/models/cylinder3d/new-export.ts

# In managed dev, as the mapped non-root user:
bazel build //modules/lidar_semantic_segmentation:libcylinder3d_component_plugin.so
# For CUDA, build with --config=gpu and set device_id to the CUDA device index.
mainboard -d modules/lidar_semantic_segmentation/dag/cylinder3d_semantic_kitti.dag
```

Create the output's parent directory first; existing artifacts are never
overwritten. For PVKD, use its matching checkout/config/checkpoint, not a
different-width Cylinder3D YAML. The sample config defaults to CPU reference
mode (`device_id: -1`) and disables TTA. Match its source topic, sensor name and
DAG reader to the real sensor before launch. Run **only one** segmentation
backend per output channel. The component fails closed if its artifact is
missing; it does not accept a raw `.pt` checkpoint or a RangeRet engine.

### Numerical regression and boundaries

`cylinder3d_test` covers projection, sparse forward/inverse convolution against
dense convolution, malformed inputs and initialization failures. For full
network comparisons, generate a small deterministic **untrained** fixture from
the supplied updated-CUDA source. The reference runs the upstream network
definition with independent dense PyTorch test operators, not the C++ executor.

```sh
python3 -m modules.lidar_semantic_segmentation.tools.cylinder3d_reference_test \
  --source-dir /path/to/Cylinder3D-updated-CUDA --output /path/to/new-fixture
bazel test //modules/lidar_semantic_segmentation:cylinder3d_test \
  --test_env=CYLINDER3D_TEST_FIXTURE=/path/to/new-fixture --test_output=errors
# On an available CUDA device:
bazel test --config=gpu //modules/lidar_semantic_segmentation:cylinder3d_test \
  --test_env=CYLINDER3D_TEST_FIXTURE=/path/to/new-fixture \
  --test_env=CYLINDER3D_TEST_CUDA=1 --test_output=errors
```

Without the fixture, the full-network case explicitly skips. The fixture
checks all four votes, both kernel layouts, strict loading, point correspondence
and output confidence. The manual `cylinder3d_component_test` uses the same
fixture for real Cyber input/output channels, including no publication on
oversized input; run it in the isolated test runtime after building in dev.
These are numerical/integration checks, **not pretrained-model accuracy,
upstream spconv CUDA parity, full-resolution memory or latency qualification**.
Pretrained evaluation requires a trusted checkpoint, representative labeled
scans and measurements on the intended deployment device; fixture results
alone do not establish it.

The original shared-key network can also generate a reference with
`--indice-semantics spconv1-shared` and its matching original source directory.
That fixture includes NaNs in unused planes, verifies rejection of active NaNs
and independent-mode NaNs, and compares all four native votes against the
original network definition with independently emulated spconv 1.2.1 caching.

### Native labeled-dataset evaluation

`evaluate_cylinder3d` reads original SemanticKITTI XYZI and instance/semantic
labels, runs the actual C++ segmenter, and persists per-point learning-ID
predictions, confusion counts, class IoUs, point accuracy and raw frame timings.
It ignores GT learning-ID zero, aggregates confusion over points before
computing the 19-class mean, and counts prediction zero as a false negative.
It quantizes remission to Apollo integer intensity before inference, rather
than silently evaluating a different float-intensity input path.

```sh
bazel build --config=gpu -c opt \
  //modules/lidar_semantic_segmentation:evaluate_cylinder3d
# Run in the isolated dataset-mounted test container, as the mapped user:
bazel-bin/modules/lidar_semantic_segmentation/evaluate_cylinder3d \
  /path/to/model.ts /path/to/dataset/sequences/08 \
  /path/to/frozen-six-digit-frame-list.txt /path/to/new-results 0 0 3 4
```

The final arguments are CUDA device (`-1` for CPU), four-flip TTA (`0/1`),
warmup repetitions on the first scan, CPU threads, and an optional backend
(`aten`, the default, or `spconv`). Reported segment time
includes projection, sparse execution, GPU-to-CPU logits and result creation;
it excludes model initialization, warmup, input I/O and GT scoring. Sample lists,
checkpoint/input hashes and hardware must accompany reported results. Output
directories must be new. Predictions use `.learning`, not submission `.label`
files, because their values have not been inverse-remapped to raw dataset IDs.

### Measured SemanticKITTI subset (2026-10-29)

The pre-optimization native evaluator processed the same **101/4,071 sequence-08 frames** for
both author checkpoints: indices `floor(i*4070/100)`, `i=0..100`, totaling
12,367,107 points (11,808,329 non-ignored). Frame IDs and scan/label SHA256
hashes were frozen before the subset runs. These are **validation-subset
results, not full validation or official test scores**.

Hardware/runtime: RTX 4060 8 GiB, i7-12700KF, CUDA LibTorch 2.10.0+cu128,
FP32 tensors, four CPU threads, grid `480x360x32`, three warmup executions,
batch one and **no TTA**. Timings are one ordered pass on a shared host,
not an exclusive-hardware real-time certification.

| Native C++ checkpoint | 19-class mIoU | Point accuracy | Mean / p50 / p95 (ms) | Sequential FPS | Peak RSS (MiB) | Sampled whole-GPU peak (MiB) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Original Cylinder3D (init 32) | 65.62% | 91.33% | 753.10 / 757.26 / 932.16 | 1.33 | 940.1 | 2,278 |
| PVKD 0.5x (init 16) | 68.21% | 92.87% | 686.05 / 701.38 / 847.73 | 1.46 | 921.7 | 2,236 |

On these identical inputs, PVKD improves mIoU by **2.59 percentage points**;
its measured mean latency is 8.90% lower (1.10x throughput). It is still
**not real-time** for a 10 Hz LiDAR. The small speed difference despite reduced
channels is consistent with a correctness-first implementation retaining CPU
coordinate indexing, not an optimized spconv CUDA backend; no bottleneck
profiling claim is made. Rare-class quality remains poor: PVKD motorcyclist
IoU is 0%, other-ground IoU is 1.73%. All 19 classes have GT support in the subset.

GPU memory is sampled every 250 ms with `nvidia-smi`, including the 55 MiB
baseline and other GPU allocations; it is not an allocator-certified
per-process peak. RSS is process high-water memory.
Independent rescoring of every saved prediction exactly reproduced both
confusion matrices, mIoU and point accuracy.

The [PVKD published table](https://github.com/cardwing/Codes-for-PVKD/tree/8cb20e1048b3ff36bece846edc909488355bfd05)
lists **68.9%** original Cylinder3D, **71.4%** PVKD 0.5x and **71.8%**
PVKD 1.0x mIoU on the **official single-scan test set**; its testing instructions
use TTA. Different splits, TTA, integer-remission quantization and uncertain
checkpoint train/val provenance prevent treating the present numbers as a
leaderboard reproduction or directly interpreting their difference as a
regression. Upstream recommends optional train+val fine-tuning; this run does
not establish that sequence 08 was held out when these weights were trained.
No verified official timing on the same hardware/protocol was available;
`0.5x` denotes channel width, not a guaranteed 2x speedup.

Artifacts are local and ignored under `.cache/cylinder3d-validation/`:
`eval101-comparison.json` contains the verified report, per-class results,
protocol and prediction hashes; `eval101-manifest.json` freezes the inputs;
`eval101-{original,pvkd-half}/` retains point predictions and raw frame timings.
Checkpoint SHA256 values are
`50a26fc238a47d8072b32b3b6bed69adac9a52ecaa18db421ebce8b28914711d`
(original) and
`49a4f756edb0fe61e7e20b0bf20f144fd5ae3a23ffb15aa9beeae000853dbb2c`
(PVKD 0.5x). Both use explicit original-spconv shared-index semantics.
The numerical regression includes original-network dense-reference parity on
CPU/CUDA and unused-NaN handling, but **real-scan parity against the original
spconv CUDA kernels remains unverified**.

The evaluator and tests were built in an isolated minimal Bazel consumer of
these module sources, reusing shared caches. Root build attempts failed on
concurrent OpenLoc/Boost dependency edits; those changes were not touched.
This is module-runtime evidence, not successful qualification of the root
build graph.

### Sparse-index performance optimization

The measured first bottleneck was repeated CPU coordinate-pair construction,
not channel count alone. With per-convolution synchronization on frame
`000000`, the baseline spent 602 ms constructing/transferring pairs and
185 ms applying sparse convolutions. These instrumented figures perturb
overlap and are attribution evidence, **not deployment latency**.

The optimized implementation borrows mechanisms, not a new dependency, from
the inspected Apache-2.0 spconv implementations:

- [spconv 1.2.1 cached pairs](https://github.com/traveller59/spconv/blob/fad3000249d27ca918f2655ff73c41f39b0f3127/spconv/conv.py#L159-L201)
  and [modern geometry validation](https://github.com/traveller59/spconv/blob/263d6b47425ef843c82f997b12d8b714013d216c/spconv/pytorch/conv.py#L523-L546):
  reuse submanifold pairs only for identical immutable coordinate identity,
  spatial shape, effective kernel, stride, padding and device. The cache is
  local to each forward/TTA vote; it never reuses stale pairs across scans.
- [Device-resident indice pairs](https://github.com/traveller59/spconv/blob/fad3000249d27ca918f2655ff73c41f39b0f3127/src/spconv/spconv_ops.cc#L167-L205):
  pack all kernel indices into one device transfer and slice resident indices
  for gather/GEMM/scatter, including inverse convolutions, rather than allocate
  and synchronously copy two index arrays per kernel per convolution.
- [Direct-address coordinate lookup](https://github.com/traveller59/spconv/blob/fad3000249d27ca918f2655ff73c41f39b0f3127/include/spconv/indice.cu.h#L194-L205):
  use a CPU coordinate-address table bounded to 32 MiB, with a hash fallback
  for larger grids. This is only an index table, **not dense feature inference**.
  Unit-stride submanifold coordinates also avoid unnecessary division/modulo.

The model, precision, grid, remission conversion, point coverage, coordinate
ordering and kernel accumulation order are unchanged. Retaining device pairs
trades bounded per-forward memory for less repeated work. No FP16/INT8, point
subsampling, weight retraining or new sparse-runtime dependency is introduced.
The prototype `at::parallel_for` did not demonstrate a gain in this build
(OpenMP was not enabled for this target), so that experimental change was
removed instead of claiming parallel acceleration.

For diagnosis only, set `CYLINDER3D_PROFILE=1` to emit pair/application timing
per convolution. It synchronizes CUDA and must be unset in performance runs.
CPU/CUDA dense-oracle regressions cover both independent and original shared
indices; focused sparse tests cover cache identity/geometry changes,
large-grid hash fallback, ordering, boundaries and inverse convolution.

#### Matched optimized results (2026-10-29)

The original frozen 101-frame list and input/checkpoint hashes were reused.
For each model, immutable baseline/optimized executables ran in **ABBA order**,
with identical dynamic libraries, three warmups per run, four CPU threads,
RTX 4060, FP32 tensors, grid `480x360x32` and no TTA. No builds or profiling
ran concurrently with the comparison. Each variant has 202 measured frame
executions (two repetitions of the **same 101 unique frames**, not 202
independent scenes); percentiles pool those individual samples.

| Model / backend | Mean (ms) | p50 (ms) | p95 (ms) | Sequential FPS | mIoU |
| --- | ---: | ---: | ---: | ---: | ---: |
| PVKD 0.5x / baseline | 638.11 | 640.33 | 827.82 | 1.57 | 68.21% |
| PVKD 0.5x / optimized | **208.13** | **209.38** | **255.04** | **4.80** | **68.21%** |
| Original / baseline | 754.26 | 759.23 | 938.05 | 1.33 | 65.62% |
| Original / optimized | **271.88** | **274.70** | **346.48** | **3.68** | **65.62%** |

PVKD throughput improves **3.07x** (67.38% lower mean latency); the original
improves **2.77x** (63.95% lower latency). The two PVKD run means were
620.53/655.69 ms before and 204.87/211.40 ms after, showing a gain much larger
than observed shared-host variability. Use this matched baseline, not the
earlier unpaired 686 ms run, to calculate speedup.

All per-point predictions in all eight runs are byte-identical to their
frozen pre-optimization references, and confusion matrices/accuracy are
unchanged. Eight native regression cases passed for each CPU/CUDA and
independent/shared fixture combination (four complete test executions).
Changed C++ lint and formatting checks passed.

The memory cost did not erase the speed gain: PVKD peak process RSS rose
919.2 -> 936.1 MiB and sampled whole-GPU peak 2,236 -> 2,244 MiB; original
RSS rose 931.8 -> 970.1 MiB and GPU 2,278 -> 2,286 MiB. These GPU samples
include baseline/shared allocations and are not certified allocator peaks.
Both backends **still miss a 100 ms / 10 Hz budget** on every measured frame;
CPU pair construction remains material. This optimization does not claim
real-time qualification or parity with an optimized native spconv CUDA backend.

The reproducible runner, immutable executables, input/runtime manifest, all
eight raw reports, per-point predictions and GPU samples are retained under
`.cache/cylinder3d-validation/performance-v1/`. `comparison.json` is the
aggregated report; `comparison-protocol.json` freezes runtime linkage, binary
hashes and ordering; `compare.py` and `summarize.py` reproduce its protocol
and aggregation (new output directories are required).

### Source-built spconv GPU backend

The opt-in backend uses the repository's existing `@spconv//:spconv`
(`2.3.8.bcr.3`, upstream
`263d6b47425ef843c82f997b12d8b714013d216c`). It calls native GPU indice
generation and indexed GEMM for submanifold, regular and saved-rulebook inverse
convolutions. It does not vendor another spconv, require runtime Python/JIT, or
change the shared FP16 CenterPoint graph adapter.

The original `/home/wfh/01code/lidar_seg/Cylinder3D` operation order, inverse
rulebook associations, skip connections and reconstruction are retained.
Effective kernels from `spconv1-shared` exports remain effective kernels; the
new runtime never restores unused non-finite checkpoint planes. GPU-generated
downsample rows can be permuted relative to the reference backend, but inverse
restores the original row identities before skip addition and point decoding.
GEMM accumulation order changes, so bitwise logits/prediction identity is not
assumed.

All tensors remain FP32. spconv autotuning is passed `use_tf32=false`; the
submanifold center's ATen GEMM is wrapped in `NoTF32Guard`. Allocations use
ATen's allocator/current CUDA stream, including temporary GPU hash/unique
workspace. The native API receives CPU pair counts copied on that stream,
rather than relying on its default-stream count-copy path. Output allocation
is dynamic: no active-voxel upper bound silently truncates points.
Submanifold caches are per forward/vote and distinguish both backends.

The existing source package targets CUDA 12.8 / SM89. Build explicitly in the
managed dev container, using the mapped non-root user and shared caches:

```sh
bazel build --config=gpu -c opt --define=CYLINDER3D_SPCONV=true \
  --@rules_cuda//cuda:archs=sm_89 \
  //modules/lidar_semantic_segmentation:evaluate_cylinder3d \
  //modules/lidar_semantic_segmentation:cylinder3d_test \
  //modules/lidar_semantic_segmentation:libcylinder3d_component_plugin.so
```

Set `device_id: 0` and `use_spconv: true` in the component config, or append
`spconv` to the evaluator arguments. Defaults remain the reference backend;
requesting spconv without the build option or a CUDA device fails closed.
Ordinary CPU/CUDA builds do not acquire the SM89-only dependency.

For numerical qualification, run `cylinder3d_test` with
`CYLINDER3D_TEST_CUDA=1`, `CYLINDER3D_TEST_SPCONV=1` and
`CYLINDER3D_TEST_FIXTURE` pointing to each independent/shared reference fixture.
The explicit spconv case covers asymmetric kernels, cache reuse, regular and
inverse convolutions, duplicate coordinates and changing nondefault CUDA
streams. Full-network fixtures compare all four votes to the independent dense
oracle. Real pretrained accuracy/performance requires the labeled evaluator,
not just these small fixtures.

This is not a full TensorRT or all-GPU pipeline: projection, unique coordinate
construction and output-coordinate bookkeeping still use the CPU. Dense GEMMs
and much of reconstruction use ATen; normalization/activation now use a fused
CUDA kernel. The CUDA result path votes/decodes on device as described below.

#### Matched spconv results (2026-10-29)

The same frozen 101 sequence-08 scans, weights, integer-remission conversion
and original point order were reused on the RTX 4060, FP32, batch one, no TTA,
four CPU threads, three warmups. Each model ran **ATen/spconv/spconv/ATen**
using the **same immutable executable and dynamic libraries**, with no
profiling or agent-initiated builds during measurement. Each backend has
202 measured executions of 101 unique scans; this remains a subset on a
shared host, not a full-validation or official-test result.

| Model / backend | Mean (ms) | p50 (ms) | p95 (ms) | Sequential FPS | mIoU |
| --- | ---: | ---: | ---: | ---: | ---: |
| PVKD 0.5x / ATen | 214.62 | 213.57 | 263.07 | 4.66 | 68.21% |
| PVKD 0.5x / spconv | **100.55** | **100.78** | **115.19** | **9.94** | **68.21%** |
| Original / ATen | 267.63 | 269.62 | 339.88 | 3.74 | 65.62% |
| Original / spconv | **147.00** | **145.93** | **184.52** | **6.80** | **65.62%** |

This is **2.13x** PVKD and **1.82x** original throughput relative to the
already-optimized ATen backend. The two PVKD run means were 101.31/99.80 ms
with spconv versus 214.54/214.71 ms with ATen. Original spconv means were
146.98/147.02 ms versus 267.88/267.37 ms. Do not compare against an older,
unpaired baseline when calculating this speedup.

Both PVKD spconv passes preserve every point label exactly. All ATen passes
also match the frozen predictions. One original spconv pass changes six
non-ignored labels among 12,367,107 points; the other changes zero.
Its mIoU ranges from 65.6187026% to 65.6189120% (0.0002094 percentage-point
variation), and accuracy from 91.3313984% to 91.3314492%. PVKD accuracy remains
92.8694737%. All eight confusion matrices were independently recomputed from
the saved predictions and raw labels. There is **no claim of bitwise
original-model parity**, and autotuning/accumulation order differs from ATen.

Peak RSS was 953.3 -> 859.7 MiB (PVKD) and 976.6 -> 873.3 MiB (original).
Sampled whole-GPU peaks were 2,241/2,239 MiB and 2,283/2,281 MiB respectively;
these tiny GPU differences are not evidence of lower allocator peak.
PVKD still exceeds 100 ms on **106/202 executions**, and original on
202/202. The result is **not stable 10 Hz qualification**.

The independent/shared-NaN fixtures passed on CPU, ATen CUDA and spconv CUDA
(50 executed native test cases across six runs, without skipped selected
cases), including all four full-network votes. A separate ordinary CPU build
(without the spconv option or SM89 flags) passed all eight native cases.
The source-built shared plugin passed relocation checks, and actual Cyber
pub/sub passed on CPU, ATen CUDA and spconv CUDA, including rejection without
publication for oversized clouds. Scoped C++ lint and formatting passed.
Artifacts, immutable executable, input/model/runtime hashes, eight raw
reports, point predictions and memory samples are in
`.cache/cylinder3d-validation/spconv-performance-v1/`; see `comparison.json`
and `protocol.json`. The executed `spconv-compare.py` and
`spconv-summarize.py` scripts are retained in its parent directory.

The root graph is still blocked by unavailable `openloc@0.1.0`. Validation
uses the existing isolated source consumer and shared caches, with the same
spconv version as root. The optional Cyber validation consumer also uses root's
`wheelos_core@1.0.6`; its local configure-file tool wrapper loads
`rules_python`'s `py_binary` instead of Bazel's legacy native launcher, avoiding
an unsubstituted `%interpreter_args%` template in that dependency. This
validation-only wrapper does not change tracked dependency declarations or
qualify the root graph.

### Remaining CPU/data-path optimization

After freezing the preceding spconv executable, synchronized stage attribution
on the same 123,389-point frame `000000` found roughly 16.7 ms in CPU voxel
deduplication, 10--12 ms in full-logit readback and 13 ms in CPU decoding/result
construction. Sparse encoder/decoder still accounted for about 49.5 ms.
These synchronized timings perturb execution and are **not end-to-end
deployment measurements**.

Two independently piloted changes address those measured CPU/data costs:

- Replace per-point ordered-tree insertion and a second tree lookup with
  contiguous `(voxel_key, original_row)` sorting and a linear grouping pass.
  The grid's overflow-checked linear key preserves exactly the previous
  lexicographic voxel order, and inverse indices restore every original point,
  including duplicates. Measured deduplication dropped from 16.7 to 5.6 ms.
- Keep CUDA vote logits on device and perform stable argmax/confidence
  decoding there. FP32 network inference is unchanged; vote addition and
  softmax remain FP64 like the existing CPU reference. Only int32 labels and
  FP32 confidences return to the CPU: about 0.99 MB instead of 9.87 MB for
  this scan. The synchronized pilot measured 1.18 ms device decoding,
  1.08 ms readback and 2.66 ms protobuf result construction. The public raw
  `Run` logits interface and CPU decoding remain available and unchanged in
  meaning. Single-vote and four-vote confidence/point-correspondence tests
  compare against that interface.

CUDA segmentation uses the new compact result path for both ATen and spconv;
no extra runtime setting is required. Four-flip inputs are projected before
execution and retained for that call; this trades bounded input/FP64 device
storage for avoiding four full-logit host copies. Empty scans remain valid,
non-finite outputs fail without publishing, and each TTA vote must retain the
same point indices. Out-of-grid direct-executor coordinates now fail before
deduplication instead of waiting for sparse execution.

Set `CYLINDER3D_PROFILE_STAGES=1` only for synchronized attribution. It logs
projection, upload, point MLP, unique, pooling, encoder, decoder,
reconstruction, decode/readback and result stages. Both this setting and
`CYLINDER3D_PROFILE` must be unset in performance runs.

#### Matched compact-path results (2026-10-29)

The immutable prior spconv baseline and candidate ran in **ABBA order** on
the same frozen 101 scans, each model, two executions per unique scan per
variant. Both use identical dynamic libraries, RTX 4060, FP32 inference,
three warmups, four CPU threads, grid `480x360x32`, no TTA, no profiling and
no agent-initiated builds during timing.

| Model / spconv path | Mean (ms) | p50 (ms) | p95 (ms) | Sequential FPS | mIoU |
| --- | ---: | ---: | ---: | ---: | ---: |
| PVKD 0.5x / previous | 98.80 | 99.53 | 110.73 | 10.12 | 68.21% |
| PVKD 0.5x / compact | **69.65** | **69.94** | **80.22** | **14.36** | **68.21%** |
| Original / previous | 146.84 | 146.12 | 183.28 | 6.81 | 65.62% |
| Original / compact | **120.34** | **120.51** | **158.50** | **8.31** | **65.62%** |

Relative to the matched baseline, PVKD mean latency falls **29.5%**
(1.42x throughput), original **18.0%** (1.22x throughput). PVKD candidate
run means were 69.23/70.07 ms versus 99.41/98.19 ms baseline.
Every PVKD candidate point label matches the frozen reference exactly,
with unchanged 68.2092253% mIoU and 92.8694737% accuracy. Original repeats
retain the preceding spconv's zero/six-label numerical variation across
12,367,107 points per run; mIoU stays 65.6187026--65.6189120%. Every run's
confusion matrix was independently recomputed from its saved predictions.

PVKD has **0/202 executions above 100 ms**, versus 95/202 for the matched
baseline; candidate worst case is 94.84 ms. This satisfies the 100 ms target
**on this frozen subset/protocol only**, not on the full sequence or as a
production real-time guarantee. Original still exceeds 100 ms on 153/202.
Peak RSS was 879.4 -> 879.8 MiB (PVKD), 875.1 -> 907.4 MiB (original).
Sampled whole-GPU peaks stayed 2,239 MiB and 2,281 MiB respectively;
these samples are not certified allocator peaks.

The six CPU/ATen-CUDA/spconv-CUDA fixture runs passed all 50 selected native
cases, including original shared/unused-NaN semantics, reordered duplicate
voxels, single/four-vote confidence, malformed/empty input and nondefault
streams. Raw attribution logs are `spconv-stage-{profile,unique,decode}-v1.log`.
The separate CPU-only build passed all eight native cases; the rebuilt
shared plugin passed relocation checks, and actual Cyber pub/sub passed on
CPU, ATen CUDA and spconv CUDA. Changed C++ lint and formatting passed.
The immutable binaries, runtime/input/model hashes, eight frame reports,
predictions and memory samples are in
`.cache/cylinder3d-validation/spconv-next-performance-v1/`; see
`comparison.json` and `protocol.json`. The executed `spconv-next-compare.py`
and `spconv-next-summarize.py` scripts are retained in the parent directory.
The same isolated-consumer/root-graph limitations described above still apply.

### FP32 normalization fusion and device-coordinate reuse

Two further changes were qualified separately, without changing weights,
grid, point coverage, thread count, FP32 inference or FP64 voting:

1. Cache each inference BN's `weight / sqrt(running_var + eps)` during model
   initialization. Fuse the original activation/normalization sequence in one
   CUDA pass, retaining separate subtract, multiply and add roundings with
   explicit round-to-nearest intrinsics, not FMA. Point MLP keeps
   `Linear -> BN -> ReLU`, sparse blocks keep `Conv -> LeakyReLU -> BN`, and
   reconstruction keeps `Conv -> BN -> sigmoid`. In particular, BN is not
   incorrectly folded through the sparse block's nonlinearity. CPU execution
   retains the corresponding ATen operation sequence with cached scales.
2. Retain GPU coordinate tensors through sparse and inverse relations, bound
   to their immutable CPU coordinate owner. New geometries on the same owner
   reuse the upload. A per-forward cached relation with matching owner, shape,
   backend and device also establishes that uniqueness/bounds were already
   validated, avoiding repeated CPU lookup-table construction. Changed owners
   or grids are not trusted. Downsample coordinates still return to the CPU;
   native pair counts still use the stream-safe CPU path. This is partial
   device residency, not removal of all CPU synchronization.

Each change ran an independent baseline/candidate/candidate/baseline comparison
on the frozen 101 sequence-08 scans, two executions per unique scan per
variant, RTX 4060, no TTA, three warmups and four CPU threads. Executables,
runtime/model/input hashes, predictions, independent GT rescoring and
whole-device memory samples are frozen separately for each comparison.

| Isolated change / model | Baseline mean (ms) | Candidate mean (ms) | Candidate p95 (ms) | Candidate sequential FPS | Mean reduction |
| --- | ---: | ---: | ---: | ---: | ---: |
| BN fusion / PVKD 0.5x | 70.14 | 63.97 | 76.87 | 15.63 | 8.8% |
| BN fusion / original | 120.15 | 109.35 | 140.23 | 9.15 | 9.0% |
| Coordinate reuse / PVKD 0.5x | 63.70 | **51.89** | **59.76** | **19.27** | 18.5% |
| Coordinate reuse / original | 109.67 | **101.89** | **132.08** | **9.81** | 7.1% |

These are paired reductions within each experiment; the first experiment's
baseline and second experiment's candidate did not run in one common ABBA.
Final PVKD run means were 51.92/51.87 ms versus 64.21/63.19 ms for the
matched BN-only baseline. Its 202 executions have no 100 ms misses, maximum
65.29 ms, and every point label matches the frozen reference exactly:
68.2092253% mIoU, 92.8694737% accuracy. This is still only the frozen subset,
not full-sequence or production deadline qualification.

BN-only original passes match all reference labels. Both final original passes
change the same six labels in frame `000529` as the previously observed
spconv numerical variation; the point indices and old/new labels were checked
against that prior run. Final original mIoU is 65.6189120%, accuracy
91.3314492%; this is not bitwise parity. Original run means are
101.63/102.15 ms, with 108/202 executions above 100 ms and maximum 141.57 ms.
No claim of stable 10 Hz is made for that model.

For the coordinate-reuse comparison, peak RSS is 878.6 -> 848.8 MiB (PVKD)
and 890.8 -> 873.2 MiB (original); sampled whole-device GPU peaks stay
1,185/1,185 MiB and 1,349/1,349 MiB respectively. The BN experiment sampled
2,239 -> 1,185 MiB and 2,281 -> 1,349 MiB. These are shared-device samples,
not certified per-process allocator peaks or proof of a precise memory saving.

Synchronized attribution on warmed frame `000000` (123,389 points), separate
from performance measurement, shows point MLP about 10.3 -> 5.1 ms with BN
fusion. Across 44 forward sparse convolutions, coordinate/indice preparation
falls from about 22.2 to 8.2 ms for PVKD with coordinate reuse; forward
convolution application stays about 26.8/26.4 ms, inverse about 2.9 ms.
The analogous original preparation is about 26.7 -> 9.4 ms; final
forward/inverse application is about 94.1/9.8 ms. Preparation includes cache
lookup, CPU validation/bookkeeping, transfers and GPU indice construction;
application includes dispatch/allocation and native indexed GEMM, not
kernel-only compute time. Synchronization perturbs execution, so these
single-frame values must not be added to explain the measured subset mean.

The evidence now favors profiling/tuning the sparse convolution application,
especially for the original model, before replacing the remaining roughly
5 ms point MLP with another engine. TensorRT MLP, GPU projection/unique, and
other spconv algorithms remain unqualified experiments, not implemented
speedups. GPU quantization must preserve projection-boundary semantics;
FP16/TF32/INT8 remain disabled rather than silently trading away accuracy.

Each candidate passed both independent/shared-NaN fixtures on CPU, ATen CUDA
and spconv CUDA (56 executed selected cases across six runs), plus the
ordinary CPU-only build's nine cases. Added coverage compares fused operations
exactly against the old ATen ordering, including exceptional values, tail
rows, empty input and a nondefault CUDA stream. Sparse coverage verifies
device-coordinate reuse, stale-owner rejection, changed-grid validation and
downsample/inverse row identities on changing streams. Shared plugin
relocation, actual Cyber pub/sub on all three backends, scoped cpplint,
clang-format and Buildifier passed.

Reports are `.cache/cylinder3d-validation/bn-fusion-performance-v1/` and
`.cache/cylinder3d-validation/coordinate-reuse-performance-v1/`, each with
`comparison.json` and `protocol.json`. Runners and report reducers are
`bn-fusion-{compare,summarize}.py` and
`coordinate-reuse-{compare,summarize}.py` in their parent directory.
Attribution logs are `bn-fusion-attribution-{model}-{variant}-v1.log` and
`coordinate-reuse-attribution-{model}-v1.log`. The same isolated-consumer,
root-graph and shared-host limitations above still apply.

## RangeRet model boundary

RangeRet lives outside this repository and is treated as the training/export source. The RangeRet component is a TensorRT runtime and therefore requires a serialized `.engine`; a PyTorch `.pt` file or ONNX file cannot be configured directly.

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
python3 ad_agents/context/framework/skills/rangeret-engine-conversion/export_rangeret_onnx.py \
  --source-dir <path-to-RangeRet-repo> \
  --checkpoint <path-to-models>/rangeret-kitti-657.pt \
  --config <path-to-RangeRet-repo>/config/RangeRet-semantickitti.yaml \
  --output <path-to-models>/rangeret-kitti-657.onnx

python3 ad_agents/context/framework/skills/rangeret-engine-conversion/build_rangeret_engine.py \
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

## Offline native projection

`project_semantickitti` reuses the actual C++ deployment projector for controlled
training/evaluation inputs rather than approximating its angular math in Python.
In managed dev, build the target and run it in the dataset-mounted test runtime:

```sh
bazel build //modules/lidar_semantic_segmentation:project_semantickitti
bazel-bin/modules/lidar_semantic_segmentation/project_semantickitti \
  ordered-raw-scan-paths.txt new-projection-directory
```

Each list entry is one original SemanticKITTI four-float scan. The tool preserves
point order, quantizes remission to Apollo uint8 intensity and uses the default
64x1024 geometry, normalization and `1/255` intensity scale. For each zero-based
list index it writes raw float32 CHW `input`, per-point `range`, int32 flattened
`pixel` IDs, int32 `pixel_to_point` nearest-point indices and float32
`projected_range`; `samples.csv` records dimensions and point counts. Invalid
indices and unoccupied projected ranges are -1. No labels or model predictions
enter this tool. Freeze the ordered list and source fingerprints separately;
output directories are never overwritten.
