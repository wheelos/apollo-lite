# Perception diagnostics

Independent, **read-only** Python evaluation and offline visualization of saved
perception results. Fusion, neural inference and production components do not
import this module. Changing a display filter never changes model inputs,
evidence or occupancy. No server, CDN, JavaScript package or network is needed
to open the exported viewer.

## nuScenes replay and export

Use the existing isolated test runtime with NumPy, protobuf and Pillow. Build
the fusion replay executable and descriptor set in managed dev as described in
[`lidar_fusion`](../../lidar_fusion/README.md). The dataset is read-only.

```sh
python3 modules/lidar_fusion/tools/prepare_nuscenes.py \
  --data-root /mnt/synology/nuScenes/OpenDataLab___nuScenes/raw/Trainval/train \
  --manifest /mnt/synology/nuScenes/mmdet3d_subset_validation/centerpoint/nvidia_centerpoint_sm89_full_val_10sweep_20261015/manifest.jsonl \
  --predictions /mnt/synology/nuScenes/mmdet3d_subset_validation/centerpoint/nvidia_centerpoint_sm89_full_val_10sweep_20261015/predictions_bazel_runtime_blocking_stream.jsonl \
  --schema /path/to/schema.pb --output /path/to/new-experiment \
  --scenes 3 --frames-per-scene 4
```

For each run in the generated `experiment.json`, execute the cached-detector
replay command in the fusion README using that run's `inputs`, `results` and
`detections` paths. The actual cached CenterPoint predictions are preserved;
RangeRet and all four fusion modes execute afresh. Detector time is unavailable.
Then export independently:

```sh
python3 modules/tools/perception_diagnostics/nuscenes.py \
  --experiment /path/to/new-experiment/experiment.json \
  --schema /path/to/schema.pb --output /path/to/new-diagnostics
```

Open `new-diagnostics/index.html` directly in a browser. Keep its `data.js`,
`viewer.js` and `camera/` next to it, or copy the entire directory. Controls
provide previous/next/play, semantic/group/explanation/height colors, ground and
background-filtered point views, GT/raw/fused boxes, unknown polygons, separate
measured/inferred/risk occupancy and diagnostic visibility. Click an instance or
point to inspect source, reasons, conflicts and original point ID. Images are
resized camera context with timestamp skew, not calibrated projected overlays.
`--max-points` bounds **display-only** stride sampling (default 8000 per frame);
the replay and metrics use all original points.

Projection version 2 preserves full native object/unknown/hypothesis evidence,
point evidence/decisions and optional-field presence. A missing raw score is
displayed as `null`; separate `ranking_score` retains historical matching order,
including the uncalibrated 1.0 ordering placeholder. Heading marks require
`orientation_reliable=true`; an enclosure axis is not a physical heading.
SemanticKITTI point inspection keeps trusted membership (`p[12]`) separate from
GT instance identity (`p[13]`); nuScenes has no such point-instance GT.

With the current fusion schema, strong cross-THING disagreement is purple and
independent of the legacy red conflict/consumer role. “Alternative THING
supports” draws original semantic core subsets, not replacement boxes. An
explicit shadow-partition replay also enables “Shadow semantic supports”:
click measured-support points for the full native point-ID set, parent unknown,
kind and semantic class/group. These overlays use display-sampled returns;
their counts and IDs retain the full subset. They do not fit boxes, establish
instance boundaries, or clear the retained parent's risk. Older payloads lack
these diagnostics; absence is not evidence that no contradiction exists.

`report.json` and `frames.csv` contain labeled fixed-operating-point results,
independent occupancy/reference/source audits, mode distributions and artifact
hashes. Matching uses score-descending, class-aware, one-to-one nearest XY
centers at an inclusive 2 m gate. GT needs actual LiDAR returns and a center in
the configured half-open base-frame ROI. Six comparable classes are evaluated;
barrier, cone, trailer and construction vehicle are excluded because fusion's
unified taxonomy cannot distinguish them. SEG_RECALL has no calibrated score
and uses fixed ordering score 1.0; this is **not AP**. Ignored annotation regions,
official class-specific range rules and full validation population are not
implemented: these results are **not nuScenes mAP/NDS**.

The raw detector is reported at both `evidence_score_min` (0.1) and the existing
`unsupported_score_min` (0.3). Accepted/fused outputs include fusion's own
support-dependent filtering. A precision difference from the 0.1 raw baseline
must not be attributed to semantic fusion alone: score filtering is a confounder.

RangeRet is cross-domain from SemanticKITTI, the local data has no lidarseg GT,
and absolute nuScenes poses are not a validated continuous local ODOM source.
This replay therefore makes no semantic mIoU, tracking or FREE-accuracy claim.
Unknown/unshown cells never imply FREE.

The checked-in
[`validation/nuscenes_subset_v1.json`](validation/nuscenes_subset_v1.json)
freezes the first 3-scene/12-frame diagnostic result. Its local artifacts are
under `.cache/lidar-fusion-validation/nuscenes-v1/`; open `viewer/index.html`.
The 0.3 detector baseline has 70.2% precision / 80.9% recall, versus full fusion
50.2% / 81.7% under the stated subset protocol. Semantic recall adds 43
comparable-class predictions with no net increase in TP: this is **not**
evidence of an algorithm-quality improvement. The baseline intentionally
retains this negative result rather than tuning on the evaluation subset.

## SemanticKITTI labeled validation

This adapter uses original validation sequence **08**, not training sequences.
The default experiment freezes three four-frame blocks starting at ordinals
0, 1000 and 2000 before looking at predictions. Labels remain outside inference.
XYZ/order are preserved; normalized float remission is explicitly quantized by
`round(remission*255)` for Apollo's integer field, with measured errors retained.
Both neural runtimes scale by 1/255. Coordinates remain the original LiDAR frame
(identity transform), not a guessed vehicle frame. The -1.73 m ground reference
is a diagnostic assumption, not annotation-derived calibration.

In the existing isolated test runtime:

```sh
python3 modules/lidar_fusion/tools/prepare_semantickitti.py \
  --data-root /mnt/synology/dataset/semantickitti \
  --schema /path/to/schema.pb --output /path/to/new-experiment
bazel-bin/modules/lidar_fusion/replay \
  /path/to/new-experiment/inputs /path/to/new-experiment/results \
  /path/to/pointpillar-40k-sm89.plan /path/to/rangeret-kitti-657.engine \
  0.00392156862745098 0.00392156862745098
python3 modules/tools/perception_diagnostics/semantickitti.py \
  --input /path/to/new-experiment/inputs \
  --results /path/to/new-experiment/results \
  --schema /path/to/schema.pb --output /path/to/new-experiment/viewer
```

The report includes confusion/mIoU, THING vs background precision/retention,
geometric ground vs semantic surface **proxy**, model/point identity audits and
visible-instance point-IoU matching. Point views are reported both on the entire
sweep and the common front ROI to avoid counting the detector's uncovered rear
hemisphere as a fusion-only loss. Semantic class 0 truth is ignored; prediction 0
on valid truth remains an error. Both observed-class mIoU and a 19-class mean
with absent classes zeroed are shown, not an official leaderboard score.

Green boxes enclose visible instance points; they are **not amodal box GT**.
All candidate variants use the same geometric box membership in original points,
class-aware one-to-one visible-point IoU >=0.5, >=5-point GT instances, and the
common XY front ROI. This diagnoses instance coverage/fragmentation, **not box AP**.
Raw and accepted DET have both 0.1/0.3 score ablations; semantic-only candidates
retain an explicitly uncalibrated fixed ordering score. In the baseline no labels
enter fusion.
The viewer adds a mapped semantic GT color mode and preserves original point IDs.

For fusion-only CPU comparisons, `lidar_fusion:replay_cached` can replace neural
replay using the same prepared inputs and frozen model PBs. Its output feeds
the same exporter without fabricated engine paths. Both model caches are
byte-checked, model inference is marked not recomputed, and semantic-oracle
provenance retains the same explicit opt-in gate. `timing.csv` contains all
warm-up/measured samples; viewer summary timings are not controlled performance
evidence. The accepted CPU-workspace result is in
[`validation/cpu_workspace_v1.json`](validation/cpu_workspace_v1.json).

The frozen negative result is
[`validation/semantickitti_subset_v1.json`](validation/semantickitti_subset_v1.json).
Open `.cache/lidar-fusion-validation/semantickitti-v1/viewer/index.html`.
The 12-frame matched-domain replay reproduces degradation: raw detector >=0.3
has 50 TP / 44 FP; full fusion has 50 TP / 322 FP under visible-point matching.
Read-only threshold ablation attributes 271 net FP to low-score DET acceptance,
without changing fusion or deleting occupancy. This is diagnostic evidence for
the quality objectives, not a delivered algorithm fix. The official validation
split is used; checkpoint training manifests have not been independently audited.

### Isolated semantic oracle (not model performance)

To isolate semantic prediction errors from fusion decisions, reuse both cached
model PBs. The replay accepts an optional segmentation directory after its cached
detection directory; cached timing fields are -1. Source identity checks and all
four degradation modes still execute. A control replay uses the original results
for both directories and must reproduce the frozen NORMAL payloads (excluding
only runtime `health.latency_ms`) before any oracle comparison.

`semantic_oracle.py --input INPUTS --models FROZEN_RESULTS --schema SCHEMA
--output NEW_ORACLE_DIR --allow-ground-truth-diagnostic` replaces only valid
SemanticKITTI GT learning classes with confidence 1. Ignored GT retains original
predictions; instance IDs never enter the semantic message. It records provenance
in `oracle.json` and refuses to overwrite an existing directory.

```sh
bazel-bin/modules/lidar_fusion/replay INPUTS NEW_ORACLE_RESULTS - - \
  0.00392156862745098 0.00392156862745098 FROZEN_RESULTS NEW_ORACLE_DIR
python3 modules/tools/perception_diagnostics/semantickitti.py \
  --input INPUTS --results NEW_ORACLE_RESULTS --schema SCHEMA \
  --output NEW_ORACLE_VIEWER --allow-semantic-oracle
```

The evaluator rejects marked oracle inputs without the explicit flag and labels
both report and viewer **NOT MODEL PERFORMANCE**. This jointly removes valid-point
semantic class/confidence errors, not geometry errors or detector errors. It is
neither a two-model truth experiment nor an instance/AP upper bound, and must not
be mixed into normal performance reports or production configuration.

Frozen evidence-control results are recorded in
[`validation/semantickitti_evidence_ablation_v1.json`](validation/semantickitti_evidence_ablation_v1.json).
Perfect valid-point semantics recovers five additional visible instances but
leaves object-point purity at 20.40%. Disjoint sequence-08 frames
3000..3003/3500..3503/4000..4003 provide actual model evidence: raw DET >=0.3
is 79 TP / 54 FP, while fusion is 87 TP / 250 FP. SEG_RECALL contributes
2 TP / 9 FP. Complementary evidence exists, but quality is still not accepted.
View `.cache/lidar-fusion-validation/semantickitti-holdout-v1/viewer/index.html`
or the explicitly labeled
`.cache/lidar-fusion-validation/semantickitti-truth-v1-oracle-viewer/index.html`.

## Other saved replays

The quality-policy reports retain `full_fusion` for **all** published candidates
and add `classified_fusion` for `classification_use=CLASSIFICATION_CANDIDATE`.
Identity-uncertain objects and retained weak detection hypotheses are counted
separately. This is a declared consumer contract, not relabeling unmatched
candidates as true positives. Per-frame `annotation_coverage` additionally
reports candidates with no valid labeled returns, while preserving historical
matching counts.

Point metrics include `trusted_object_points_thing`, separate from the original
geometric `object_points_thing`; always compare its precision **and** recall.
In the viewer use “Semantic-supported object points” or “Classification-use
candidates only”. Unknown polygons marked `point_supported_footprint` are drawn as unfilled
extents; the “Unknown risk” grid shows actual supported risk instead.

The opt-in policy's frozen development and previously unseen test results are
recorded in [`validation/quality_policy_v1.json`](validation/quality_policy_v1.json).
On the new 12-frame sequence-08 test, raw DET >=0.3 is 91 TP / 49 FP / 29 FN;
all new fusion candidates are 100 / 65 / 20, and classified candidates are
100 / 26 / 20. Identity-uncertain candidates remain published; weak hypotheses,
raw evidence and measured occupancy are retained. The strong point view has
98.69% precision / 84.07% recall, not complete target coverage. On nuScenes,
explicitly unqualified semantic identity falls back to raw detection accuracy
(106 / 45 / 25), not a demonstrated fusion gain. These are limited consumer-role
diagnostics, not production, full-dataset or drivable-area acceptance.

Selected offline bundles are under `.cache/lidar-fusion-validation/quality-v1/`:
`baseline-verified/viewer`, `development-final/viewer`, `test-final/viewer`
and `nuscenes-verified-viewer`; open each `index.html`. The reports retain
source fingerprints, per-frame matches and metrics. The local `comparison.json`
adds per-block counts, exact GT gains/losses and evidence/occupancy invariants.

Each SemanticKITTI frame also reports `unknown_support`: actual finite support
count, the largest component's semantic-ground/THING mixture, support AABB area
and whether that AABB spans the sensor origin. Ignored semantic GT is counted
separately. These diagnose mixed components; the AABB is not filled occupancy,
and semantic ground is not verified physical free space.

The next architecture/performance iteration is frozen in
[`validation/contracts_performance_v1.json`](validation/contracts_performance_v1.json).
It fixes temporal identity/unknown-footprint contracts and L6 source-layer
repair, then measures a 1.24-1.64x NORMAL CPU speedup on three fixed development
frames with exact output equality. New preselected blocks 750/1750/2750 retain
the prior accepted policy's 69 TP / 22 FP / 14 FN classified result versus raw
65 / 26 / 18; these are not newly caused gains from the performance change.
The experimental robust ground variant improves ground recall but increases
THING-as-ground errors and is not selected. Open
`.cache/lidar-fusion-validation/next-iteration/new-selected-viewer/index.html`;
the rejected experiment is separately available under `new-robust-viewer`.

`summarize_replay.py` verifies four-mode coverage and freezes local identities.
`evaluate_baseline.py` audits the Apollo record's single-frame and temporal
outputs without importing fusion implementation code. Its CLI is unchanged
apart from the new module location; see the fusion README for the full command.
The historical `sensor_rgb` result files are not regenerated by this extraction.

Run focused regressions in managed dev:

```sh
python3 -m unittest discover -s modules/tools/perception_diagnostics -p '*_test.py' -v
```
