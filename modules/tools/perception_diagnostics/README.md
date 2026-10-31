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

## Ground optimization experiment (shadow only)

`ground_experiment.py` provides a working, read-only path from full cached
fusion evidence to original-point-ID ground estimates, labeled comparison,
and hypothetical measured-occupancy changes. It never writes a `FusionOutput`,
changes production defaults, or emits FREE. Inputs use the frozen sensor-to-base
transform in metres; each frame starts afresh. Semantic/instance GT is loaded
only after fitting and cannot initialize the estimator.

The initial design compares the current Cartesian tile/global-height-band fit
with anchored radial piecewise lines. Primary sources reviewed on 2026-10-30:

| Reference / pinned revision | Verified mechanism and reuse decision |
| --- | --- |
| [linefit](https://github.com/lorenwel/linefit_ground_segmentation/blob/e6b9418cc5756e30e4ea87b11d8792f936a2002a/linefit_ground_segmentation/src/segment.cc), BSD-3-Clause | Per-sector minimum-height bins, bounded least-squares line fits and ground-height continuity. Use this mechanism as an independently implemented NumPy reference; do not copy source or add ROS/PCL/Boost. Upstream's example launch node is not evidence that our sensor contract is validated. |
| [Patchwork++](https://github.com/url-kaist/patchwork-plusplus/blob/3e6903a1d5537a4cc2ace897b0bbb98a92d6014c/cpp/patchworkpp/src/patchworkpp.cpp), BSD-2-Clause | Concentric patches, low-point seeds, covariance plane fit, uprightness/elevation/flatness gates and adaptive state. A useful next comparator for full 2D terrain; not adopted here because state qualification and its additional integration are outside the first diagnostic path. The inspected Python smoke test includes nonempty partition checks, but no ground-accuracy assertions. |
| [ETH traversability](https://github.com/leggedrobotics/traversability_estimation/tree/14d24c059e1c43466aadf328280adf6394d78039/traversability_estimation_filters), BSD-3-Clause license text | Separate slope, roughness and step layers over valid elevation cells. Reuse this separation of concerns, not its ROS/grid_map/pluginlib implementation. Ground classification alone must not authorize vehicle clearance or FREE. |

This is conceptual reuse, not a vendored dependency or an upstream-equivalent
implementation. No upstream code or dependencies are distributed. Direct
integration would require a separate transitive-license/dependency review.
The existing baseline remains the production choice pending evidence.

The reference has fixed experimental defaults: 360 sectors, 0.5 m bins,
1 m maximum support gap, 4 m maximum line span and at least three observed
bins per line. A new component needs an observed anchor within 8 m and the
existing configured expected-ground height band; only a nearby accepted line
can continue beyond that band. Slope, maximum vertical fit residual and point
distance gates reuse the frozen fusion configuration. These are experimental
parameters, not validated vehicle thresholds. Unlike upstream, this reference
does not search neighboring sectors, bridge long gaps or carry adaptive state.
Unsupported estimates remain NaN. A radial slope is not a full terrain normal
and cannot establish cross-slope traversability.

Create a local JSON experiment with `runs` entries containing unique `name`,
`inputs` (prepared `manifest.json`) and `results` (full current-source replay)
paths, optional `options` matching `RadialOptions`, and optional `continuity`
matching `ContinuityOptions`. Then run in the existing isolated test runtime:

```sh
python3 modules/tools/perception_diagnostics/ground_experiment.py \
  --cases /path/to/experiment.json --schema /path/to/schema.pb \
  --output /path/to/new-ground-comparison
```

The baseline measured occupancy must exactly reproduce the native output.
The tool checks source/GT hashes, cloud/output identity, rigid transform,
point ordering and input immutability; each `.ground.npz` preserves original
point IDs, estimated ground heights and the experimental ground mask.
`report.json` retains per-frame full-ROI/front-domain confusion, THING errors,
new GT non-ground removals, and changed measured cells. Inferred/unknown layers
stay fixed in the hypothetical occupancy union: this is not a recomputation of
object association, unknowns or production fusion.

The primary objective is **continuous observed ground around ego**, not perfect
pointwise classification. `ground_continuity.py` constructs a 0.5 m observed
grid, blocks measured obstacle cells, and connects four-neighbor ground cells
only across height changes <=0.2 m. It selects the largest component touching
an observed seed within 3 m of base XY origin. If this disk has no unblocked
ground, an explicit bounded fallback uses the nearest observed ground ring
(at most 8 m, with a half-cell margin). The effective radius and fallback
status are reported. This never creates points in the ego blind spot or fills
holes, nor joins diagonally separated regions. Set `max_seed_radius` equal to
`seed_radius` to disable the fallback. Access across that blind zone is unproven.
The SemanticKITTI frame is still a native LiDAR-frame diagnostic, not independently
verified vehicle geometry. Cell counts/area describe observed support, not all
terrain or a drivable polygon. Near-ego connectivity is reported at 5/10/20/30 m,
alongside the semantic ground-cell proxy.

An ablation downsamples obstacle support into 0.1 m voxels and filters isolated
26-neighbor voxel components containing at most three **original** points.
Raw counts are retained: many returns in one voxel must not turn a dense narrow
obstacle into an isolated singleton. This filter never deletes source points,
model evidence or native occupancy; removed points are explicit in the shadow
mask. `max_isolated_points=0` disables filtering. Filtering can remove a real
small obstacle and requires application-specific acceptance before deployment.
These grid, step, seed and sparsity settings are frozen experimental defaults,
not inferred vehicle capabilities.

The revised comparison prioritizes increased near-ego connected area and reports
point errors without an absolute zero-error gate. Instance retention includes
all baseline instances, separately reports sparse instances with fewer than
three baseline spatial voxels, and retains the historical supported-instance
subset for comparison. Complete erasure or complete replacement of the original
support prevents native-integration eligibility; partial point and voxel losses
remain explicit. Counts are instance-frame observations, not unique tracks.
The primary check uses the unfiltered candidate; filtering
has its own supported-instance check and is not qualified for production.
Frames lacking observed seeds cannot provide positive continuity evidence.
Per-frame connected-area regressions must be reviewed before native integration;
an average increase alone does not clear the experiment gate.
Passing only permits a native integration experiment, not
deployment or small-obstacle qualification. Baseline/candidate both have raw
and filtered connectivity variants, so filtering is not silently attributed
to better ground fitting. The NPZ also retains connected grid XY indices and
the candidate filtered obstacle mask. No-GT runs provide structural checks only.
Both domains still need full 2D slope/step/roughness, verified ray origins and the
existing vehicle/footprint contracts before traversability.

`ground_validation.py --previous PREVIOUS_REPORT --schema SCHEMA
--inventory RAW_GT_INVENTORY --output NEW_OUTPUT` recomputes the frozen cached
comparison, traces radial rejection and connectivity boundaries, and runs the
paired raw/GT inventory under explicit sensor-frame height hypotheses. GT is
loaded after estimation; height sensitivity is not calibration or holdout
acceptance. `ground_validation_closure.py --root OUTPUT` adds closure evidence, including
comparisons using the intersection of the original observed seed boundaries.
No shared observed seed is a blocked comparison, not a reason to enlarge the
seed radius. Independent-seed area regressions remain reported even when this
controlled comparison identifies seed selection as a confounder.

### Independent patch and observed-terrain comparison

`ground_patch.py` adds a stateless NumPy comparator using observed low points
in fixed XY patches and full-rank local planes. It is conceptual reuse of
patch-based ground mechanisms, not an upstream-equivalent Patchwork++
implementation. It does not change the native or radial estimators. Frozen
height, slope, residual and patch-layout hypotheses remain experimental;
unsupported patches do not provide ground or fill gaps.

`spatial_support/observed_terrain.py` separately measures raw observed heights
and plane-normal residuals. Its sampling and residual statistic are explicit;
these quantities are not the existing fitted-height roughness proxy, an
upstream-equivalent ETH filter, or calibrated physical terrain accuracy.
The comparison keeps original point IDs, all-instance original-support losses,
native/radial controls, and seed comparability separate from aggregate area.
Semantic GT is evaluation-only. More connected cells do not establish that
the same observed component was retained or that real obstacles survived.

In the isolated test runtime, execute the frozen cached report and paired
raw inventory without changing inference:

```sh
python3 modules/tools/perception_diagnostics/ground_patch_experiment.py \
  --previous /path/to/frozen-ground/report.json --schema /path/to/schema.pb \
  --inventory /path/to/raw-gt-inventory.json --output /path/to/new-patch-results
```

The exporter writes `report.json`, `ground-report.json`, `case-matrix.json`,
cached `<case>/<file>.ground.npz` and raw `patch-raw-<sequence>-<ordinal>.npz`.
Each raw archive retains all three fixed height hypotheses separately;
they are sensor-frame sensitivity experiments, not calibration or holdout
qualification. The new patch artifacts are comparison evidence, not an
automatic replacement for the frozen radial inputs of the spatial exporter.

### Observed terrain surface and depression metrics

`terrain_surface.py` is a stateless, frame-local terrain estimator. It groups
raw returns into observed height layers per XY cell, then follows a supported
observed root through compatible four-neighbor layers. An optional expected
height constrains initialization, not every distant cell. Explicit roots allow
bridge decks to be selected without merging the lower surface. Automatic root
selection is a diagnostic policy, not independently verified vehicle ground.
Missing cells are not filled, and a nearest observed root does not prove access
across the sensor blind zone.

Slope uses supported local observed elevations. Step diagnostics distinguish
adjacent-plane discontinuity from within-cell height span; a span alone cannot
identify a physical step rather than overlapping surfaces. Depression depth
requires observed floor returns and a surrounding supported rim. Widths describe
observed floor-cell extents, not reconstructed unseen pit boundaries. Missing
floor or incomplete rim remains unknown.
Depth is a relative observed height measurement, not proof of a physical pit.
When a selected upper surface also has lower returns in the same cell,
`unknown_layer_ambiguity` distinguishes an unresolved lower level from
`no_depression_observed`; depth remains NaN rather than assigning pit semantics.

The independent risk mask does not influence raw surface selection, but its
points are excluded from the final ground mask. Candidate surface membership
and retained risk remain separately inspectable. Geometry alone cannot identify
a semantic object whose returns coincide with the selected surface. These
outputs measure terrain observations; they do not emit FREE, apply calibrated
vehicle limits, or change production fusion.

Run the independent mathematical cases in managed dev:

```sh
python3 modules/tools/perception_diagnostics/terrain_surface_experiment.py \
  --analytic-only
```

The runner reads the frozen experiment manifests, configurations and inventory
under `--root` (the repository by default). A real-input run strictly requires
the original sources. If those sources are unavailable, explicit
`--allow-archived-raw` recomputes from previously archived original XYZ and IDs;
it does not reuse prior predictions. Archive hashes and cached coordinate
alignment are checked, while unavailable original-binary identity and GT audits
remain blocked. Outputs under `.cache/lidar-fusion-validation/terrain-v4/`
include `case-matrix.json`, `real-data-report.json`,
`persisted-output-check.json` and per-frame `.terrain.npz` archives.
Surface-supported status is distinct from risk-filtered root connectivity
and never constitutes a vehicle traversability result.

### Observed traversability detection

`terrain_traversability.py` applies explicit terrain limits to the observed
surface without changing its fit, selected root, raw layers or source risks.
Its two stages keep ground quality separate from obstacle qualification:

| Output | Meaning |
| --- | --- |
| `terrain_state` | `UNKNOWN`, `GROUND_SUPPORTED`, `TERRAIN_PASSABLE` or `BLOCKED` under the supplied diagnostic terrain limits |
| `decision_state` | `CANDIDATE` only with qualified terrain and observed, empty independent risk layers; `BLOCKED` for known violations or measured/object risk; otherwise `UNKNOWN` |
| `terrain_candidate` | Observed surface satisfying slope, roughness, step and depression checks; not a vehicle safety decision |
| `risk_constrained_candidate` | Terrain candidate after independent risk and risk-coverage checks |
| `terrain_connected`, `risk_constrained_connected` | Separate four-connected components from the same exact observed root; no reseeding around a blocked root |
| `reason_bits` | Overlapping terrain, missing-evidence, risk and connectivity reasons; not a single winning explanation |

Missing measurements are not zeros. A within-cell height span or an unresolved
lower layer is ambiguous, not proof of a physical step or pit. Known excessive
boundary steps and measured depression depths can block unsupported cells.
Cells without floor returns stay unknown; neither gap filling nor diagonal
connections are allowed. The root is observed surface support, not verified
vehicle ground or ego reachability across the sensor blind zone.
An isolated cell is not automatically deleted: a coplanar singleton can still
be supported by full-rank neighboring observations, so small support alone is
not a noise classifier. A smooth bowl can remain in the connected selected
surface without receiving a separate depression-depth measurement;
`no_depression_observed` is not proof that no physical pit exists.

Independent risk uses separate measured, object and unknown bits. Original
point risks are retained even if the caller supplies an empty risk grid.
Native occupied cell rectangles are conservatively projected into the terrain
grid, including cells without point support. Semantic ground overlap can explain
where coverage is lost, but cannot authorize clearing UNKNOWN. Inputs without
independent risk coverage remain geometry-only and cannot emit `CANDIDATE`.
Native cells omitted from the sparse space map remain unknown, even inside its
bounding ROI. Risk coverage requires explicit current-cell evidence over the
whole destination cell; it is not verified ray visibility.

`terrain_traversability_experiment.py` exports the frozen real-input comparison.
Provide an explicit limits JSON, for example:

```json
{
  "diagnostic_only": true,
  "max_slope_gradient": 0.30,
  "max_roughness_m": 0.10,
  "max_step_m": 0.20,
  "max_depression_depth_m": 0.20
}
```

These are illustrative experimental limits, not calibrated vehicle capabilities.
Run in the managed test runtime with the original datasets visible:

```sh
python3 modules/tools/perception_diagnostics/terrain_traversability_experiment.py \
  --root /apollo --limits /path/to/limits.json --output /path/to/new-results
```

Per-frame NPZ outputs retain identities, source fingerprints, states, reason
memberships and both connected masks. Reports distinguish cached native-risk
inputs from geometry-only inputs and semantic-ground proxy evaluation from
physical traversability truth. This diagnostic path emits no FREE, performs no
vehicle-footprint or verified ray-visibility qualification, and does not modify
production fusion or planning.

### BEV and occupancy debug images

Export the audited terrain comparison to the ignored local `data/` directory:

```sh
python3 modules/tools/perception_diagnostics/terrain_debug.py \
  --report .cache/lidar-fusion-validation/terrain-v6/report.json \
  --output data/lidar_fusion_debug/python
```

Open `data/lidar_fusion_debug/python/index.html` locally. Each frame has six PNG
views: raw maximum-height BEV, independent source occupancy, terrain state,
risk-constrained decision, exact-root connectivity and explicit risk coverage.
`manifest.json` retains archive/image hashes, coordinate bounds and the source
bit palette. No model inference or terrain fit is rerun by this exporter.

Images use **+x up and +y left**, in frame-local metres. White crosses mark
observed roots; blue crosses mark the frame origin, not a calibrated vehicle
footprint. Yellow, red and violet occupancy represent measured, object and
UNKNOWN sources; distinct mixed colors retain overlapping source bits.
Dark missing evidence is not empty or FREE. Root/origin crosses are display
annotations, not modifications of the numerical archives. The raw height
palette clips display colors to -3 through +5 metres; original heights remain
unchanged.

`terrain_native_validation.py --binary PATH --output data/lidar_fusion_debug/cpp`
feeds original XYZ/IDs and explicit config/risk to the C++ terrain CLI, not
Python-fitted features. It compares selected roots, raw layers/source IDs,
point labels, cell measurements, states, reasons and connected masks against
the frozen reference. Native `bev/index.html` contains actual C++ outputs;
`native-validation.json` retains binary fingerprints, differences and
descriptive origin-centered coverage windows. A successful replay process is
not a parity pass when the comparison report contains differences.
The merged module README describes the numerical libraries and separate
official Bazel validation blocker.

## Terrain, clearance and trajectory support (offline only)

The priority table, directory structure, architecture and promotion boundaries
are in [the fusion module README](../../lidar_fusion/README.md#spatial-capability-delivery-priorities).
`spatial_support_experiment.py` extends the frozen ground comparison with
observed local terrain, independent risk-layer distances and explicit
diagnostic polyline footprint queries. Observed ego-connected ground masks are
joined from the exact frozen ground artifacts; disconnected islands remain
unknown, not usable terrain. It does not modify production fusion,
enable FREE, run a planner or issue control commands.

Provide a JSON with `diagnostic_only: true`, `ground_results` pointing to the
ground experiment output, `schema`, `diagnostic_max_step_m`, explicit
`vehicle: {length, width, margin}` in metres, and `paths` entries with `name`,
`kind: "diagnostic_probe"` and `xy` base-frame metre waypoints. These are
illustrative probes, not real vehicle trajectories. Run in the isolated test
runtime:

```sh
python3 modules/tools/perception_diagnostics/spatial_support_experiment.py \
  --cases /path/to/spatial-experiment.json --output /path/to/new-spatial-results
```

The exporter retains baseline/candidate `.spatial.npz` layers, source hashes,
same-frame path sample distances and all unknown/risk reasons in `report.json`.
Unsupported terrain and missing/approximate visibility cannot be reported
as safe. Candidate measured occupancy is hypothetical; native inferred/unknown
layers remain unchanged. Sparse obstacle filtering is deliberately excluded.

P2 also exports `cost_*` arrays: independent obstacle sources, terrain ratios,
unsupported/visibility masks and diagnostic admissibility (never a weighted
planner decision). Cost ratios, terrain-limit masks and admissibility use the
same explicit positive, finite thresholds without changing the scene's original
terrain masks. Layer diagnostics report individual, sole and overlapping
blockers; unverified visibility remains a blocker, not an accuracy shortcut.
`directional_paths` contains rectangle-versus-cell queries
with segment heading, translation sampling inflation and conservative corner
rotation envelopes only at heading changes. Duplicate and same-direction
collinear waypoints do not introduce artificial turns. A single-pose path
requires explicit `yaw`; optional fixed
`yaw` on a polyline is a diagnostic orientation, not a steering trajectory.
Actual vehicle-center offsets, steering feasibility and planner adaptation
still require qualification. The full design is canonical in
[`ad_agents/context`](../../../ad_agents/context/modules/perception/knowledge/lidar-fusion-driving-capabilities-design.md).

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
