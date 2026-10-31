# Annotation-injected OpenLane diagnostic

This is **not detector accuracy, model execution, pixel inference, lane–light
relation accuracy, closed-loop validation, or a performance benchmark**.
The exporter audits local JSON and JPEG header dimensions; replay injects
annotation boxes and colors through `StaticDetectorProviderPort` after prompting.
It calls the same `CreateTrafficLightPipeline` factory as the component:
Predict → Prompter → Detector → Tracker → Binder → Fusion (heuristics disabled).
No map, lane context, topology, or signal IDs are injected. Every frame asserts
that `bound_lights` and `final_lights` remain empty despite visual observations.
Static injection emits all annotation boxes regardless of prompts: coverage
measures prompt proposals, not measured crop-detector recall or inference cost.

## Reproduce the local sample

From the repository root, export on the host (Python standard library only):

```bash
mkdir -p .cache
python3 modules/traffic_light/tools/export_openlane.py \
  /mnt/synology/dataset/OpenLane-V2_sample \
  --local-prefix-diagnostic --exclude-ambiguous-ids --clip-partial-boxes \
  --output .cache/openlane-session.tsv
bash docker/scripts/whl.sh exec dev -- bash -lc \
  'bazel run //modules/traffic_light:openlane_replay -- /apollo/.cache/openlane-session.tsv'
```

The binary requires `tools/openlane_replay.cc` and dependencies
`:tl_pipeline_assembly` (header-only shared factory), `:tl_stages`, `:tl_pipeline`,
and `:tl_ports`; no JSON dependency. It must not depend on the always-linked
`:tl_component`: that pulls Cyber component static registration into this
standalone executable and caused a startup crash during glog initialization.
The dataset need not be mounted inside the container: only exported TSV is read.
Use `--verify-images` only if Pillow is already available; it verifies then
reopens and fully decodes every image, reporting `decoded_images` in the audit,
without supplying pixels to replay. No package is installed.

The frozen local sample has 32 frames each in `00492` and `00556`, front camera
1550×2048, approximately 0.5-second intervals, 471 `tl_*` observations. The
exporter's stderr audit includes category/attribute counts and the SHA-256 of
the exact TSV. Two explicit data-quality policies are needed for all 64 frames:

* `--exclude-ambiguous-ids`: two `00492` frames, timestamps
  `315970277249927220` and `315970277749927214`, each repeat `tl_06`.
  All four associated boxes are retained for injection/coverage but excluded
  from identity matching. No replacement persistent IDs are invented.
* `--clip-partial-boxes`: nine boxes extend outside the image (one by floating
  roundoff, eight by about 1.15 pixels). Clip to the visible image intersection,
  audit the count; still reject nonfinite, reversed, degenerate, or entirely
  off-image boxes. Without these flags the respective defects are rejected.

The declared version must be exactly `OpenLaneV2_V0.1`. Local categories conflict
with expected semantic use: `tl_*` occurs in both categories 0 and 1, and `rs_*`
also occurs in category 0. Therefore no automatic/official category mapping is
offered. `--local-prefix-diagnostic` explicitly selects local `tl_*` only.
The accepted local tuples are `tl:0:{0,4}`, `tl:1:{1,2,3}`, `rs:0:{4,5,6}`;
other mappings fail. Attribute 1→red, 2→green, 3→yellow, otherwise unknown.
In particular mixed attribute 4 is **not arrow evidence**: all injected shapes
remain UNKNOWN. Objectness and semantic confidence are fixed at 0.9.
Topology matrix sizes and binary entries are checked; lane–element edge counts
are audited only, never supplied to Binder or scored as correctness.

## TSV contract and reported quantities

UTF-8, tab-separated, no quoting, blank lines, or optional columns:

```text
openlane-replay-v1<TAB>OpenLaneV2_V0.1<TAB>local-tl-prefix-diagnostic
F<TAB>sequence<TAB>ring_front_center<TAB>timestamp_ns<TAB>width<TAB>height<TAB>N
G<TAB>evaluation_id<TAB>x<TAB>y<TAB>width<TAB>height<TAB>color
... exactly N G rows, then the next F row
```

Box coordinates are image pixels, `(x,y)` is top-left, width/height positive.
Color is `red`, `green`, `yellow`, or `unknown`. Evaluation ID is a unique
per-frame `tl_*` ID or `-` for explicitly excluded ambiguous identity; IDs are
never copied into neural candidates' signal IDs. Sequence frames must be
contiguous, timestamp-monotonic, and dimension-consistent. A fresh pipeline,
identity history, and static baseline are created for each sequence.
Nanosecond timestamps are passed through unchanged, so the tracker uses real dt.
Malformed data or failed invariants exit nonzero without success JSON.

JSON stdout reports observations, confirmed observed-track matches, identity
switches, prompt coverage/area, exploration counts, and the empty-output checks.
Identity association is deterministic greedy descending IoU, one-to-one,
IoU≥0.5; ties use track and annotation order. A switch means a GT ID's current
matched track differs from its last matched track within that sequence, including
gaps. Lost-track predictions are not identity matches. Four ambiguous-ID boxes
are included in observations but reported separately from identity eligibility.
These are annotation-injected mechanism diagnostics, not general MOT results.

History coverage is the count of current boxes fully contained in any HISTORY
prompt proposal. These are **all proposals**, not the budget-selected regions
returned by `BuildInferenceRegions`. Periodic full-frame exploration is excluded
from both history coverage and `history_area_px2_sum`; consequently that area
is neither total inference area nor a compute/cost measurement. The baseline uses each
forecast track's last actual visual observation, held stationary, with the
same forecast uncertainty margin (2σ per side) and history expansion ratio
(default 1.4) as the predictive prompter, clipped to image bounds.
Coverage and summed ROI areas are reported separately: areas count overlap
multiple times, are not a union, and are not forced equal after clipping and
prompt deduplication. No improvement/pass threshold is asserted. Newly observed
tracks and label colors are scored only after pipeline decisions; annotation
labels do provide the explicitly disclosed injected detector observations.

The integrated run on 2026-10-28 with global two-stage assignment and
center/size prediction reported 64 frames, 471 observations, 369 IoU matches,
and **49 identity switches**. Predictive-history proposals covered 428 boxes
versus 400 for the static baseline, with 640 proposals each and summed areas
9.38248e6 versus 9.38699e6 pixels². The input TSV SHA-256 was
`ad97d6b4cfe19d59ce21f4eaad922c48709d2972851a7ce5815d587665ffe89a`.
These figures describe that run only; rerun after tracker changes.
The 49 switches are not hidden by the
coverage comparison: production tracking quality is **not validated**.

Run the module's C++ contracts and six hermetic exporter regressions in the
managed container; run C++ lint separately during review:

```bash
bash docker/scripts/whl.sh exec dev -- bash -lc \
  'cd /apollo && bazel test //modules/traffic_light:all --test_output=errors'
```

Flow-level and simulated-camera/V2X Cyber integration scenarios live in the
independent `ad_agents/integration_tests/traffic_light/` package, rather than
the module's unit-test target.
