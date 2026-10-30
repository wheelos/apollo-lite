// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");
const vm = require("node:vm");
const source = fs.readFileSync(path.join(__dirname, "viewer.js"), "utf8");

function viewer(objects = [], points = [], raw = [], partitions = [], flags = []) {
  const calls = [], elements = new Map();
  const ctx = new Proxy({}, {
    get: (_, name) => (...args) => calls.push([name, ...args]),
    set: (_, name, value) => { calls.push([name, value]); return true; }
  });
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      checked: ["objects", "points", ...flags].includes(id),
      value: id === "zoom" ? "100" : id === "filter" ? "all" : "4",
      textContent: "", addEventListener() {}, removeAttribute() {}, appendChild() {},
      getBoundingClientRect: () => ({width: 400, height: 400, left: 0, top: 0}),
      getContext: () => ctx
    });
    return elements.get(id);
  };
  const frame = {
    cells: [], points, objects, raw, shadow_partitions: partitions,
    truth: [], unknowns: [], stamp: 1,
    scene: "fixture", file: "fixture.pb", grid: {resolution: 0.25, x: -20, y: -30}
  };
  const DATA = {frames: [frame], labels: {
    Group: {0: "GENERIC_OBS", 1: "THING"}, Explanation: {0: "UNEXPLAINED"},
    UnifiedClass: {0: "CLASS_UNKNOWN", 1: "CAR", 4: "PEDESTRIAN"},
    Source: {0: "GEOMETRY"}, Reason: {}
  }};
  vm.runInNewContext(source, {
    window: {DATA, devicePixelRatio: 1, addEventListener() {}},
    document: {getElementById: element, querySelectorAll: () => [], createElement: () => ({})},
    setInterval() {}, clearInterval() {}
  });
  return {
    calls,
    click(x, y) {
      element("bev").onclick({clientX: x, clientY: y});
      return JSON.parse(element("detail").textContent);
    }
  };
}

function object(orientation_reliable) {
  return {
    id: 7, class: "unknown", center: [2, 0, 0], size: [4, 2, 1], yaw: 0,
    orientation_reliable, score: null, ranking_score: 1,
    evidence: {point_id: [3, 5]}
  };
}

test("heading marks require explicit orientation reliability", () => {
  const lines = value => viewer([value]).calls.filter(c => c[0] === "lineTo").length;
  assert.equal(lines(object(true)), lines(object(false)) + 1);
  assert.equal(lines(object(undefined)), lines(object(false)));
});

test("point inspection keeps trusted and GT instance identity separate", () => {
  const point = [2, 0, 0, 1, 1, 0, 77, 0.95, null, [], 0, 1, 1, 325,
    {raw_semantic_label: 1, semantic_probability: 0.95, unified_class: "CAR"}, {}];
  const detail = viewer([], [point]).click(200, 192);
  assert.equal(detail.visible_instance_id, 325);
  assert.equal(detail.trusted, true);
  assert.equal(detail.point_id, 77);
  assert.equal(detail.evidence.unified_class, "CAR");
  const noTruth = [...point];
  noTruth[13] = null;
  assert.equal(viewer([], [noTruth]).click(200, 192).visible_instance_id, null);
});

test("absent semantic fields remain unavailable in inspection", () => {
  const point = [2, 0, 0, 0, 0, 0, 77, 0, null, [], 0, null, 0, null, {}, {}];
  const detail = viewer([], [point]).click(200, 192);
  assert.equal(detail.raw_semantic, null);
  assert.equal(detail.semantic_probability, null);
});

test("object inspection retains absent score and linked native hypotheses", () => {
  const raw = [{evidence: {detection_index: 3, object_id: 7,
    existence_supported: true, association: {n_core: 6}}}];
  const detail = viewer([object(false)], [], raw).click(200, 192);
  assert.equal(detail.score, null);
  assert.equal(detail.ranking_score, 1);
  assert.deepEqual(detail.evidence.point_id, [3, 5]);
  assert.equal(detail.hypotheses[0].association.n_core, 6);
  assert.equal(detail.cross_thing_disagreement, null);
});

test("cross-THING evidence is visible without changing the consumer role", () => {
  const semantic = {identity_qualified: true, source_available: true,
    cross_thing_disagreement: true,
    thing_support: [{unified_class: "PEDESTRIAN", point_id: [77, 78]}]};
  const raw = [{evidence: {detection_index: 3, object_id: 7, unified_class: "CAR",
    association: {n_core: 6, semantic_evidence: semantic}}}];
  const item = {...object(false), class: "car", classification_use: 1};
  const result = viewer([item], [], raw);
  const detail = result.click(200, 192);
  assert.equal(detail.cross_thing_disagreement, true);
  assert.equal(detail.class, "car");
  assert.equal(detail.classification_use, 1);
  assert.ok(result.calls.some(c => c[0] === "strokeStyle" && c[1] === "#c084fc"));
  const point = [10, 0, 0, 2, 1, 0, 77, 0.95, null, [], 0];
  const support = viewer([], [point], raw, [], ["alternatives"]).click(200, 160);
  assert.equal(support.layer, "alternative_thing_support");
  assert.equal(support.detection_class, "CAR");
  assert.equal(support.unified_class, "PEDESTRIAN");
  assert.equal(support.displayed_support_points, 1);
});

test("shadow overlays retain parent and full IDs without inventing geometry", () => {
  const point = [2, 0, 0, 1, 1, 0, 77, 0.95, null, [], 0];
  const partition = {id: 4, parent_unknown_id: 71, kind: "PARTITION_THING_SUPPORT",
    group: "THING", unified_class: "CAR", point_id: [77, 78, 79]};
  const result = viewer([], [point], [], [partition], ["partitions"]);
  const detail = result.click(200, 192);
  assert.equal(detail.layer, "shadow_partition");
  assert.equal(detail.parent_unknown_id, 71);
  assert.deepEqual(detail.point_id, [77, 78, 79]);
  assert.equal(detail.displayed_support_points, 1);
  assert.equal(detail.box, undefined);
  assert.equal(detail.orientation_reliable, undefined);
});
