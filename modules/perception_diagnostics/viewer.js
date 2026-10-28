// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
"use strict";
(() => {
  const data = window.DATA, byId = id => document.getElementById(id);
  const canvas = byId("bev"), ctx = canvas.getContext("2d");
  const palette = ["#94a3b8", "#38bdf8", "#a3e635", "#86efac", "#c084fc", "#e879f9", "#facc15", "#f87171"];
  let index = 0, timer = null, hits = [], scale = 1, width = 0, height = 0;
  const enabled = id => byId(id).checked;
  const frame = () => data.frames[index];
  const xy = (x, y) => [width / 2 - y * scale, height / 2 - x * scale];
  const text = (id, value) => { byId(id).textContent = typeof value === "string" ? value : JSON.stringify(value, null, 2); };
  if (data.title) { text("page-title", data.title); document.title = data.title; }
  if (data.banner) text("banner", data.banner);
  if (data.semantic_ground_truth) {
    const option = document.createElement("option");
    option.value = "11"; option.textContent = "Mapped semantic GT";
    byId("color").appendChild(option);
  }
  function polygon(vertices, color, fill = false) {
    if (!vertices.length) return;
    ctx.beginPath();
    vertices.forEach((v, i) => { const p = xy(v[0], v[1]); if (i) ctx.lineTo(...p); else ctx.moveTo(...p); });
    ctx.closePath(); ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.stroke();
    if (fill) { ctx.fillStyle = color + "20"; ctx.fill(); }
  }
  function drawBox(b, color, layer) {
    const [x, y] = b.center, [l, w] = b.size;
    const c = Math.cos(b.yaw), s = Math.sin(b.yaw);
    polygon([[l/2,w/2], [l/2,-w/2], [-l/2,-w/2], [-l/2,w/2]].map(
      ([a, d]) => [x + a*c - d*s, y + a*s + d*c]), color);
    const p = xy(x, y), nose = xy(x + l/2*c, y + l/2*s);
    ctx.beginPath(); ctx.moveTo(...p); ctx.lineTo(...nose); ctx.stroke();
    ctx.fillStyle = color; ctx.font = "10px sans-serif";
    ctx.fillText(`${b.class}${b.id !== undefined ? ":" + b.id : ""}`, p[0] + 3, p[1] - 3);
    hits.push({p, value: {layer, ...b}});
  }
  function accept(p) {
    const filter = byId("filter").value;
    return filter === "all" || (filter === "ground" && p[5] === 1) ||
      (filter === "non_ground" && ![1, 2, 5, 6, 7].includes(p[5])) ||
      (filter === "object" && [3, 4].includes(p[5])) ||
      (filter === "trusted" && p[12] === 1) ||
      (filter === "unknown" && [0, 4].includes(p[5]));
  }
  function draw() {
    const rect = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
    width = rect.width; height = rect.height;
    canvas.width = width * ratio; canvas.height = height * ratio;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.fillStyle = "#18212d"; ctx.fillRect(0, 0, width, height);
    scale = Math.min(width, height) / Number(byId("zoom").value);
    const f = frame(); hits = [];
    for (const cell of f.cells) {
      let color = null;
      if (enabled("visibility") && cell[7]) color = "#33445b";
      if (enabled("ground") && cell[6]) color = "#365144";
      if (enabled("cells") && cell[2]) color = "#e2e8f060";
      if (enabled("inferred") && cell[3]) color = "#38bdf850";
      if (enabled("risk") && cell[4]) color = "#fbbf2450";
      if (color) {
        const r = f.grid.resolution, p = xy(f.grid.x + (cell[0] + 1)*r, f.grid.y + (cell[1] + 1)*r);
        ctx.fillStyle = color; ctx.fillRect(p[0], p[1], r*scale + 0.3, r*scale + 0.3);
      }
    }
    ctx.strokeStyle = "#64748b50"; ctx.lineWidth = 0.5;
    for (let m = -60; m <= 60; m += 10) {
      ctx.beginPath(); ctx.moveTo(...xy(m, -60)); ctx.lineTo(...xy(m, 60)); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(...xy(-60, m)); ctx.lineTo(...xy(60, m)); ctx.stroke();
      const p = xy(m, 0); ctx.fillStyle = "#94a3b8"; ctx.fillText(`${m}m`, p[0]+3, p[1]-2);
    }
    if (enabled("points")) for (const p of f.points) {
      if (!accept(p)) continue;
      const mode = Number(byId("color").value), pos = xy(p[0], p[1]);
      ctx.fillStyle = mode === 2 ? `hsl(${Math.max(0, Math.min(240, 180-p[2]*30))},80%,65%)` : palette[p[mode] % palette.length];
      ctx.fillRect(pos[0], pos[1], 1.6, 1.6);
    }
    if (enabled("truth")) for (const b of f.truth) drawBox(b, "#4ade80", "ground_truth");
    if (enabled("raw")) for (const b of f.raw) drawBox(b, "#60a5fa", "raw_detector");
    if (enabled("objects")) for (const b of f.objects) {
      if (enabled("classified") && b.classification_use !== 1) continue;
      drawBox(b, b.conflict ? "#fb7185" : "#22d3ee", "fused");
    }
    if (enabled("unknowns")) for (const u of f.unknowns) {
      polygon(u.polygon, "#fbbf24", !u.point_supported_footprint);
      if (u.polygon.length) {
        const center = u.polygon.reduce((s, p) => [s[0]+p[0]/u.polygon.length, s[1]+p[1]/u.polygon.length], [0,0]);
        hits.push({p: xy(...center), value: {layer: "unknown", ...u}});
      }
    }
    const origin = xy(0, 0);
    ctx.fillStyle = "white"; ctx.beginPath(); ctx.arc(...origin, 4, 0, 2*Math.PI); ctx.fill();
    ctx.fillText("ego +X", origin[0]+5, origin[1]-12);
    text("title", `${index + 1}/${data.frames.length} ${f.scene} ${f.file} t=${f.stamp.toFixed(3)}`);
    byId("frame").value = index;
    if (f.camera) byId("camera").src = f.camera;
    else byId("camera").removeAttribute("src");
    byId("camera").hidden = !f.camera;
    text("skew", f.camera ? `CAM_FRONT skew ${f.camera_skew === null ? "unknown" : f.camera_skew.toFixed(4) + " s"}` : "No camera in this export");
    text("stats", {sample_token: f.sample_token, display_points: f.points.length,
      original_points: f.original_points, display_stride: f.stride,
      detection_model_revision: f.detection_model_revision, ...f.audit, accuracy: f.accuracy});
    text("labels", {group: data.labels.Group, explanation: data.labels.Explanation,
      mapped_semantic: data.semantic_labels});
  }
  function move(delta) { index = (index + delta + data.frames.length) % data.frames.length; draw(); }
  byId("frame").max = data.frames.length - 1;
  byId("frame").addEventListener("input", e => { index = Number(e.target.value); draw(); });
  byId("prev").onclick = () => move(-1);
  byId("next").onclick = () => move(1);
  byId("play").onclick = () => {
    if (timer) { clearInterval(timer); timer = null; text("play", "Play"); }
    else { timer = setInterval(() => move(1), 800); text("play", "Pause"); }
  };
  for (const el of document.querySelectorAll('input:not(#frame),select')) el.addEventListener("input", draw);
  window.addEventListener("resize", draw);
  window.addEventListener("keydown", e => { if (e.key === "ArrowRight") move(1); if (e.key === "ArrowLeft") move(-1); });
  canvas.onclick = e => {
    const rect = canvas.getBoundingClientRect(), at = [e.clientX-rect.left, e.clientY-rect.top];
    let best = null, distance = 15;
    for (const hit of hits) {
      const d = Math.hypot(hit.p[0]-at[0], hit.p[1]-at[1]);
      if (d < distance) { best = hit.value; distance = d; }
    }
    if (!best) for (const p of frame().points) {
      if (!enabled("points") || !accept(p)) continue;
      const pos = xy(p[0], p[1]), d = Math.hypot(pos[0]-at[0], pos[1]-at[1]);
      if (d < distance) {
        distance = d; best = {point_id: p[6], xyz: p.slice(0,3), raw_semantic: p[3],
          group: data.labels.Group[p[4]], explanation: data.labels.Explanation[p[5]],
          semantic_probability: p[7], object_id: p[8], source: data.labels.Source[p[10]],
          reason: p[9].map(r => data.labels.Reason[r]),
          mapped_semantic_truth: p[11], visible_instance_id: p[12]};
      }
    }
    text("detail", best || "No displayed evidence near this click.");
  };
  draw();
})();
