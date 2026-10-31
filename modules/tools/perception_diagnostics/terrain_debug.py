#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Export inspectable BEV images from identity-audited terrain archives."""

import argparse
import hashlib
import html
import json
import struct
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BACKGROUND = (24, 28, 36)
RISK_COLORS = (
    (55, 100, 70),
    (240, 185, 45),
    (240, 65, 65),
    (255, 130, 45),
    (155, 85, 220),
    (215, 155, 230),
    (235, 90, 175),
    (250, 235, 250),
)
PANELS = (
    ("bev", "Raw BEV: maximum observed z per cell; blue -3 m to red +5 m"),
    ("occupancy", "Independent occupancy: measured/object/UNKNOWN bits retained"),
    ("terrain", "Terrain: gray unknown, blue support, green passable, red blocked"),
    ("decision", "Risk-constrained: gray unknown, green candidate, red blocked"),
    ("connectivity", "Root components: dark green geometry, bright cyan constrained"),
    ("coverage", "Risk evidence: blue full cell coverage, gray missing/partial"),
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _chunk(kind, data):
    return (
        struct.pack(">I", len(data))
        + kind
        + data
        + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    )


def png_bytes(rgb):
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError("PNG input must be a uint8 HxWx3 image")
    height, width, _ = rgb.shape
    if not height or not width:
        raise ValueError("PNG dimensions must be positive")
    rows = b"".join(b"\0" + row.tobytes() for row in rgb)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(rows))
        + _chunk(b"IEND", b"")
    )


def bev_image(grid_rgb, scale=3):
    """Rows are +x up; columns are +y left. No transpose or mirrored labels."""
    if not isinstance(scale, int) or not 1 <= scale <= 8:
        raise ValueError("image scale must be an integer between 1 and 8")
    image = np.asarray(grid_rgb, dtype=np.uint8)[::-1, ::-1]
    return np.repeat(np.repeat(image, scale, axis=0), scale, axis=1)


def _blank(shape):
    image = np.empty((*shape, 3), dtype=np.uint8)
    image[:] = BACKGROUND
    return image


def _marker(image, cell, color):
    nx, ny = image.shape[:2]
    ix, iy = map(int, cell)
    if not (0 <= ix < nx and 0 <= iy < ny):
        return
    for dx, dy in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
        xx, yy = ix + dx, iy + dy
        if 0 <= xx < nx and 0 <= yy < ny:
            image[xx, yy] = color


def render_panels(arrays, config):
    shape = arrays["terrain_state"].shape
    if len(shape) != 2:
        raise ValueError("terrain grid must be two dimensional")
    for key in (
        "risk_layers",
        "risk_observed",
        "decision_state",
        "terrain_connected",
        "risk_constrained_connected",
    ):
        if arrays[key].shape != shape:
            raise ValueError(f"inconsistent grid shape: {key}")
    bits = arrays["risk_layers"]
    if bits.dtype.kind not in "iu" or np.any(bits > 7):
        raise ValueError("invalid occupancy source bits")
    panels = {key: _blank(shape) for key, _ in PANELS}
    occupied = bits != 0
    observed = arrays["risk_observed"]
    panels["occupancy"][occupied | observed] = np.asarray(RISK_COLORS, dtype=np.uint8)[
        bits[occupied | observed]
    ]
    for state, color in (
        ("GROUND_SUPPORTED", (65, 110, 180)),
        ("TERRAIN_PASSABLE", (65, 185, 110)),
        ("BLOCKED", (235, 65, 65)),
    ):
        panels["terrain"][arrays["terrain_state"] == state] = color
    panels["decision"][arrays["decision_state"] == "CANDIDATE"] = (65, 185, 110)
    panels["decision"][arrays["decision_state"] == "BLOCKED"] = (235, 65, 65)
    panels["connectivity"][arrays["terrain_connected"]] = (50, 130, 80)
    panels["connectivity"][arrays["risk_constrained_connected"]] = (45, 235, 230)
    panels["coverage"][observed] = (65, 150, 230)

    xyz = arrays["xyz_m"]
    cells = arrays["point_cell_id"]
    if xyz.shape != (len(cells), 3):
        raise ValueError("point/cell identity shape mismatch")
    valid = (cells >= 0) & (cells < np.prod(shape)) & np.isfinite(xyz).all(axis=1)
    heights = np.full(np.prod(shape), -np.inf)
    np.maximum.at(heights, cells[valid], xyz[valid, 2])
    heights = heights.reshape(shape)
    measured = np.isfinite(heights)
    fraction = np.clip((heights[measured] + 3.0) / 8.0, 0, 1)
    panels["bev"][measured] = np.column_stack(
        (
            40 + 205 * fraction,
            100 + 100 * (1 - np.abs(2 * fraction - 1)),
            245 - 205 * fraction,
        )
    ).astype(np.uint8)
    origin_cell = np.floor(
        -np.array([config["x_min"], config["y_min"]]) / config["cell_size"]
    ).astype(int)
    root = arrays["root_cell"]
    for image in panels.values():
        _marker(image, origin_cell, (30, 140, 255))
        _marker(image, root, (255, 255, 255))
    return panels


def _configs(root):
    report = json.loads(
        (
            root / ".cache/lidar-fusion-validation/terrain-v4/real-data-report.json"
        ).read_text()
    )
    entries = report["cached_frames"] + report["raw_non08_frames"]
    return {
        "frames/"
        + str(entry["artifact_path"])
        .split("/frames/", 1)[1]
        .replace(".terrain.npz", ".traversability.npz"): entry["terrain_config"]
        for entry in entries
    }


def _frame_directory(output, frame):
    identity = Path(frame["frame_id"])
    group = Path(frame["dataset_group"])
    if (
        identity.is_absolute()
        or group.is_absolute()
        or ".." in identity.parts
        or len(group.parts) != 1
        or ".." in group.parts
    ):
        raise ValueError("frame identity must remain inside the output directory")
    return output / group / identity


def export_debug(report_path, output, *, root=ROOT, scale=3):
    report_path, output, root = map(Path, (report_path, output, root))
    report_hash = digest(report_path)
    report = json.loads(report_path.read_text())
    configs = _configs(root)
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    for frame in report["frames"]:
        source = report_path.parent / frame["artifact_path"]
        if digest(source) != frame["artifact_sha256"]:
            raise ValueError(f"changed input archive: {source}")
        config = configs[frame["artifact_path"]]
        with np.load(source, allow_pickle=False) as stored:
            arrays = {key: stored[key] for key in stored.files}
        images = render_panels(arrays, config)
        directory = _frame_directory(output, frame)
        directory.mkdir(parents=True, exist_ok=True)
        links, hashes, cards = {}, {}, []
        for name, caption in PANELS:
            path = directory / f"{name}.png"
            path.write_bytes(png_bytes(bev_image(images[name], scale)))
            links[name] = str(path.relative_to(output))
            hashes[name] = digest(path)
            cards.append(
                f"<figure><figcaption>{html.escape(caption)}</figcaption>"
                f'<img src="{name}.png" alt="{name}"></figure>'
            )
        title = html.escape(frame["frame_id"])
        axis = (
            f"+x up ({config['x_min']} to {config['x_max']} m); "
            f"+y left ({config['y_min']} to {config['y_max']} m); "
            f"cell {config['cell_size']} m. White cross: observed root; "
            "blue cross: frame origin, not calibrated vehicle footprint."
        )
        page = (
            "<!doctype html><meta charset='utf-8'>"
            "<style>body{background:#181c24;color:#eee;font:16px sans-serif}"
            ".panels{display:flex;flex-wrap:wrap}figure{margin:8px}"
            "img{max-width:100%;image-rendering:pixelated}</style>"
            f"<h1>{title}</h1><p>{html.escape(axis)}</p>"
            "<p>Diagnostic only. UNKNOWN is not free space. Geometry and risk "
            "layers remain independent. No vehicle clearance qualification.</p>"
            f"<p>{html.escape(json.dumps(frame['metrics'], sort_keys=True))}</p>"
            f"<div class='panels'>{''.join(cards)}</div>"
        )
        (directory / "index.html").write_text(page)
        rows.append(
            {
                "frame_id": frame["frame_id"],
                "dataset_group": frame["dataset_group"],
                "input_archive": str(source),
                "input_sha256": frame["artifact_sha256"],
                "raw_provenance": frame["input_provenance"],
                "coordinate_config": config,
                "point_count": len(arrays["point_id"]),
                "root_cell": arrays["root_cell"].tolist(),
                "images": links,
                "image_sha256": hashes,
                "page": str((directory / "index.html").relative_to(output)),
            }
        )
    if digest(report_path) != report_hash:
        raise ValueError("report changed during image export")
    manifest = {
        "diagnostic_only": True,
        "report_path": str(report_path),
        "report_sha256": report_hash,
        "exporter_sha256": digest(__file__),
        "frame_count": len(rows),
        "PNG_count": len(rows) * len(PANELS),
        "axis": "+x up, +y left; metres; original frame-local coordinates",
        "occupancy_palette": {
            str(bits): {"RGB": color, "source_bits": bits}
            for bits, color in enumerate(RISK_COLORS)
        },
        "risk_bits": {"measured": 1, "object": 2, "UNKNOWN": 4},
        "missing_evidence_RGB": BACKGROUND,
        "frames": rows,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n"
    )
    links = "".join(
        f"<li><a href='{html.escape(row['page'], quote=True)}'>"
        f"{html.escape(row['frame_id'])}</a> ({row['dataset_group']})</li>"
        for row in rows
    )
    (output / "index.html").write_text(
        "<!doctype html><meta charset='utf-8'><title>LiDAR fusion debug BEV</title>"
        "<h1>LiDAR fusion debug BEV</h1><p>Diagnostic only; not FREE or safe passage. "
        "Open a frame for raw BEV, source occupancy, terrain, risk decisions, "
        "connectivity and evidence coverage. +x up, +y left.</p>"
        "<p>Occupancy: yellow=measured, red=object, violet=UNKNOWN; orange/pink/"
        "white=overlapping bits. Dark background=missing coverage, "
        f"muted green=assessed empty risk, not free space.</p><ul>{links}</ul>"
    )
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report",
        type=Path,
        default=ROOT / ".cache/lidar-fusion-validation/terrain-v6/report.json",
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "data/lidar_fusion_debug/python"
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--scale", type=int, default=3)
    args = parser.parse_args()
    result = export_debug(args.report, args.output, root=args.root, scale=args.scale)
    print(
        f"Exported {result['PNG_count']} PNG images for {result['frame_count']} frames"
    )


if __name__ == "__main__":
    main()
