#!/usr/bin/env python3
"""Export the local OpenLane-V2 V0.1 sample; never infer official semantics."""

import argparse
import collections
import hashlib
import io
import json
import math
import pathlib
import struct
import sys


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def dimensions(path):
    """Read JPEG SOF dimensions without decoding pixels or requiring Pillow."""
    with path.open("rb") as stream:
        require(stream.read(2) == b"\xff\xd8", f"not JPEG: {path}")
        while True:
            require(stream.read(1) == b"\xff", f"invalid JPEG marker: {path}")
            marker = stream.read(1)
            while marker == b"\xff":
                marker = stream.read(1)
            require(len(marker) == 1, f"truncated JPEG: {path}")
            size = stream.read(2)
            require(len(size) == 2, f"truncated JPEG segment: {path}")
            size = struct.unpack(">H", size)[0]
            require(size >= 2, f"invalid JPEG segment: {path}")
            payload = stream.read(size - 2)
            require(len(payload) == size - 2, f"truncated JPEG: {path}")
            if marker[0] in (0xC0, 0xC1, 0xC2):
                require(len(payload) >= 5, f"invalid JPEG SOF: {path}")
                height, width = struct.unpack(">HH", payload[1:5])
                require(width > 0 and height > 0, "nonpositive dimensions")
                return width, height


def export(args):
    require(
        args.local_prefix_diagnostic,
        "category/id mismatch: explicitly opt "
        "in with --local-prefix-diagnostic; not official category semantics",
    )
    root = args.root.resolve()
    out = io.StringIO()
    out.write("openlane-replay-v1\tOpenLaneV2_V0.1\tlocal-tl-prefix-diagnostic\n")
    audit = {
        "frames": 0,
        "observations": 0,
        "topology_lcte_edges": 0,
        "category_attribute_counts": {},
        "image_pixels_used": False,
        "ambiguous_identity_observations": 0,
        "clipped_boxes": 0,
        "decoded_images": 0,
    }
    for segment in ("00492", "00556"):
        paths = sorted((root / "train" / segment / "info").glob("*.json"))
        require(paths, f"no annotation frames: {segment}")
        previous = -1
        expected_dimensions = None
        for path in paths:
            data = json.loads(path.read_text())
            require(data["version"] == "OpenLaneV2_V0.1", "unsupported version")
            require(data["segment_id"] == segment, "segment mismatch")
            timestamp = data["timestamp"]
            require(
                type(timestamp) is int and previous < timestamp < 2**64,
                "invalid/nonmonotonic nanosecond timestamp",
            )
            require(path.stem == str(timestamp), "filename/timestamp mismatch")
            previous = timestamp
            image = (root / data["sensor"]["ring_front_center"]["image_path"]).resolve()
            require(image.is_relative_to(root), "image outside dataset root")
            width, height = dimensions(image)
            if expected_dimensions is None:
                expected_dimensions = (width, height)
            require((width, height) == expected_dimensions, "dimensions changed")
            if args.verify_images:
                from PIL import Image  # Optional, never installed by this tool.

                with Image.open(image) as decoded:
                    require(decoded.size == (width, height), "image size mismatch")
                    decoded.verify()
                with Image.open(image) as decoded:
                    decoded.load()
                audit["decoded_images"] += 1
            annotation = data["annotation"]
            lanes, elements = (
                annotation["lane_centerline"],
                annotation["traffic_element"],
            )
            require(isinstance(lanes, list) and isinstance(elements, list), "bad lists")
            lane_ids = set()
            for lane in lanes:
                require(
                    isinstance(lane["id"], str) and lane["id"] not in lane_ids,
                    "invalid/duplicate lane id",
                )
                lane_ids.add(lane["id"])
                require(
                    isinstance(lane["points"], list)
                    and len(lane["points"]) >= 2
                    and all(
                        isinstance(p, list)
                        and len(p) == 3
                        and all(finite(v) for v in p)
                        for p in lane["points"]
                    ),
                    "invalid lane geometry",
                )
            for key, columns in (
                ("topology_lclc", len(lanes)),
                ("topology_lcte", len(elements)),
            ):
                matrix = annotation[key]
                require(
                    isinstance(matrix, list)
                    and len(matrix) == len(lanes)
                    and all(
                        isinstance(row, list)
                        and len(row) == columns
                        and all(type(v) is int and v in (0, 1) for v in row)
                        for row in matrix
                    ),
                    f"invalid {key}",
                )
            audit["topology_lcte_edges"] += sum(map(sum, annotation["topology_lcte"]))
            rows = []
            counts = collections.Counter(element["id"] for element in elements)
            require(
                args.exclude_ambiguous_ids or max(counts.values(), default=0) <= 1,
                f"duplicate evaluation IDs in {path}; use "
                "--exclude-ambiguous-ids to retain boxes but exclude identity scoring",
            )
            for element in elements:
                ident, category, attr = (
                    element[k] for k in ("id", "category", "attribute")
                )
                require(
                    isinstance(ident, str)
                    and all(c.isalnum() or c == "_" for c in ident),
                    "invalid id",
                )
                require(type(category) is int and type(attr) is int, "invalid labels")
                valid = (
                    ident.startswith("tl_")
                    and (
                        (category == 0 and attr in (0, 4))
                        or (category == 1 and attr in (1, 2, 3))
                    )
                ) or (ident.startswith("rs_") and category == 0 and attr in (4, 5, 6))
                require(valid, "unexpected local category/attribute/prefix mapping")
                points = element["points"]
                require(
                    isinstance(points, list)
                    and len(points) == 2
                    and all(
                        isinstance(p, list)
                        and len(p) == 2
                        and all(finite(v) for v in p)
                        for p in points
                    ),
                    "invalid element geometry",
                )
                (x1, y1), (x2, y2) = points
                require(x1 < x2 and y1 < y2, "degenerate element")
                if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
                    require(
                        args.clip_partial_boxes,
                        "element outside image; "
                        "use --clip-partial-boxes to audit clipping",
                    )
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(width, x2), min(height, y2)
                    require(x1 < x2 and y1 < y2, "element entirely outside image")
                    audit["clipped_boxes"] += 1
                key = f"{ident.split('_')[0]}:{category}:{attr}"
                audit["category_attribute_counts"][key] = (
                    audit["category_attribute_counts"].get(key, 0) + 1
                )
                if ident.startswith("tl_"):
                    color = {1: "red", 2: "green", 3: "yellow"}.get(attr, "unknown")
                    evaluation_id = ident if counts[ident] == 1 else "-"
                    audit["ambiguous_identity_observations"] += evaluation_id == "-"
                    rows.append((evaluation_id, x1, y1, x2 - x1, y2 - y1, color))
            out.write(
                f"F\t{segment}\tring_front_center\t{timestamp}\t"
                f"{width}\t{height}\t{len(rows)}\n"
            )
            for row in rows:
                out.write("G\t" + "\t".join(map(str, row)) + "\n")
            audit["frames"] += 1
            audit["observations"] += len(rows)
    require(audit["frames"] > 0 and audit["observations"] > 0, "empty diagnostic")
    text = out.getvalue()
    audit["tsv_sha256"] = hashlib.sha256(text.encode()).hexdigest()
    sys.stderr.write(json.dumps(audit, sort_keys=True) + "\n")
    if args.output:
        args.output.write_text(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path)
    parser.add_argument("--local-prefix-diagnostic", action="store_true")
    parser.add_argument("--verify-images", action="store_true")
    parser.add_argument("--exclude-ambiguous-ids", action="store_true")
    parser.add_argument("--clip-partial-boxes", action="store_true")
    try:
        export(parser.parse_args())
    except (ValueError, KeyError, TypeError, OSError, ImportError) as error:
        sys.stderr.write(f"export failed: {error}\n")
        sys.exit(1)
