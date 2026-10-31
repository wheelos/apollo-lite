#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Run modular C++ terrain on frozen original XYZ and compare reference outputs."""

import argparse
import json
import shlex
import subprocess
from pathlib import Path

import numpy as np
from terrain_debug import ROOT, _configs, digest, export_debug
from terrain_traversability_experiment import _frame_metrics

CONFIG_FIELDS = (
    "x_min",
    "x_max",
    "y_min",
    "y_max",
    "cell_size",
    "expected_ground_z",
    "seed_height_band",
    "anchor_radius",
    "layer_gap",
    "ground_tolerance",
    "max_slope_gradient",
    "max_neighbor_height_change",
    "pit_rim_search_radius",
    "min_anchor_points",
    "min_pit_floor_points",
)
CELL_FIELDS = {
    "cell_status": "surface_cell_status",
    "elevation_z": "surface_cell_elevation_z_m",
    "slope_gradient": "surface_cell_slope_gradient",
    "roughness_m": "surface_cell_roughness_m",
    "step_m": "surface_cell_step_m",
    "step_status": "surface_cell_step_status",
    "pit_depth_m": "surface_pit_depth_m",
    "pit_status": "surface_pit_status",
    "terrain_state": "terrain_state",
    "decision_state": "decision_state",
    "reason_bits": "reason_bits",
    "terrain_candidate": "terrain_candidate",
    "risk_candidate": "risk_constrained_candidate",
    "terrain_connected": "terrain_connected",
    "risk_connected": "risk_constrained_connected",
    "risk_bits": "risk_layers",
    "risk_observed": "risk_observed",
}
BOOL_FIELDS = {
    "terrain_candidate",
    "risk_candidate",
    "terrain_connected",
    "risk_connected",
    "risk_observed",
}
STRING_FIELDS = {
    "cell_status",
    "step_status",
    "pit_status",
    "terrain_state",
    "decision_state",
}
INT_FIELDS = {"ix", "iy", "reason_bits", "risk_bits", "raw_return_count"}


def write_input(path, arrays, config, limits, root_id):
    shape = arrays["terrain_state"].shape
    with Path(path).open("w") as stream:
        stream.write("TERRAIN_V1\nCONFIG ")
        values = [
            "nan" if config[name] is None else str(config[name])
            for name in CONFIG_FIELDS
        ]
        stream.write(" ".join(values + [str(root_id)]) + "\nLIMITS ")
        stream.write(
            " ".join(
                str(limits[name])
                for name in (
                    "max_slope_gradient",
                    "max_roughness_m",
                    "max_step_m",
                    "max_depression_depth_m",
                )
            )
            + f"\nPOINTS {len(arrays['point_id'])}\n"
        )
        for point_id, xyz, risk in zip(
            arrays["point_id"], arrays["xyz_m"], arrays["surface_point_risk_mask"]
        ):
            stream.write(
                f"{int(point_id)} {xyz[0]:.17g} {xyz[1]:.17g} {xyz[2]:.17g} "
                f"{4 if risk else 0}\n"
            )
        stream.write(f"RISK_GRID {shape[0]} {shape[1]}\n")
        for ix, iy in np.ndindex(shape):
            stream.write(
                f"{ix} {iy} {int(arrays['risk_layers'][ix, iy])} "
                f"{int(arrays['risk_observed'][ix, iy])}\n"
            )


def read_output(path):
    with Path(path).open() as stream:
        if stream.readline().strip() != "TERRAIN_V1_OUTPUT":
            raise ValueError("invalid C++ output version")
        meta = shlex.split(stream.readline())
        if meta[0] != "META":
            raise ValueError("missing C++ output metadata")
        shape = tuple(map(int, meta[1:3]))
        header = stream.readline().split()
        if header[0] != "CELL_COLUMNS":
            raise ValueError("missing C++ cell column schema")
        columns = header[1:]
        rows = [stream.readline().split() for _ in range(np.prod(shape))]
        if any(len(row) != len(columns) for row in rows):
            raise ValueError("truncated C++ cell rows")
        cells = {}
        for index, name in enumerate(columns):
            dtype = (
                str
                if name in STRING_FIELDS
                else np.int64 if name in INT_FIELDS | BOOL_FIELDS else float
            )
            cells[name] = np.asarray([row[index] for row in rows], dtype=dtype).reshape(
                shape
            )
        for name in BOOL_FIELDS:
            if not np.isin(cells[name], (0, 1)).all():
                raise ValueError(f"nonboolean C++ field: {name}")
            cells[name] = cells[name].astype(bool)
        ix, iy = np.indices(shape)
        if not np.array_equal(cells["ix"], ix) or not np.array_equal(cells["iy"], iy):
            raise ValueError("C++ cell order differs from x-major input")
        header = stream.readline().split()
        layers = []
        if header and header[0] == "LAYER_COLUMNS":
            header = stream.readline().split()
            while header and header[0] != "POINT_COLUMNS":
                if len(header) != 7:
                    raise ValueError("invalid C++ raw-layer row")
                layers.append(
                    {
                        "cell": list(map(int, header[:2])),
                        "id": int(header[2]),
                        "elevation": float(header[3]),
                        "count": int(header[4]),
                        "selected": int(header[5]),
                        "point_ids": list(map(int, header[6].split(","))),
                    }
                )
                header = stream.readline().split()
        if header[0] != "POINT_COLUMNS":
            raise ValueError("missing C++ point schema")
        point_columns = header[1:]
        point_rows = [line.split() for line in stream if line.strip()]
        if any(len(row) != len(point_columns) for row in point_rows):
            raise ValueError("truncated C++ point rows")
        points = {
            name: np.asarray(
                [row[index] for row in point_rows],
                dtype=float if name in ("surface_z", "residual_z") else np.int64,
            )
            for index, name in enumerate(point_columns)
        }
        return meta, cells, points, layers


def compare_fields(native, reference):
    deltas = {}
    for name, reference_name in CELL_FIELDS.items():
        got, expected = native[name], reference[reference_name]
        if got.shape != expected.shape:
            raise ValueError(f"reference shape differs: {name}")
        if got.dtype.kind == "f":
            equal = np.isclose(got, expected, rtol=1e-8, atol=1e-9, equal_nan=True)
        else:
            equal = got == expected
        if not equal.all():
            deltas[name] = {
                "different_cells": int((~equal).sum()),
                "first_cells": np.argwhere(~equal)[:5].tolist(),
            }
    return deltas


def local_coverage(cells, config):
    """Descriptive origin-centered windows, not vehicle acceptance thresholds."""
    ix, iy = np.indices(cells["terrain_candidate"].shape)
    xx = config["x_min"] + (ix + 0.5) * config["cell_size"]
    yy = config["y_min"] + (iy + 0.5) * config["cell_size"]
    distance = np.hypot(xx, yy)
    rows = []
    for radius in (5, 10, 20):
        window = distance <= radius
        row = {"radius_m": radius, "ROI_cells": int(window.sum())}
        for field in (
            "terrain_candidate",
            "risk_candidate",
            "terrain_connected",
            "risk_connected",
            "risk_observed",
        ):
            row[field + "_cells"] = int(np.count_nonzero(window & cells[field]))
        for name, bit in (("measured", 1), ("object", 2), ("unknown", 4)):
            row[name + "_risk_on_terrain_candidate_cells"] = int(
                np.count_nonzero(
                    window
                    & cells["terrain_candidate"]
                    & ((cells["risk_bits"] & bit) != 0)
                )
            )
        row["unobserved_risk_on_terrain_candidate_cells"] = int(
            np.count_nonzero(
                window & cells["terrain_candidate"] & ~cells["risk_observed"]
            )
        )
        rows.append(row)
    return rows


def run_validation(binary, report_path, output, root=ROOT):
    binary, report_path, output, root = map(Path, (binary, report_path, output, root))
    report = json.loads(report_path.read_text())
    source_hash = digest(report_path)
    binary_hash = digest(binary)
    configs = _configs(root)
    original = json.loads(
        (
            root / ".cache/lidar-fusion-validation/terrain-v4/real-data-report.json"
        ).read_text()
    )
    original_entries = original["cached_frames"] + original["raw_non08_frames"]
    originals = {
        "frames/"
        + str(frame["artifact_path"])
        .split("/frames/", 1)[1]
        .replace(".terrain.npz", ".traversability.npz"): frame
        for frame in original_entries
    }
    output.mkdir(parents=True, exist_ok=True)
    results, display_frames = [], []
    for frame in report["frames"]:
        relative = Path(frame["artifact_path"])
        source = report_path.parent / relative
        if digest(source) != frame["artifact_sha256"]:
            raise ValueError(f"changed reference archive: {source}")
        with np.load(source, allow_pickle=False) as stored:
            arrays = {name: stored[name] for name in stored.files}
        frozen = originals[str(relative)]
        frozen_path = root / frozen["artifact_path"]
        if digest(frozen_path) != frozen["artifact_sha256"]:
            raise ValueError("changed original XYZ archive")
        with np.load(frozen_path, allow_pickle=False) as stored:
            root_id = int(stored["root_anchor_requested_id"][0])
            selected_root_id = int(stored["root_anchor_selected_id"][0])
            point_reference = {
                "id": stored["point_id"],
                "cell_id": stored["point_cell_id"],
                "layer_id": stored["point_layer_id"],
                "ground_candidate": stored["point_ground_candidate_mask"],
                "ground": stored["point_ground_mask"],
                "risk": stored["point_risk_mask"],
                "surface_z": stored["point_surface_z"],
                "residual_z": stored["point_residual_z"],
            }
            layer_reference = {
                key: stored[key]
                for key in stored.files
                if key.startswith("observed_layer_")
            }
            extra_reference = {
                "raw_return_count": stored["cell_raw_return_count"],
                "observed_layer_span_m": stored["cell_observed_layer_span_m"],
                "pit_width_x_m": stored["pit_width_x_m"],
                "pit_width_y_m": stored["pit_width_y_m"],
            }
        base = output / relative.with_suffix("")
        base.parent.mkdir(parents=True, exist_ok=True)
        input_path, output_path = base.with_suffix(".input.txt"), base.with_suffix(
            ".native.txt"
        )
        write_input(
            input_path, arrays, configs[str(relative)], report["limits"], root_id
        )
        completed = subprocess.run(
            [str(binary), "--input", str(input_path), "--output", str(output_path)],
            check=False,
            capture_output=True,
            text=True,
            timeout=300,
        )
        if completed.returncode:
            raise RuntimeError(f"C++ terrain failed: {completed.stderr}")
        meta, cells, points, layers = read_output(output_path)
        deltas = compare_fields(cells, arrays)
        for name, expected in extra_reference.items():
            equal = np.isclose(
                cells[name], expected, rtol=1e-8, atol=1e-9, equal_nan=True
            )
            if not equal.all():
                deltas[name] = {
                    "different_cells": int((~equal).sum()),
                    "first_cells": np.argwhere(~equal)[:5].tolist(),
                }
        native_root = [-1, -1] if meta[4] == "none" else list(map(int, meta[4:6]))
        if native_root != arrays["root_cell"].tolist():
            deltas["root_cell"] = {
                "native": native_root,
                "reference": arrays["root_cell"].tolist(),
            }
        native_root_id = -1 if meta[3] == "none" else int(meta[3])
        if native_root_id != selected_root_id:
            deltas["root_anchor_id"] = {
                "native": native_root_id,
                "reference": selected_root_id,
            }
        point_deltas = {}
        for name, expected in point_reference.items():
            got = points[name]
            if name == "risk":
                got = got != 0
            if got.shape != expected.shape:
                raise ValueError(f"C++ point count differs: {name}")
            equal = (
                np.isclose(got, expected, rtol=1e-8, atol=1e-9, equal_nan=True)
                if got.dtype.kind == "f"
                else got == expected
            )
            if not equal.all():
                point_deltas[name] = int((~equal).sum())
        layer_deltas = []
        if len(layers) != len(layer_reference["observed_layer_id"]):
            layer_deltas.append("raw_layer_count")
        else:
            offsets = layer_reference["observed_layer_source_offsets"]
            for index, layer in enumerate(layers):
                ids = layer_reference["observed_layer_source_point_ids"][
                    offsets[index] : offsets[index + 1]
                ]
                if (
                    layer["cell"]
                    != layer_reference["observed_layer_cell_xy"][index].tolist()
                    or layer["id"] != layer_reference["observed_layer_id"][index]
                    or layer["count"]
                    != layer_reference["observed_layer_point_count"][index]
                    or bool(layer["selected"])
                    != layer_reference["observed_layer_selected"][index]
                    or not np.isclose(
                        layer["elevation"],
                        layer_reference["observed_layer_elevation_z_m"][index],
                        rtol=1e-8,
                        atol=1e-9,
                    )
                    or not np.array_equal(layer["point_ids"], ids)
                ):
                    layer_deltas.append(index)
        visual = dict(arrays)
        visual.update({target: cells[name] for name, target in CELL_FIELDS.items()})
        visual["point_cell_id"] = points["cell_id"]
        visual["point_layer_id"] = points["layer_id"]
        visual["root_cell"] = np.asarray(native_root)
        visual["root_anchor_point_id"] = np.asarray([native_root_id])
        visual["overall_status"] = np.asarray(meta[7])
        provenance = json.loads(str(visual["source_hashes_json"]))
        provenance.update(
            {
                "native_binary_sha256": binary_hash,
                "native_input_sha256": digest(input_path),
                "native_output_sha256": digest(output_path),
                "prediction_source": "C++ terrain numerical modules",
                "reference_archive_sha256": frame["artifact_sha256"],
            }
        )
        visual["source_hashes_json"] = np.asarray(
            json.dumps(provenance, sort_keys=True)
        )
        artifact = output / relative
        np.savez_compressed(artifact, **visual)
        display_frame = dict(frame)
        display_frame["artifact_sha256"] = digest(artifact)
        display_frame["metrics"] = _frame_metrics(
            visual, configs[str(relative)]["cell_size"], meta[7]
        )
        display_frames.append(display_frame)
        result = {
            "frame_id": frame["frame_id"],
            "reference_sha256": frame["artifact_sha256"],
            "input_sha256": digest(input_path),
            "native_output_sha256": digest(output_path),
            "native_archive": str(artifact),
            "native_archive_sha256": digest(artifact),
            "cell_deltas": deltas,
            "point_deltas": point_deltas,
            "raw_layer_deltas": layer_deltas,
            "native_metadata": meta,
            "requested_root_id": root_id,
            "origin_centered_coverage_windows": local_coverage(
                cells, configs[str(relative)]
            ),
        }
        results.append(result)
        print(
            f"Native terrain {len(results)}/{len(report['frames'])}: "
            f"{frame['frame_id']} cell_deltas={len(deltas)} "
            f"point_deltas={len(point_deltas)}",
            flush=True,
        )
    if digest(binary) != binary_hash or digest(report_path) != source_hash:
        raise ValueError("binary/reference changed during validation")
    display = dict(report)
    display["frames"] = display_frames
    display["prediction_source"] = "Actual C++ terrain numerical output"
    display_path = output / "native-display-report.json"
    display_path.write_text(json.dumps(display, indent=2, allow_nan=False) + "\n")
    export_debug(display_path, output / "bev", root=root)
    final = {
        "status": (
            "PASS_CPP_REFERENCE_PARITY"
            if all(
                not row["cell_deltas"]
                and not row["point_deltas"]
                and not row["raw_layer_deltas"]
                for row in results
            )
            else "CPP_REFERENCE_DIFFERENCES_REQUIRE_ANALYSIS"
        ),
        "diagnostic_only": True,
        "binary": str(binary),
        "binary_sha256": binary_hash,
        "reference_report_sha256": source_hash,
        "validator_sha256": digest(__file__),
        "frame_count": len(results),
        "numeric_tolerance": {"relative": 1e-8, "absolute": 1e-9},
        "numeric_tolerance_contract": (
            "Local plane linear algebra roundoff in metre-scale XYZ; "
            "states and IDs compared exactly."
        ),
        "root_reference_contract": (
            "Compare selected root ID from the frozen surface result; requested "
            "root IDs can exist even when initialization returns no anchor."
        ),
        "frames": results,
    }
    (output / "native-validation.json").write_text(
        json.dumps(final, indent=2, allow_nan=False) + "\n"
    )
    return final


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument(
        "--report",
        type=Path,
        default=ROOT / ".cache/lidar-fusion-validation/terrain-v6/report.json",
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "data/lidar_fusion_debug/cpp"
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    result = run_validation(args.binary, args.report, args.output, args.root)
    print(result["status"])


if __name__ == "__main__":
    main()
