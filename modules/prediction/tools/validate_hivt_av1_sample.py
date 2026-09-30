"""Smoke-test deployed HiVT on AV1 forecasting samples in their city frame.

Dataset, HiVT checkout, checkpoint, and engine paths are supplied via CLI.
Lane selection uses XML centerlines, not ArgoverseMap's polygon bboxes; the
reported errors are not reference metrics.
"""

import argparse
import csv
import importlib
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch


def load_map(path):
    root = ET.parse(path).getroot()
    nodes = {
        node.attrib["id"]: (float(node.attrib["x"]), float(node.attrib["y"]))
        for node in root.iter("node")
    }
    lanes = []
    for way in root.iter("way"):
        points = np.asarray(
            [nodes[nd.attrib["ref"]] for nd in way.findall("nd")],
            dtype=np.float32,
        )
        if len(points) < 2:
            continue
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in way.findall("tag")}
        lanes.append((points, tags))
    return lanes


def build_scene(path, lanes, temporal_data_type):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    timestamps = sorted({float(row["TIMESTAMP"]) for row in rows})
    if len(timestamps) != 50:
        raise ValueError(f"{path}: expected 50 distinct timestamps")
    city = {row["CITY_NAME"] for row in rows}
    if len(city) != 1:
        raise ValueError(f"{path}: mixed cities")
    actor_ids = list(dict.fromkeys(
        row["TRACK_ID"] for row in rows if float(row["TIMESTAMP"]) <= timestamps[19]
    ))
    if not 2 <= len(actor_ids) <= 64:
        raise ValueError(f"{path}: unsupported actor count {len(actor_ids)}")
    tracks = defaultdict(dict)
    types = {}
    time_indices = {t: i for i, t in enumerate(timestamps)}
    for row in rows:
        tracks[row["TRACK_ID"]][time_indices[float(row["TIMESTAMP"])]] = (
            float(row["X"]), float(row["Y"])
        )
        types[row["TRACK_ID"]] = row["OBJECT_TYPE"]
    av, agent = (
        next(actor for actor in actor_ids if types[actor] == kind)
        for kind in ("AV", "AGENT")
    )
    origin_tensor = torch.tensor(tracks[av][19], dtype=torch.float32)
    heading = origin_tensor - torch.tensor(tracks[av][18], dtype=torch.float32)
    theta_tensor = torch.atan2(heading[1], heading[0])
    theta = float(theta_tensor)
    origin = origin_tensor.numpy()
    rotate_tensor = torch.tensor(
        [[torch.cos(theta_tensor), -torch.sin(theta_tensor)],
         [torch.sin(theta_tensor), torch.cos(theta_tensor)]]
    )
    rotate = rotate_tensor.numpy()
    count = len(actor_ids)
    positions = torch.zeros(count, 50, 2)
    padding = torch.ones(count, 50, dtype=torch.bool)
    angles = torch.zeros(count)
    for i, actor in enumerate(actor_ids):
        steps = tracks[actor]
        for t in range(20):
            if t in steps:
                positions[i, t] = (
                    torch.tensor(steps[t], dtype=torch.float32) - origin_tensor
                ) @ rotate_tensor
                padding[i, t] = False
        if 19 in steps:
            padding[i, 20:] = False
        valid = [t for t in range(20) if t in steps]
        if len(valid) > 1:
            delta = positions[i, valid[-1]] - positions[i, valid[-2]]
            angles[i] = torch.atan2(delta[1], delta[0])
    if 19 not in tracks[agent] or any(t not in tracks[agent] for t in range(20, 50)):
        raise ValueError(f"{path}: AGENT missing current/future labels")
    np.testing.assert_allclose(
        origin + positions[actor_ids.index(agent), 19].numpy() @ rotate.T,
        np.asarray(tracks[agent][19]), rtol=0, atol=1e-3,
    )
    bos = torch.zeros(count, 20, dtype=torch.bool)
    bos[:, 0] = ~padding[:, 0]
    bos[:, 1:] = padding[:, :19] & ~padding[:, 1:20]
    x = torch.zeros(count, 20, 2)
    valid_pairs = ~padding[:, 1:20] & ~padding[:, :19]
    x[:, 1:] = torch.where(
        valid_pairs.unsqueeze(-1), positions[:, 1:20] - positions[:, :19],
        torch.zeros_like(x[:, 1:]),
    )

    current = np.asarray(
        [tracks[actor][19] for actor in actor_ids if 19 in tracks[actor]],
        dtype=np.float32,
    )
    current_indices = [i for i, actor in enumerate(actor_ids) if 19 in tracks[actor]]
    # The deployment caps lane polylines at 12; use centerline proximity here.
    nearby = []
    for points, tags in lanes:
        distance = np.min(np.linalg.norm(
            points[:, None, :] - current[None, :, :], axis=-1
        ))
        if distance < 50:
            nearby.append((distance, points, tags))
    nearby.sort(key=lambda entry: entry[0])
    lane_starts, lane_deltas, intersections, turns, controls = [], [], [], [], []
    for _, points, tags in nearby[:12]:
        start = (points[:-1] - origin) @ rotate
        delta = (points[1:] - points[:-1]) @ rotate
        lane_starts.extend(start)
        lane_deltas.extend(delta)
        intersections.extend([int(tags["is_intersection"] == "True")] * len(delta))
        turns.extend([{"NONE": 0, "LEFT": 1, "RIGHT": 2}[tags["turn_direction"]]] * len(delta))
        controls.extend([int(tags["has_traffic_control"] == "True")] * len(delta))
    if not lane_deltas or len(lane_deltas) > 1024:
        raise ValueError(f"{path}: lane vectors outside engine capacity: {len(lane_deltas)}")
    current_scene = positions[current_indices, 19].numpy()
    pairs, relative = [], []
    for lane_index, start in enumerate(lane_starts):
        for i, actor_index in enumerate(current_indices):
            vector = start - current_scene[i]
            if np.linalg.norm(vector) < 50:
                pairs.append((lane_index, actor_index))
                relative.append(vector)
    if not pairs or len(pairs) > 64 * 1024:
        raise ValueError(f"{path}: invalid lane/actor edges: {len(pairs)}")
    scene = temporal_data_type(
        x=x, positions=positions,
        edge_index=torch.tensor(
            [(i, j) for i in range(count) for j in range(count) if i != j],
            dtype=torch.long,
        ).T.contiguous(),
        num_nodes=count, padding_mask=padding, bos_mask=bos,
        rotate_angles=angles,
        lane_vectors=torch.tensor(np.asarray(lane_deltas)),
        is_intersections=torch.tensor(intersections, dtype=torch.uint8),
        turn_directions=torch.tensor(turns, dtype=torch.uint8),
        traffic_controls=torch.tensor(controls, dtype=torch.uint8),
        lane_actor_index=torch.tensor(pairs, dtype=torch.long).T.contiguous(),
        lane_actor_vectors=torch.tensor(np.asarray(relative)),
    )
    ground_truth = np.asarray([tracks[agent][t] for t in range(20, 50)])
    return scene, actor_ids.index(agent), np.asarray(tracks[agent][19]), ground_truth, theta, len(nearby)


def main():
    parser = argparse.ArgumentParser(
        description="Compare the deployed HiVT engine with its checkpoint on AV1 samples."
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        required=True,
        help="Argoverse dataset root containing forecasting_sample/data and map_files",
    )
    parser.add_argument(
        "--hivt-root",
        type=Path,
        required=True,
        help="HiVT checkout containing deployment/, models/, and utils.py",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        required=True,
        help="HiVT checkpoint compatible with the deployed engine",
    )
    parser.add_argument(
        "--engine",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "data" / "hivt64.engine",
        help="TensorRT engine (default: modules/prediction/data/hivt64.engine)",
    )
    args = parser.parse_args()

    hivt_root = args.hivt_root.resolve()
    dataset_root = args.dataset.resolve()
    checkpoint_path = args.checkpoint.resolve()
    engine_path = args.engine.resolve()
    runtime_paths = [
        hivt_root / "deployment" / "build_hivt_trt_engine.py",
        hivt_root / "deployment" / "export_trace.py",
        hivt_root / "models" / "hivt.py",
        hivt_root / "utils.py",
    ]
    for path in [*runtime_paths, checkpoint_path, engine_path]:
        if not path.is_file():
            parser.error(f"required file does not exist: {path}")
    sample_dir = dataset_root / "forecasting_sample" / "data"
    map_dir = dataset_root / "map_files"
    if not sample_dir.is_dir() or not map_dir.is_dir():
        parser.error(
            "--dataset must contain forecasting_sample/data and map_files"
        )

    sys.path[:0] = [str(hivt_root / "deployment"), str(hivt_root)]
    engine_runtime = importlib.import_module("build_hivt_trt_engine")
    trace_export = importlib.import_module("export_trace")
    hivt_model = importlib.import_module("models.hivt")
    temporal_data_type = importlib.import_module("utils").TemporalData
    trt = importlib.import_module("tensorrt")

    paths = sorted(sample_dir.glob("*.csv"))
    if not paths:
        raise ValueError("No AV1 forecasting sample CSVs found")
    logger = trt.Logger(trt.Logger.WARNING)
    if not trt.init_libnvinfer_plugins(logger, ""):
        raise RuntimeError("TensorRT plugin initialization failed")
    runtime = trt.Runtime(logger)
    with engine_path.open("rb") as stream:
        engine = runtime.deserialize_cuda_engine(stream.read())
    if engine is None:
        raise RuntimeError("Could not load deployed HiVT engine")
    cuda = engine_runtime.cuda_api()
    engine_runtime.check_cuda(cuda.cudaSetDevice(0), "cudaSetDevice")
    model = hivt_model.HiVT.load_from_checkpoint(
        checkpoint_path=str(checkpoint_path),
        map_location="cpu", weights_only=False, parallel=True,
    ).eval()
    wrapper = trace_export.TensorHiVT(model).eval()
    maps = {}
    errors = []
    for path in paths:
        with path.open(newline="") as stream:
            city = next(csv.DictReader(stream))["CITY_NAME"]
        if city not in maps:
            city_id = {"PIT": "PIT_10314", "MIA": "MIA_10316"}[city]
            maps[city] = load_map(
                map_dir / f"pruned_argoverse_{city_id}_vector_map.xml"
            )
        scene, agent, anchor, truth, theta, lane_count = build_scene(
            path, maps[city], temporal_data_type
        )
        inputs = trace_export.tensor_inputs(scene)
        with torch.inference_mode():
            torch_traj, torch_logits = wrapper(*inputs)
        actual = engine_runtime.run_engine(engine, runtime, cuda, inputs)
        expected = torch_traj.numpy()
        delta = float(np.max(np.abs(actual["trajectories"] - expected)))
        logit_delta = float(np.max(np.abs(actual["logits"] - torch_logits.numpy())))
        np.testing.assert_allclose(actual["trajectories"], expected, rtol=2e-3, atol=2e-3)
        np.testing.assert_allclose(actual["logits"], torch_logits.numpy(), rtol=2e-3, atol=2e-3)
        local = actual["trajectories"][:, agent, :, :2]
        angle = theta + float(scene.rotate_angles[agent])
        rotation = np.array(
            [[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]]
        )
        city_xy = anchor + local @ rotation
        # Independent scene-frame reconstruction of the actor-local decode.
        scene_rot = np.array(
            [[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]]
        )
        actor_angle = float(scene.rotate_angles[agent])
        actor_rot = np.array(
            [[math.cos(actor_angle), math.sin(actor_angle)],
             [-math.sin(actor_angle), math.cos(actor_angle)]]
        )
        decoded = anchor + (local @ actor_rot) @ scene_rot
        np.testing.assert_allclose(city_xy, decoded, rtol=0, atol=1e-5)
        if not np.isfinite(city_xy).all():
            raise ValueError(f"{path}: non-finite decoded coordinates")
        distances = np.linalg.norm(city_xy - truth[None, :, :], axis=-1)
        probs = torch.softmax(torch.from_numpy(actual["logits"][agent]), dim=0).numpy()
        np.testing.assert_allclose(probs.sum(), 1, rtol=0, atol=1e-6)
        top = int(np.argmax(probs))
        best = int(np.argmin(distances.mean(axis=1)))
        errors.append((distances[top].mean(), distances[top, -1],
                       distances[best].mean(), distances[best, -1]))
        print(
            f"{path.name} city={city} actors={scene.num_nodes} lanes={lane_count}/12 "
            f"vectors={len(scene.lane_vectors)} edges={len(scene.lane_actor_vectors)} "
            f"max_raw_delta={delta:.6g}/{logit_delta:.6g} "
            f"top1_ADE/FDE={errors[-1][0]:.2f}/{errors[-1][1]:.2f}m "
            f"bestADE6_ADE/FDE={errors[-1][2]:.2f}/{errors[-1][3]:.2f}m "
            f"miss6_2m={int(np.min(distances[:, -1]) > 2)}",
            flush=True,
        )
    print(f"processed={len(errors)}/{len(paths)} mean_top1_ADE/FDE_bestADE6_ADE/FDE="
          f"{np.mean(errors, axis=0).round(2).tolist()}m")


if __name__ == "__main__":
    main()
