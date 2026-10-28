#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Prepare bounded nuScenes original-sweep replay, never substitute truth for models."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
from google.protobuf import text_format

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "perception_diagnostics"))
from proto import Reader  # noqa: E402


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def records(path):
    """Stream official JSON arrays without retaining multi-GB tables."""
    decoder = json.JSONDecoder()
    with path.open() as stream:
        buffer = ""
        eof = False
        while True:
            buffer = buffer.lstrip()
            if buffer.startswith(("[", ",")):
                buffer = buffer[1:].lstrip()
            if buffer.startswith("]"):
                return
            try:
                value, end = decoder.raw_decode(buffer)
            except json.JSONDecodeError:
                if eof:
                    raise ValueError(f"truncated or malformed JSON array: {path}")
                block = stream.read(1024 * 1024)
                eof = not block
                buffer += block
                continue
            yield value
            buffer = buffer[end:]


def rotation(q):
    w, x, y, z = q
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-5:
        raise ValueError("invalid nuScenes quaternion")
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def category(name):
    if name.startswith("human.pedestrian."):
        return "pedestrian"
    return {
        "vehicle.car": "car",
        "vehicle.truck": "truck",
        "vehicle.bus.bendy": "bus",
        "vehicle.bus.rigid": "bus",
        "vehicle.trailer": "trailer",
        "vehicle.construction": "construction_vehicle",
        "vehicle.motorcycle": "motorcycle",
        "vehicle.bicycle": "bicycle",
        "movable_object.barrier": "barrier",
        "movable_object.trafficcone": "traffic_cone",
    }.get(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("data-root", "manifest", "predictions", "schema", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--scenes", type=int, default=3)
    parser.add_argument("--frames-per-scene", type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.scenes <= 10 or not 1 <= args.frames_per_scene <= 20:
        parser.error("bounded scope: 1..10 scenes, 1..20 frames per scene")
    args.output.mkdir(parents=True, exist_ok=False)
    tables = args.data_root / "v1.0-trainval"
    samples = {s["token"]: s for s in records(tables / "sample.json")}
    scenes = {s["token"]: s for s in records(tables / "scene.json")}
    selected, scene_order = [], []
    with args.manifest.open() as stream:
        for line in stream:
            item = json.loads(line)
            scene = samples[item["sample_token"]]["scene_token"]
            if scene not in scene_order and len(scene_order) < args.scenes:
                scene_order.append(scene)
            if (
                scene in scene_order
                and sum(s["_scene"] == scene for s in selected) < args.frames_per_scene
            ):
                item["_scene"] = scene
                selected.append(item)
    if len(selected) != args.scenes * args.frames_per_scene:
        raise ValueError("insufficient bounded validation frames")
    for scene in scene_order:
        rows = [s for s in selected if s["_scene"] == scene]
        for first, second in zip(rows, rows[1:]):
            if samples[first["sample_token"]]["next"] != second["sample_token"]:
                raise ValueError("selected frames must be consecutive official samples")
    tokens = {s["sample_token"] for s in selected}
    prediction = {}
    with args.predictions.open() as stream:
        for line in stream:
            item = json.loads(line)
            if item.get("sample_token") in tokens:
                if item["sample_token"] in prediction:
                    raise ValueError("duplicate cached detection token")
                prediction[item["sample_token"]] = item
    if set(prediction) != tokens:
        raise ValueError("missing token-aligned actual detector outputs")
    sensors = {s["token"]: s["channel"] for s in records(tables / "sensor.json")}
    calib = {s["token"]: s for s in records(tables / "calibrated_sensor.json")}
    keyframes = {}
    for sd in records(tables / "sample_data.json"):
        if sd["sample_token"] in tokens and sd["is_key_frame"]:
            channel = sensors[calib[sd["calibrated_sensor_token"]]["sensor_token"]]
            if channel in ("LIDAR_TOP", "CAM_FRONT"):
                keyframes[(sd["sample_token"], channel)] = sd
    pose_tokens = {sd["ego_pose_token"] for sd in keyframes.values()}
    official_poses = {
        p["token"]: p
        for p in records(tables / "ego_pose.json")
        if p["token"] in pose_tokens
    }
    instances = {
        s["token"]: s["category_token"] for s in records(tables / "instance.json")
    }
    categories = {s["token"]: s["name"] for s in records(tables / "category.json")}
    annotations = {token: [] for token in tokens}
    for ann in records(tables / "sample_annotation.json"):
        if ann["sample_token"] in tokens:
            ann["category"] = categories[instances[ann["instance_token"]]]
            annotations[ann["sample_token"]].append(ann)
    reader = Reader(args.schema)
    template = Path("modules/lidar_fusion/conf/lidar_fusion.pb.txt")
    experiment = {
        "dataset": "nuScenes trainval validation subset",
        "selection": "first scenes and leading frames in existing validation manifest",
        "prediction_provenance": "cached actual CenterPoint 10-sweep runtime outputs",
        "prediction_sha256": digest(args.predictions),
        "validation_manifest_sha256": digest(args.manifest),
        "segmentation": (
            "new RangeRet SemanticKITTI inference; cross-domain, no lidarseg GT"
        ),
        "ground_assumption": "expected_ground_z=0 in ego frame, not ground truth",
        "runs": [],
    }
    for scene_token in scene_order:
        scene = scenes[scene_token]
        run = args.output / scene["name"]
        inputs, detections = run / "inputs", run / "detections"
        inputs.mkdir(parents=True)
        detections.mkdir()
        truth, rows = {}, []
        config = reader.message("apollo.lidar_fusion.FusionConfig")
        text_format.Parse(template.read_text(), config)
        config.pointcloud_channel = "/offline/nuscenes/LIDAR_TOP"
        config.detection_channel = "/offline/nuscenes/centerpoint"
        config.segmentation_channel = "/offline/nuscenes/rangeret"
        config.sensor_frame = "LIDAR_TOP"
        config.base_frame = "nuscenes_ego"
        config.min_x = config.min_y = -50
        config.max_x = config.max_y = 50
        config.spatial.point_views = True
        config.spatial.trace_rays = True
        config.spatial.verified_single_origin = False
        calibration = None
        for index, item in enumerate(s for s in selected if s["_scene"] == scene_token):
            token = item["sample_token"]
            sd = keyframes[(token, "LIDAR_TOP")]
            cs = calib[sd["calibrated_sensor_token"]]
            matrix = np.asarray(item["lidar2ego"], dtype=float)
            pose = np.asarray(item["ego2global"], dtype=float)
            official_pose = official_poses[sd["ego_pose_token"]]
            if not np.allclose(
                pose[:3, :3], rotation(official_pose["rotation"]), atol=1e-6
            ) or not np.allclose(pose[:3, 3], official_pose["translation"], atol=1e-4):
                raise ValueError("official ego pose and cached manifest disagree")
            if not np.allclose(
                matrix[:3, :3], rotation(cs["rotation"]), atol=1e-6
            ) or not np.allclose(matrix[:3, 3], cs["translation"], atol=1e-6):
                raise ValueError("official calibration and detector manifest disagree")
            if calibration is not None and not np.allclose(
                calibration, matrix, atol=1e-9
            ):
                raise ValueError("per-run calibration changed")
            calibration = matrix
            source = args.data_root / sd["filename"]
            if source.resolve() != Path(item["source_path"]).resolve():
                raise ValueError("cached detector source path mismatch")
            cached = prediction[token]
            if (
                Path(cached["source_path"]).resolve() != source.resolve()
                or abs(
                    cached.get("timestamp", item["timestamp"]) - sd["timestamp"] / 1e6
                )
                > 1e-6
            ):
                raise ValueError("cached detector source/time mismatch")
            points = np.fromfile(source, dtype="<f4")
            if points.size % 5 or not np.isfinite(points).all():
                raise ValueError("invalid original nuScenes five-float sweep")
            points = points.reshape(-1, 5)
            if not np.all(points[:, 3] == np.floor(points[:, 3])) or np.any(
                points[:, 3] < 0
            ):
                raise ValueError("integer intensity contract would lose original data")
            cloud = reader.message("apollo.drivers.PointCloud")
            cloud.frame_id = cloud.header.frame_id = "LIDAR_TOP"
            cloud.header.sequence_num = index + 1
            cloud.header.lidar_timestamp = sd["timestamp"] * 1000
            cloud.header.timestamp_sec = cloud.measurement_time = sd["timestamp"] / 1e6
            for x, y, z, intensity, _ring in points:
                p = cloud.point.add()
                p.x, p.y, p.z, p.intensity = (
                    float(x),
                    float(y),
                    float(z),
                    int(intensity),
                )
            name = f"{index:06d}.cloud.pb"
            payload = cloud.SerializeToString()
            (inputs / name).write_bytes(payload)
            det = reader.message(
                "apollo.lidar_object_detection.LidarObjectDetectionFrameResult"
            )
            det.header.CopyFrom(cloud.header)
            det.backend = config.detection_backend
            det.sensor_frame_id = "LIDAR_TOP"
            det.model_revision = cached["model_revision"] + "-cached-10sweep"
            det.input_point_count = cached["point_count"]
            for b in cached["detections"]:
                box = det.detection.add()
                box.center_x, box.center_y, box.center_z = b["center"]
                box.size_x, box.size_y, box.size_z = b["size"]
                box.yaw, box.class_id, box.score = b["yaw"], b["class_id"], b["score"]
                # Cached runtime velocities are placeholders, not measured motion.
            (detections / (name + ".det.pb")).write_bytes(det.SerializeToString())
            boxes = []
            for ann in annotations[token]:
                cls = category(ann["category"])
                if cls is None or ann["num_lidar_pts"] == 0:
                    continue
                center = pose[:3, :3].T @ (np.array(ann["translation"]) - pose[:3, 3])
                orient = pose[:3, :3].T @ rotation(ann["rotation"])
                boxes.append(
                    {
                        "token": ann["token"],
                        "class": cls,
                        "center": center.tolist(),
                        "size": [ann["size"][1], ann["size"][0], ann["size"][2]],
                        "yaw": math.atan2(orient[1, 0], orient[0, 0]),
                        "num_lidar_pts": ann["num_lidar_pts"],
                    }
                )
            camera = keyframes.get((token, "CAM_FRONT"))
            truth[name] = {
                "sample_token": token,
                "scene": scene["name"],
                "boxes": boxes,
                "camera": str(args.data_root / camera["filename"]) if camera else None,
                "camera_timestamp_skew_seconds": (
                    (camera["timestamp"] - sd["timestamp"]) / 1e6 if camera else None
                ),
            }
            rows.append(
                {
                    "file": name,
                    "sample_token": token,
                    "points": len(points),
                    "measurement_time": cloud.measurement_time,
                    "sha256": hashlib.sha256(payload).hexdigest(),
                    "source": str(source),
                    "source_sha256": digest(source),
                    "lidar2ego": matrix.tolist(),
                    "ego2global": pose.tolist(),
                }
            )
        t = config.sensor_to_base
        t.version = "nuscenes-" + hashlib.sha256(calibration.tobytes()).hexdigest()[:16]
        t.x, t.y, t.z = calibration[:3, 3]
        r = calibration[:3, :3]
        t.roll = math.atan2(r[2, 1], r[2, 2])
        t.pitch = math.asin(-r[2, 0])
        t.yaw = math.atan2(r[1, 0], r[0, 0])
        (inputs / "fusion.pb.txt").write_text(text_format.MessageToString(config))
        (inputs / "manifest.json").write_text(
            json.dumps(
                {
                    "samples": rows,
                    "total_clouds": len(rows),
                    "output_frame": "nuscenes_ego",
                    "calibration_parent": "nuScenes calibrated_sensor ego",
                    "record": str(args.data_root),
                },
                indent=2,
            )
            + "\n"
        )
        (run / "truth.json").write_text(json.dumps(truth, indent=2) + "\n")
        experiment["runs"].append(
            {
                "scene": scene["name"],
                "inputs": str(inputs.resolve()),
                "detections": str(detections.resolve()),
                "results": str((run / "results").resolve()),
                "truth": str((run / "truth.json").resolve()),
                "frames": len(rows),
            }
        )
    (args.output / "experiment.json").write_text(
        json.dumps(experiment, indent=2) + "\n"
    )
    print(
        json.dumps(
            {"scenes": len(scene_order), "frames": len(selected), "status": "prepared"}
        )
    )


if __name__ == "__main__":
    main()
