#!/usr/bin/env python3
"""Publish image directories or videos as raw RGB Cyber Image messages."""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


IMAGE_EXTENSIONS = {
    ".bmp", ".jpeg", ".jpg", ".png", ".ppm", ".tif", ".tiff", ".webp",
}


def probe_video(path):
    result = subprocess.run(
        [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=width,height,r_frame_rate", "-of", "json",
            str(path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"ffprobe failed for {path}: {result.stderr.strip()}")
    streams = json.loads(result.stdout).get("streams", [])
    if not streams:
        raise RuntimeError(f"no video stream found in {path}")
    stream = streams[0]
    return int(stream["width"]), int(stream["height"]), stream.get("r_frame_rate")


def decode_image(path):
    width, height, _ = probe_video(path)
    result = subprocess.run(
        [
            "ffmpeg", "-v", "error", "-i", str(path), "-frames:v", "1",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
        ],
        check=False,
        capture_output=True,
    )
    expected_size = width * height * 3
    if result.returncode != 0:
        raise RuntimeError(
            f"ffmpeg failed for {path}: "
            f"{result.stderr.decode(errors='replace').strip()}")
    if len(result.stdout) != expected_size:
        raise RuntimeError(
            f"decoded image size mismatch for {path}: "
            f"{len(result.stdout)} != {expected_size}")
    return width, height, result.stdout


def image_paths(directory, recursive, max_frames):
    iterator = (
        (Path(root) / name for root, _, names in os.walk(directory)
         for name in names)
        if recursive
        else (Path(entry.path) for entry in os.scandir(directory)
              if entry.is_file())
    )
    paths = []
    for path in iterator:
        if path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        paths.append(path)
        if max_frames and len(paths) >= max_frames:
            break
    return paths


def relative_path(path, dataset_root):
    if dataset_root is None:
        return path.name
    try:
        return str(path.resolve().relative_to(dataset_root.resolve()))
    except ValueError as error:
        raise RuntimeError(
            f"{path} is outside dataset root {dataset_root}") from error


def publish_message(writer, image_type, width, height, rgb, sequence,
                    timestamp_ns, frame_id):
    message = image_type()
    message.header.timestamp_sec = timestamp_ns / 1_000_000_000
    message.header.camera_timestamp = timestamp_ns
    message.header.module_name = "whl_image_message_publisher"
    message.header.sequence_num = sequence
    message.header.frame_id = frame_id
    message.frame_id = frame_id
    message.measurement_time = timestamp_ns / 1_000_000_000
    message.height = height
    message.width = width
    message.encoding = "rgb8"
    message.step = width * 3
    message.data = rgb
    writer.write(message)


def publish_images(args, writer, image_type, manifest):
    paths = image_paths(Path(args.input), args.recursive, args.max_frames)
    if not paths:
        raise RuntimeError(f"no supported images found in {args.input}")
    interval_ns = round(1_000_000_000 / args.fps)
    for index, path in enumerate(paths, start=1):
        width, height, rgb = decode_image(path)
        timestamp_ns = args.start_timestamp_ns + (index - 1) * interval_ns
        record = {
            "sequence_num": index,
            "camera_timestamp_ns": timestamp_ns,
            "source_path": str(path.resolve()),
            "relative_path": relative_path(path, args.dataset_root),
            "source_frame_index": index - 1,
            "width": width,
            "height": height,
        }
        manifest.write(json.dumps(record, sort_keys=True) + "\n")
        manifest.flush()
        publish_message(
            writer, image_type, width, height, rgb, index, timestamp_ns,
            args.frame_id)
        print(f"published {index}: {path} at {timestamp_ns} ns", flush=True)
        time.sleep(1.0 / args.fps)


def publish_video(args, writer, image_type, manifest):
    path = Path(args.input)
    width, height, source_rate = probe_video(path)
    fps = args.fps
    if fps is None:
        numerator, denominator = source_rate.split("/", maxsplit=1)
        fps = float(numerator) / float(denominator)
    frame_size = width * height * 3
    process = subprocess.Popen(
        [
            "ffmpeg", "-v", "error", "-i", str(path), "-f", "rawvideo",
            "-pix_fmt", "rgb24", "pipe:1",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    interval_ns = round(1_000_000_000 / fps)
    sequence = 1
    try:
        while not args.max_frames or sequence <= args.max_frames:
            rgb = process.stdout.read(frame_size)
            if not rgb:
                break
            if len(rgb) != frame_size:
                raise RuntimeError(
                    f"partial decoded video frame: {len(rgb)} != {frame_size}")
            timestamp_ns = (
                args.start_timestamp_ns + (sequence - 1) * interval_ns)
            virtual_path = f"{path.stem}/frame_{sequence:06d}.jpg"
            record = {
                "sequence_num": sequence,
                "camera_timestamp_ns": timestamp_ns,
                "source_path": str(path.resolve()),
                "relative_path": virtual_path,
                "source_frame_index": sequence - 1,
                "width": width,
                "height": height,
            }
            manifest.write(json.dumps(record, sort_keys=True) + "\n")
            manifest.flush()
            publish_message(
                writer, image_type, width, height, rgb, sequence, timestamp_ns,
                args.frame_id)
            print(
                f"published video frame {sequence} at {timestamp_ns} ns",
                flush=True)
            sequence += 1
            time.sleep(1.0 / fps)
    finally:
        process.terminate()
        process.wait()
    if sequence == 1:
        error = process.stderr.read().decode(errors="replace").strip()
        raise RuntimeError(f"no video frames decoded from {path}: {error}")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Publish an image directory or video as rgb8 Cyber Image messages "
            "and write a source-ID manifest for lane evaluation."))
    parser.add_argument("--input", required=True)
    parser.add_argument(
        "--channel", default="/apollo/sensor/camera/front_6mm/image")
    parser.add_argument("--frame-id", default="camera_front_6mm")
    parser.add_argument("--fps", type=float)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--startup-wait-sec", type=float, default=3.0)
    parser.add_argument("--start-timestamp-ns", type=int, default=0)
    parser.add_argument("--manifest", default="image_manifest.jsonl")
    parser.add_argument("--dataset-root", type=Path)
    parser.add_argument("--recursive", action="store_true")
    args = parser.parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        parser.error(f"input does not exist: {input_path}")
    if args.fps is None and input_path.is_dir():
        args.fps = 10.0
    if args.fps is not None and args.fps <= 0:
        parser.error("--fps must be positive")
    if args.max_frames < 0 or args.startup_wait_sec < 0:
        parser.error("--max-frames and --startup-wait-sec must be non-negative")
    if args.start_timestamp_ns < 0:
        parser.error("--start-timestamp-ns must be non-negative")
    if args.start_timestamp_ns == 0:
        args.start_timestamp_ns = time.time_ns()
    return args


def main():
    args = parse_args()
    if shutil.which("ffmpeg") is None or shutil.which("ffprobe") is None:
        print("ffmpeg and ffprobe are required", file=sys.stderr)
        return 2
    try:
        from cyber.python.cyber_py3 import cyber
        from wheelos_msgs.sensor_msgs import sensor_image_pb2
    except ImportError as error:
        print(f"Cyber runtime imports failed: {error}", file=sys.stderr)
        return 2

    manifest_path = Path(args.manifest)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with manifest_path.open("x", encoding="utf-8") as manifest:
            cyber.init("whl_image_message_publisher")
            node = cyber.Node("whl_image_message_publisher")
            writer = node.create_writer(
                args.channel, sensor_image_pb2.Image, 8)
            time.sleep(args.startup_wait_sec)
            if Path(args.input).is_dir():
                publish_images(args, writer, sensor_image_pb2.Image, manifest)
            else:
                publish_video(args, writer, sensor_image_pb2.Image, manifest)
    except (OSError, RuntimeError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1
    finally:
        if "cyber" in locals():
            cyber.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
