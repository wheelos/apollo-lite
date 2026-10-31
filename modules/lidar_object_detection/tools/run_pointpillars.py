#!/usr/bin/env python3
"""Build the fixed KITTI engine or infer packed little-endian XYZI frames."""

import argparse
import ctypes
import hashlib
import json
import math
import struct
import time
from pathlib import Path


class Detection(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("dx", ctypes.c_float),
        ("dy", ctypes.c_float),
        ("dz", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("class_id", ctypes.c_int32),
        ("score", ctypes.c_float),
    ]


def read_points(path):
    data = Path(path).read_bytes()
    if not data or len(data) % 16 != 0 or len(data) > 300000 * 16:
        raise ValueError("Input must contain 1..300000 packed float32 XYZI rows")
    values = struct.unpack(f"<{len(data) // 4}f", data)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Input contains nonfinite XYZI features")
    return (ctypes.c_float * len(values))(*values)


def load_runtime(path):
    runtime = ctypes.CDLL(str(Path(path).resolve()))
    runtime.pointpillars_runtime_abi_version.restype = ctypes.c_uint32
    runtime.pointpillars_runtime_detection_size.restype = ctypes.c_size_t
    if (
        runtime.pointpillars_runtime_abi_version() != 1
        or runtime.pointpillars_runtime_detection_size() != ctypes.sizeof(Detection)
    ):
        raise RuntimeError("PointPillars runtime ABI mismatch")
    runtime.pointpillars_runtime_build_engine.argtypes = [
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    runtime.pointpillars_runtime_build_engine.restype = ctypes.c_bool
    runtime.pointpillars_runtime_create.argtypes = [
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    runtime.pointpillars_runtime_create.restype = ctypes.c_void_p
    runtime.pointpillars_runtime_destroy.argtypes = [ctypes.c_void_p]
    runtime.pointpillars_runtime_destroy.restype = None
    runtime.pointpillars_runtime_infer.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_uint32,
        ctypes.POINTER(Detection),
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.c_char_p,
        ctypes.c_size_t,
    ]
    runtime.pointpillars_runtime_infer.restype = ctypes.c_bool
    return runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", required=True)
    parser.add_argument("--engine", required=True)
    parser.add_argument("--gpu", type=int, default=0)
    parser.add_argument("--build-onnx")
    parser.add_argument("--input", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if bool(args.build_onnx) == bool(args.input):
        parser.error("Choose --build-onnx or --input")
    if args.input and args.output is None:
        parser.error("--input requires --output")
    runtime = load_runtime(args.runtime)
    error = ctypes.create_string_buffer(4096)
    engine = str(Path(args.engine).resolve()).encode()
    if args.build_onnx:
        if Path(args.engine).exists():
            parser.error("Refusing to overwrite an existing engine")
        if not runtime.pointpillars_runtime_build_engine(
            str(Path(args.build_onnx).resolve()).encode(),
            engine,
            args.gpu,
            error,
            len(error),
        ):
            raise RuntimeError(error.value.decode())
        return
    handle = runtime.pointpillars_runtime_create(
        engine, args.gpu, error, len(error)
    )
    if not handle:
        raise RuntimeError(error.value.decode())
    engine_sha256 = hashlib.sha256(Path(args.engine).read_bytes()).hexdigest()
    try:
        with args.output.open("x", encoding="utf-8") as output:
            for path in args.input:
                points = read_points(path)
                boxes = (Detection * 40000)()
                count = ctypes.c_size_t()
                start = time.perf_counter()
                if not runtime.pointpillars_runtime_infer(
                    handle, points, len(points) // 4, boxes, len(boxes),
                    ctypes.byref(count), error, len(error),
                ):
                    raise RuntimeError(f"{path}: {error.value.decode()}")
                inference_ms = (time.perf_counter() - start) * 1000
                record = {
                    "source_path": str(path.resolve()),
                    "engine_path": engine.decode(),
                    "engine_sha256": engine_sha256,
                    "input_point_count": len(points) // 4,
                    "inference_ms": inference_ms,
                    "coordinate_frame": "input_lidar",
                    "box_convention": "geometric_center_xyz_size_xyz_yaw_ccw",
                    "class_names": ["Car", "Pedestrian", "Cyclist"],
                    "detections": [
                        {name: getattr(boxes[i], name) for name, _ in Detection._fields_}
                        for i in range(count.value)
                    ],
                }
                output.write(json.dumps(record, allow_nan=False) + "\n")
                output.flush()
    finally:
        runtime.pointpillars_runtime_destroy(handle)


if __name__ == "__main__":
    main()
