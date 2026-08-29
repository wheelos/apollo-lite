#!/usr/bin/env python3
"""Save lane overlays only when Image and PerceptionLanes share one timestamp."""

import argparse
import json
import signal
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path


LANE_COLORS = (
    (255, 64, 64),
    (64, 255, 64),
    (64, 128, 255),
    (255, 220, 64),
    (255, 64, 220),
    (64, 255, 255),
)
DIGITS = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("111", "001", "111", "100", "111"),
    "3": ("111", "001", "111", "001", "111"),
    "4": ("101", "101", "111", "001", "001"),
    "5": ("111", "100", "111", "001", "111"),
    "6": ("111", "100", "111", "101", "111"),
    "7": ("111", "001", "010", "010", "010"),
    "8": ("111", "101", "111", "101", "111"),
    "9": ("111", "101", "111", "001", "111"),
    "-": ("000", "000", "111", "000", "000"),
}


def timestamp_ns(timestamp_sec):
    return int(timestamp_sec * 1_000_000_000)


def set_pixel(pixels, width, height, x, y, color):
    if 0 <= x < width and 0 <= y < height:
        offset = (y * width + x) * 3
        pixels[offset:offset + 3] = bytes(color)


def draw_disc(pixels, width, height, x, y, color, radius=3):
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                set_pixel(pixels, width, height, x + dx, y + dy, color)


def draw_line(pixels, width, height, start, end, color):
    x0, y0 = start
    x1, y1 = end
    steps = max(abs(x1 - x0), abs(y1 - y0))
    if steps == 0:
        draw_disc(pixels, width, height, x0, y0, color)
        return
    for step in range(steps + 1):
        ratio = step / steps
        draw_disc(
            pixels,
            width,
            height,
            round(x0 + (x1 - x0) * ratio),
            round(y0 + (y1 - y0) * ratio),
            color,
            radius=2,
        )


def draw_text(pixels, width, height, x, y, text, color):
    cursor = x
    for character in text:
        glyph = DIGITS.get(character)
        if glyph is None:
            cursor += 4
            continue
        for row, bits in enumerate(glyph):
            for column, bit in enumerate(bits):
                if bit == "1":
                    for dy in range(2):
                        for dx in range(2):
                            set_pixel(
                                pixels, width, height, cursor + column * 2 + dx,
                                y + row * 2 + dy, color)
        cursor += 8


def image_to_rgb(image):
    expected_size = image.width * image.height * 3
    if image.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding: {image.encoding}")
    if image.step != image.width * 3:
        raise ValueError(f"unexpected image stride: {image.step}")
    if len(image.data) != expected_size:
        raise ValueError(
            f"unexpected image payload size: {len(image.data)} != {expected_size}")
    if image.encoding == "rgb8":
        return bytearray(image.data)

    rgb = bytearray(expected_size)
    for offset in range(0, expected_size, 3):
        rgb[offset] = image.data[offset + 2]
        rgb[offset + 1] = image.data[offset + 1]
        rgb[offset + 2] = image.data[offset]
    return rgb


def render_overlay(image, lanes, output_path):
    pixels = image_to_rgb(image)
    for index, lane in enumerate(lanes.camera_laneline):
        color = LANE_COLORS[index % len(LANE_COLORS)]
        points = [
            (round(point.x), round(point.y))
            for point in lane.curve_image_point_set
        ]
        for point in points:
            draw_disc(pixels, image.width, image.height, point[0], point[1], color)
        for start, end in zip(points, points[1:]):
            draw_line(pixels, image.width, image.height, start, end, color)
        if points:
            draw_text(
                pixels, image.width, image.height, points[0][0] + 6,
                points[0][1] + 6, str(lane.track_id), color)

    with output_path.open("xb") as output:
        output.write(f"P6\n{image.width} {image.height}\n255\n".encode("ascii"))
        output.write(pixels)


def optional_header_value(message, field_name):
    if not message.HasField("header"):
        return None
    header = message.header
    if not header.HasField(field_name):
        return None
    return getattr(header, field_name)


class LaneDebugVisualizer:
    def __init__(self, args, image_type, lane_type, cyber):
        self.args = args
        self.image_type = image_type
        self.lane_type = lane_type
        self.cyber = cyber
        self.images = OrderedDict()
        self.lanes = OrderedDict()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.saved_count = 0
        self.output_dir = Path(args.output_dir)
        self.manifest_path = self.output_dir / "manifest.jsonl"

    def _store(self, cache, key, message):
        cache[key] = message
        cache.move_to_end(key)
        while len(cache) > self.args.max_pending:
            cache.popitem(last=False)

    def _take_pair(self, key, image=None, lanes=None):
        with self.lock:
            if image is not None:
                counterpart = self.lanes.pop(key, None)
                if counterpart is None:
                    self._store(self.images, key, image)
                    return None
                return image, counterpart
            counterpart = self.images.pop(key, None)
            if counterpart is None:
                self._store(self.lanes, key, lanes)
                return None
            return counterpart, lanes

    def _save_pair(self, key, image, lanes):
        with self.lock:
            if self.saved_count >= self.args.max_frames:
                self.stop_event.set()
                return
            self.saved_count += 1
        output_path = self.output_dir / f"lane_{key}.ppm"
        try:
            render_overlay(image, lanes, output_path)
        except FileExistsError:
            print(f"refusing to overwrite existing overlay: {output_path}", file=sys.stderr)
            self.stop_event.set()
            return
        except ValueError as error:
            print(f"skipping timestamp {key}: {error}", file=sys.stderr)
            return
        record = {
            "camera_timestamp_ns": key,
            "image_header_camera_timestamp_ns": optional_header_value(
                image, "camera_timestamp"),
            "image_header_sequence_num": optional_header_value(
                image, "sequence_num"),
            "image_header_frame_id": optional_header_value(image, "frame_id"),
            "image_measurement_time_ns": timestamp_ns(image.measurement_time),
            "lane_camera_timestamp_ns": lanes.header.camera_timestamp,
            "lane_header_sequence_num": optional_header_value(
                lanes, "sequence_num"),
            "lane_header_frame_id": optional_header_value(lanes, "frame_id"),
            "image_frame_id": image.frame_id,
            "lane_count": len(lanes.camera_laneline),
            "path": str(output_path),
        }
        with self.manifest_path.open("a", encoding="utf-8") as manifest:
            manifest.write(json.dumps(record, sort_keys=True) + "\n")
        print(f"saved {output_path} ({record['lane_count']} lanes)", flush=True)

    def on_image(self, image):
        key = timestamp_ns(image.measurement_time)
        pair = self._take_pair(key, image=image)
        if pair is not None:
            self._save_pair(key, *pair)

    def on_lanes(self, lanes):
        if not lanes.HasField("header") or not lanes.header.HasField("camera_timestamp"):
            print("skipping lane message without camera_timestamp", file=sys.stderr)
            return
        key = lanes.header.camera_timestamp
        pair = self._take_pair(key, lanes=lanes)
        if pair is not None:
            self._save_pair(key, *pair)

    def run(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.cyber.init()
        node = self.cyber.Node(self.args.node_name)
        node.create_reader(self.args.image_channel, self.image_type, self.on_image)
        node.create_reader(self.args.lane_channel, self.lane_type, self.on_lanes)
        print(
            f"subscribed to {self.args.image_channel} and {self.args.lane_channel}; "
            f"strict timestamp matching, saving at most {self.args.max_frames} overlays",
            flush=True,
        )
        try:
            while not self.stop_event.is_set() and not self.cyber.is_shutdown():
                time.sleep(0.01)
        finally:
            self.cyber.shutdown()
        return 0


def run_self_test(output_dir):
    class Image:
        width = 64
        height = 48
        step = 192
        encoding = "rgb8"
        data = bytes([32, 32, 32]) * (width * height)

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    class Lane:
        track_id = 7
        curve_image_point_set = [Point(20, 42), Point(25, 32), Point(30, 22)]

    class Lanes:
        camera_laneline = [Lane()]

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    output_path = destination / "lane_42.ppm"
    render_overlay(Image(), Lanes(), output_path)
    print(f"self-test overlay saved to {output_path}")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pair Image and PerceptionLanes by identical camera timestamp and save PPM overlays.")
    parser.add_argument("--image-channel", default="/apollo/sensor/camera/front_6mm/image")
    parser.add_argument("--lane-channel", default="/perception/lanes")
    parser.add_argument("--output-dir", default="lane_debug_output")
    parser.add_argument("--max-frames", type=int, default=50)
    parser.add_argument("--max-pending", type=int, default=256)
    parser.add_argument("--node-name", default="whl_lane_debug_visualizer")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.max_frames <= 0 or args.max_pending <= 0:
        parser.error("--max-frames and --max-pending must be positive")
    return args


def main():
    args = parse_args()
    if args.self_test:
        return run_self_test(args.output_dir)
    try:
        from cyber.python.cyber_py3 import cyber
        from wheelos_msgs.perception_msgs import perception_lane_pb2
        from wheelos_msgs.sensor_msgs import sensor_image_pb2
    except ImportError as error:
        print(
            f"Cyber runtime imports failed: {error}. Run 'source scripts/runtime_env.sh' "
            "from the Apollo workspace first.",
            file=sys.stderr,
        )
        return 2
    return LaneDebugVisualizer(
        args, sensor_image_pb2.Image, perception_lane_pb2.PerceptionLanes, cyber).run()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    sys.exit(main())
