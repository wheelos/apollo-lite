#!/usr/bin/env python3
"""Save camera semantic segmentation overlays paired with Cyber Image messages."""

import argparse
import json
import signal
import sys
import threading
import time
from collections import OrderedDict
from pathlib import Path


CITYSCAPES_PALETTE = (
    (128, 64, 128),   # 0: road
    (244, 35, 232),   # 1: sidewalk
    (70, 70, 70),     # 2: building
    (102, 102, 156),  # 3: wall
    (190, 153, 153),  # 4: fence
    (153, 153, 153),  # 5: pole
    (250, 170, 30),   # 6: traffic light
    (220, 220, 0),    # 7: traffic sign
    (107, 142, 35),   # 8: vegetation
    (152, 251, 152),  # 9: terrain
    (70, 130, 180),   # 10: sky
    (220, 20, 60),    # 11: person
    (255, 0, 0),      # 12: rider
    (0, 0, 142),      # 13: car
    (0, 0, 70),       # 14: truck
    (0, 60, 100),     # 15: bus
    (0, 80, 100),     # 16: train
    (0, 0, 230),      # 17: motorcycle
    (119, 11, 32),    # 18: bicycle
)


def timestamp_ns(timestamp_sec):
    return int(timestamp_sec * 1_000_000_000)


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


def get_mask_labels(result):
    if len(result.mask) == result.width * result.height:
        return result.mask
    if len(result.pixel_labels) == result.width * result.height:
        return bytes(result.pixel_labels)
    raise ValueError(
        f"mask length mismatch: mask={len(result.mask)}, "
        f"pixel_labels={len(result.pixel_labels)}, "
        f"expected={result.width * result.height}")


def render_overlay(image, result, output_path, alpha=0.5):
    img_w, img_h = image.width, image.height
    mask_w, mask_h = result.width, result.height
    labels = get_mask_labels(result)

    rgb = image_to_rgb(image)
    overlay = bytearray(img_w * img_h * 3)

    scale_x = mask_w / img_w
    scale_y = mask_h / img_h

    for y in range(img_h):
        mask_y = min(int(y * scale_y), mask_h - 1)
        row_offset = y * img_w * 3
        mask_row_offset = mask_y * mask_w

        for x in range(img_w):
            mask_x = min(int(x * scale_x), mask_w - 1)
            cls_id = labels[mask_row_offset + mask_x]

            if cls_id < len(CITYSCAPES_PALETTE):
                color = CITYSCAPES_PALETTE[cls_id]
            else:
                color = (
                    (cls_id * 37 + 17) % 256,
                    (cls_id * 67 + 53) % 256,
                    (cls_id * 101 + 97) % 256,
                )

            pix_offset = row_offset + x * 3
            r = int(rgb[pix_offset] * (1.0 - alpha) + color[0] * alpha)
            g = int(rgb[pix_offset + 1] * (1.0 - alpha) + color[1] * alpha)
            b = int(rgb[pix_offset + 2] * (1.0 - alpha) + color[2] * alpha)

            overlay[pix_offset] = r
            overlay[pix_offset + 1] = g
            overlay[pix_offset + 2] = b

    with output_path.open("xb") as output:
        output.write(f"P6\n{img_w} {img_h}\n255\n".encode("ascii"))
        output.write(overlay)


def optional_header_value(message, field_name):
    if not message.HasField("header"):
        return None
    header = message.header
    if not header.HasField(field_name):
        return None
    return getattr(header, field_name)


class SourceManifest:
    def __init__(self, path):
        self.path = Path(path) if path else None
        self.offset = 0
        self.by_sequence = {}

    def lookup(self, sequence_num):
        if self.path is None:
            return None
        if self.path.exists():
            with self.path.open(encoding="utf-8") as source:
                source.seek(self.offset)
                while True:
                    line = source.readline()
                    if not line:
                        break
                    record = json.loads(line)
                    self.by_sequence[record["sequence_num"]] = record
                self.offset = source.tell()
        return self.by_sequence.get(sequence_num)


class CameraSemanticSegmentationVisualizer:
    def __init__(self, args, image_type, result_type, cyber):
        self.args = args
        self.image_type = image_type
        self.result_type = result_type
        self.cyber = cyber
        self.images = OrderedDict()
        self.results = OrderedDict()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.saved_count = 0
        self.output_dir = Path(args.output_dir)
        self.manifest_path = self.output_dir / "manifest.jsonl"
        self.source_manifest = SourceManifest(args.source_manifest)

    def _store(self, cache, key, message):
        cache[key] = message
        cache.move_to_end(key)
        while len(cache) > self.args.max_pending:
            cache.popitem(last=False)

    def _take_pair(self, key, image=None, result=None):
        with self.lock:
            if image is not None:
                counterpart = self.results.pop(key, None)
                if counterpart is None:
                    self._store(self.images, key, image)
                    return None
                return image, counterpart
            counterpart = self.images.pop(key, None)
            if counterpart is None:
                self._store(self.results, key, result)
                return None
            return counterpart, result

    def _save_pair(self, key, image, result):
        with self.lock:
            if self.saved_count >= self.args.max_frames:
                self.stop_event.set()
                return
            self.saved_count += 1
        output_path = None
        if not self.args.no_overlay:
            output_path = self.output_dir / f"seg_{key}.ppm"
            try:
                render_overlay(image, result, output_path, alpha=self.args.alpha)
            except FileExistsError:
                print(
                    f"refusing to overwrite existing overlay: {output_path}",
                    file=sys.stderr)
                self.stop_event.set()
                return
            except ValueError as error:
                print(f"skipping timestamp {key}: {error}", file=sys.stderr)
                return

        sequence_num = optional_header_value(result, "sequence_num")
        source = self.source_manifest.lookup(sequence_num)

        record = {
            "camera_timestamp_ns": key,
            "image_header_camera_timestamp_ns": optional_header_value(
                image, "camera_timestamp"),
            "image_header_sequence_num": optional_header_value(
                image, "sequence_num"),
            "image_measurement_time_ns": timestamp_ns(image.measurement_time),
            "result_camera_timestamp_ns": optional_header_value(
                result, "camera_timestamp"),
            "result_sequence_num": sequence_num,
            "mask_width": result.width,
            "mask_height": result.height,
            "num_classes": result.num_classes,
            "overlay_path": str(output_path) if output_path is not None else None,
            "source_path": source["source_path"] if source else None,
            "relative_path": source["relative_path"] if source else None,
        }
        with self.manifest_path.open("a", encoding="utf-8") as manifest:
            manifest.write(json.dumps(record, sort_keys=True) + "\n")
        print(
            f"saved segmentation pair {key} (mask {result.width}x{result.height}"
            f"{', no overlay' if output_path is None else ''})",
            flush=True)

    def on_image(self, image):
        key = None
        if image.HasField("header") and image.header.HasField("camera_timestamp") and image.header.camera_timestamp > 0:
            key = image.header.camera_timestamp
        elif image.HasField("measurement_time"):
            key = timestamp_ns(image.measurement_time)
        if key is None:
            return
        pair = self._take_pair(key, image=image)
        if pair is not None:
            self._save_pair(key, *pair)

    def on_result(self, result):
        key = None
        if result.HasField("header") and result.header.HasField("camera_timestamp") and result.header.camera_timestamp > 0:
            key = result.header.camera_timestamp
        elif result.HasField("header") and result.header.HasField("timestamp_sec"):
            key = timestamp_ns(result.header.timestamp_sec)
        if key is None:
            print("skipping result message without camera_timestamp", file=sys.stderr)
            return
        pair = self._take_pair(key, result=result)
        if pair is not None:
            self._save_pair(key, *pair)

    def run(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.cyber.init()
        node = self.cyber.Node(self.args.node_name)
        node.create_reader(self.args.image_channel, self.image_type, self.on_image)
        node.create_reader(self.args.segmentation_channel, self.result_type, self.on_result)
        print(
            f"subscribed to {self.args.image_channel} and {self.args.segmentation_channel}; "
            f"saving at most {self.args.max_frames} overlays",
            flush=True,
        )
        try:
            while not self.stop_event.is_set() and not self.cyber.is_shutdown():
                time.sleep(0.01)
        finally:
            self.cyber.shutdown()
        return 0


def run_self_test(output_dir):
    class Header:
        camera_timestamp = 1000
        sequence_num = 1

        def HasField(self, name):
            return True

    class Image:
        width = 64
        height = 48
        step = 192
        encoding = "rgb8"
        data = bytes([128, 128, 128]) * (width * height)
        measurement_time = 0.000001
        header = Header()

        def HasField(self, name):
            return True

    class SegmentationResult:
        width = 64
        height = 48
        num_classes = 19
        mask = bytes([0] * (width * height // 2) + [13] * (width * height // 2))
        pixel_labels = []
        header = Header()

        def HasField(self, name):
            return True

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    output_path = destination / "seg_self_test.ppm"
    render_overlay(Image(), SegmentationResult(), output_path)
    print(f"self-test overlay saved to {output_path}")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pair Image and CameraSemanticSegmentationResult by timestamp and save colorized PPM overlays.")
    parser.add_argument("--image-channel", default="/apollo/sensor/camera/front_6mm/image")
    parser.add_argument("--segmentation-channel", default="/perception/camera_semantic_segmentation")
    parser.add_argument("--output-dir", default="camera_seg_debug_output")
    parser.add_argument("--max-frames", type=int, default=50)
    parser.add_argument("--max-pending", type=int, default=256)
    parser.add_argument("--alpha", type=float, default=0.5)
    parser.add_argument("--node-name", default="whl_camera_seg_visualizer")
    parser.add_argument(
        "--source-manifest",
        help="Publisher manifest used to recover source dataset paths.")
    parser.add_argument(
        "--no-overlay", action="store_true",
        help="Export paired predictions without rendering PPM overlays.")
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
        from modules.camera_semantic_segmentation.proto import camera_semantic_segmentation_pb2
        from wheelos_msgs.sensor_msgs import sensor_image_pb2
    except ImportError as error:
        print(
            f"Cyber runtime imports failed: {error}. Run 'source scripts/runtime_env.sh' "
            "from the Apollo workspace first.",
            file=sys.stderr,
        )
        return 2
    return CameraSemanticSegmentationVisualizer(
        args, sensor_image_pb2.Image,
        camera_semantic_segmentation_pb2.CameraSemanticSegmentationResult,
        cyber).run()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    sys.exit(main())
