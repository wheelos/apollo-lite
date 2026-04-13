from __future__ import annotations

import json
import os
import queue
import shlex
import shutil
import subprocess
import threading
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Dict, List

from flask import Response, abort, jsonify, request, send_file

try:
    from .container_runtime import (
        REPO_ROOT,
        container_binary_supports_flag,
        container_command,
        ensure_container_binary,
        host_to_container,
        run_container_script,
        wait_for_file,
    )
    from .job_manager import JobContext, JobManager
    from .record_utils import (
        copy_tree,
        count_topic_messages,
        extract_pointcloud_dataset,
        inspect_records,
    )
except ImportError:  # pragma: no cover
    from container_runtime import (
        REPO_ROOT,
        container_binary_supports_flag,
        container_command,
        ensure_container_binary,
        host_to_container,
        run_container_script,
        wait_for_file,
    )
    from job_manager import JobContext, JobManager
    from record_utils import copy_tree, count_topic_messages, extract_pointcloud_dataset, inspect_records


def _first_existing(paths: List[str]) -> str:
    for path in paths:
        if (REPO_ROOT / path).exists():
            return path
    return ""


def _extract_first_channel_from_dag(dag_path: Path) -> str:
    if not dag_path.exists():
        return ""
    for raw_line in dag_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line.startswith("channel:"):
            continue
        return line.split(":", 1)[1].strip().strip('"')
    return ""


def _populate_perception_viewer_assets(viewer_dir: Path) -> None:
    viewer_src = REPO_ROOT / "modules/perception/lidar/tools/web_viewer"
    if not viewer_src.exists():
        raise RuntimeError(f"missing perception web viewer assets: {viewer_src}")
    for name in ("index.html", "app.js", "styles.css"):
        source = viewer_src / name
        if not source.exists():
            raise RuntimeError(f"missing perception web viewer file: {source}")
        shutil.copy2(source, viewer_dir / name)


class ToolboxPlugin(ABC):
    plugin_id: str
    name: str
    description: str

    def __init__(self, job_manager: JobManager):
        self.job_manager = job_manager

    @abstractmethod
    def probe(self) -> Dict[str, Any]:
        raise NotImplementedError

    @abstractmethod
    def actions(self) -> List[Dict[str, Any]]:
        raise NotImplementedError

    def metadata(self) -> Dict[str, Any]:
        return {
            "plugin_id": self.plugin_id,
            "name": self.name,
            "description": self.description,
            "probe": self.probe(),
            "actions": self.actions(),
        }

    def register_routes(self, app) -> None:
        return None

    def inspect_record(self, action_id: str, raw_path: str) -> Dict[str, Any]:
        return inspect_records(raw_path)

    @abstractmethod
    def run_action(self, action_id: str, params: Dict[str, Any], ctx: JobContext) -> None:
        raise NotImplementedError


class PerceptionLidarPlugin(ToolboxPlugin):
    plugin_id = "perception_lidar"
    name = "Perception LiDAR"
    description = "Record to offline obstacle perception and frame-by-frame visualization, with optional GT comparison."
    default_pose_topic = "/apollo/localization/pose"

    def probe(self) -> Dict[str, Any]:
        offline_src = (REPO_ROOT / "modules/perception/lidar/tools/offline_lidar_obstacle_perception.cc").exists()
        web_exporter_src = (REPO_ROOT / "modules/perception/tool/benchmark/lidar/lidar_web_visualizer_exporter.cc").exists()
        benchmark_src = (REPO_ROOT / "modules/perception/tool/benchmark/lidar/lidar_detection_benchmark.cc").exists()
        return {
            "available": offline_src and web_exporter_src,
            "actions": {
                "visualize_result": offline_src and web_exporter_src,
            },
            "missing": [],
            "warnings": [] if benchmark_src else ["GT compare is unavailable because lidar_benchmark source is missing."],
        }

    def actions(self) -> List[Dict[str, Any]]:
        benchmark_available = (
            REPO_ROOT / "modules/perception/tool/benchmark/lidar/lidar_detection_benchmark.cc"
        ).exists()
        detection_default = _first_existing(
            [
                "modules/perception/pipeline/config/lidar_detection_background_pipeline_trt.pb.txt",
                "modules/perception/pipeline/config/lidar_detection_pipeline_trt.pb.txt",
                "modules/perception/pipeline/config/lidar_detection_pipeline.pb.txt",
            ]
        )
        tracking_default = _first_existing(
            ["modules/perception/pipeline/config/lidar_tracking_pipeline.pb.txt"]
        )
        return [
            {
                "action_id": "visualize_result",
                "title": "Run And Visualize",
                "description": "Toolbox manages the output workspace. When finished, use the reported result directory as future GT if needed.",
                "fields": [
                    {
                        "name": "data_package",
                        "label": "Data Package",
                        "type": "path",
                        "required": True,
                    },
                    {
                        "name": "pointcloud_topic",
                        "label": "PointCloud Topic",
                        "type": "text",
                        "required": True,
                    },
                    {
                        "name": "sensor_name",
                        "label": "Sensor Name",
                        "type": "text",
                        "default": "velodyne64",
                        "required": True,
                    },
                    {
                        "name": "detection_conf",
                        "label": "Detection Config",
                        "type": "path",
                        "default": detection_default,
                    },
                    {
                        "name": "tracking_conf",
                        "label": "Tracking Config",
                        "type": "path",
                        "default": tracking_default,
                    },
                    {"name": "max_points", "label": "Max Points", "type": "number", "default": 30000},
                    *(
                        [
                            {
                                "name": "use_gt",
                                "label": "Compare With GT",
                                "type": "checkbox",
                                "default": False,
                            },
                            {
                                "name": "baseline_result_dir",
                                "label": "Baseline Result Dir",
                                "type": "path",
                                "required": True,
                                "visible_when": {"field": "use_gt", "equals": True},
                            },
                            {
                                "name": "reserve",
                                "label": "Benchmark Reserve",
                                "type": "text",
                                "default": "JACCARD:0.9",
                                "visible_when": {"field": "use_gt", "equals": True},
                            },
                        ]
                        if benchmark_available
                        else []
                    ),
                ],
            },
        ]

    def _resolve_topic(self, raw_path: str, requested_topic: str, *, pointcloud: bool) -> str:
        if requested_topic:
            return requested_topic
        inspect_result = inspect_records(raw_path)
        topics = inspect_result["pointcloud_topics" if pointcloud else "localization_topics"]
        return topics[0] if topics else ""

    def _prepare_lists(self, pcd_dir: Path, result_dir: Path, gt_dir: Path, lists_dir: Path) -> tuple[Path, Path, Path]:
        lists_dir.mkdir(parents=True, exist_ok=True)
        cloud_list = lists_dir / "cloud.list"
        result_list = lists_dir / "result.list"
        gt_list = lists_dir / "groundtruth.list"
        with cloud_list.open("w", encoding="utf-8") as fout_cloud, \
            result_list.open("w", encoding="utf-8") as fout_result, \
            gt_list.open("w", encoding="utf-8") as fout_gt:
            for pcd_path in sorted(pcd_dir.glob("*.pcd")):
                stem = pcd_path.stem
                result_path = result_dir / f"{stem}.txt"
                gt_path = gt_dir / f"{stem}.txt"
                if not result_path.exists() or not gt_path.exists():
                    continue
                fout_cloud.write(f"{pcd_path}\n")
                fout_result.write(f"{result_path}\n")
                fout_gt.write(f"{gt_path}\n")
        return cloud_list, result_list, gt_list

    def run_action(self, action_id: str, params: Dict[str, Any], ctx: JobContext) -> None:
        if action_id != "visualize_result":
            raise RuntimeError(f"unsupported action: {action_id}")
        raw_path = params["data_package"]
        pointcloud_topic = str(params.get("pointcloud_topic") or "").strip()
        if not pointcloud_topic:
            raise RuntimeError("pointcloud_topic is required")
        inspect_result = inspect_records(raw_path)
        localization_topics = inspect_result.get("localization_topics", [])
        pose_topic = (
            self.default_pose_topic
            if self.default_pose_topic in localization_topics
            else self._resolve_topic(raw_path, "", pointcloud=False)
        )

        extract_dir = ctx.workspace / "extract"
        result_dir = ctx.workspace / "result"
        viewer_dir = ctx.workspace / "viewer"
        logs_dir = ctx.workspace / "logs"
        progress_file = ctx.workspace / "offline_progress.json"
        extract_dir.mkdir(parents=True, exist_ok=True)
        result_dir.mkdir(parents=True, exist_ok=True)
        logs_dir.mkdir(parents=True, exist_ok=True)

        ctx.set_progress(1.0, "inspect", f"pointcloud topic: {pointcloud_topic}")

        def _extract_progress(current: int, total: int, message: str) -> None:
            percent = 5.0 if total == 0 else 5.0 + 20.0 * float(current) / float(total)
            ctx.set_progress(percent, "extract", message)

        extract_summary = extract_pointcloud_dataset(
            raw_path,
            pointcloud_topic,
            extract_dir,
            pose_topic=pose_topic,
            progress_cb=_extract_progress,
        )
        ctx.log(json.dumps(extract_summary, ensure_ascii=False))

        offline_bin = ensure_container_binary(
            "//modules/perception/lidar/tools:offline_lidar_obstacle_perception", ctx
        )
        supports_progress_file = container_binary_supports_flag(offline_bin, "--progress_file")
        detection_conf = params.get("detection_conf") or _first_existing(
            ["modules/perception/pipeline/config/lidar_detection_pipeline_trt.pb.txt"]
        )
        tracking_conf = params.get("tracking_conf") or _first_existing(
            ["modules/perception/pipeline/config/lidar_tracking_pipeline.pb.txt"]
        )
        if not detection_conf or not tracking_conf:
            raise RuntimeError("missing detection_conf or tracking_conf")
        flagfile = _first_existing(["modules/common/data/global_flagfile.txt"])
        if not flagfile:
            raise RuntimeError("missing global flagfile")

        ctx.set_progress(25.0, "infer", "running offline lidar obstacle perception")
        offline_parts = [
            shlex.quote(offline_bin),
            f"--flagfile={shlex.quote(host_to_container(REPO_ROOT / flagfile))}",
            f"--pcd_path={shlex.quote(host_to_container(extract_dir / 'pcd'))}",
            f"--pose_path={shlex.quote(host_to_container(extract_dir / 'pose'))}",
            f"--output_path={shlex.quote(host_to_container(result_dir))}",
            f"--sensor_name={shlex.quote(params.get('sensor_name', 'velodyne64'))}",
            f"--lidar_detection_config_file={shlex.quote(host_to_container(REPO_ROOT / detection_conf))}",
            f"--lidar_tracking_config_file={shlex.quote(host_to_container(REPO_ROOT / tracking_conf))}",
            f"--enable_tracking={'true' if pose_topic else 'false'}",
        ]
        if supports_progress_file:
            offline_parts.append(
                f"--progress_file={shlex.quote(host_to_container(progress_file))}"
            )
        run_container_script(
            " ".join(offline_parts),
            job=ctx,
            progress_file=progress_file if supports_progress_file else None,
        )

        summary: Dict[str, Any] = {
            "result_dir": str(result_dir),
            "extract_summary": extract_summary,
            "pointcloud_topic": pointcloud_topic,
            "pose_topic": pose_topic,
        }
        ctx.add_artifact("result", "Result Directory", result_dir)
        max_points = int(params.get("max_points") or 30000)
        exporter_bin = ensure_container_binary(
            "//modules/perception/tool/benchmark/lidar:lidar_web_visualizer_exporter",
            ctx,
        )
        gt_enabled = bool(params.get("use_gt"))
        gt_dir = None
        lists_dir = ctx.workspace / "lists"
        reserve = str(params.get("reserve") or "").strip()
        if gt_enabled:
            baseline_value = str(params.get("baseline_result_dir") or "").strip()
            if not baseline_value:
                raise RuntimeError("baseline_result_dir is required when Compare With GT is enabled")
            baseline_src = Path(baseline_value).expanduser().resolve()
            baseline_dir = ctx.workspace / "baseline"
            if baseline_src != baseline_dir:
                copy_tree(baseline_src, baseline_dir)
            gt_dir = baseline_dir
            benchmark_bin = ensure_container_binary(
                "//modules/perception/tool/benchmark/lidar:lidar_benchmark", ctx
            )
            cloud_list, result_list, gt_list = self._prepare_lists(
                extract_dir / "pcd", result_dir, gt_dir, lists_dir
            )
            ctx.set_progress(80.0, "benchmark", "running lidar benchmark")
            run_container_script(
                " ".join(
                    [
                        shlex.quote(benchmark_bin),
                        f"--cloud={shlex.quote(host_to_container(cloud_list))}",
                        f"--result={shlex.quote(host_to_container(result_list))}",
                        f"--groundtruth={shlex.quote(host_to_container(gt_list))}",
                        "--is_folder=false",
                        f"--reserve={shlex.quote(reserve or 'JACCARD:0.9')}",
                    ]
                ),
                job=ctx,
            )
            summary["baseline_dir"] = str(gt_dir)
            summary["gt_enabled"] = True
            summary["benchmark_reserve"] = reserve or "JACCARD:0.9"
        else:
            summary["gt_enabled"] = False

        ctx.set_progress(88.0, "viewer", "exporting offline viewer")
        exporter_parts = [
            shlex.quote(exporter_bin),
            f"--cloud={shlex.quote(host_to_container(extract_dir / 'pcd'))}",
            f"--result={shlex.quote(host_to_container(result_dir))}",
            f"--output={shlex.quote(host_to_container(viewer_dir))}",
            "--is_folder=true",
            f"--max_points={max_points}",
        ]
        if gt_dir:
            exporter_parts.append(f"--groundtruth={shlex.quote(host_to_container(gt_dir))}")
        if reserve and gt_dir:
            exporter_parts.append(f"--reserve={shlex.quote(reserve)}")
        run_container_script(" ".join(exporter_parts), job=ctx)
        _populate_perception_viewer_assets(viewer_dir)
        ctx.add_artifact("viewer", "Viewer", viewer_dir, "index.html")
        summary["raw_viewer_url"] = f"/artifacts/{ctx.job_id}/viewer/index.html"
        summary["viewer_url"] = f"/tools/jobs/{ctx.job_id}/viewer"
        summary["viewer_dir"] = str(viewer_dir)
        ctx.set_summary(summary)


class EndpointStaticPlugin(ToolboxPlugin):
    plugin_id = "endpoint_static"
    name = "Endpoint Static"
    description = "Replay record bags into endpoint_static_visualizer_exporter and open the generated viewer."

    def probe(self) -> Dict[str, Any]:
        exporter_src = (REPO_ROOT / "modules/localization/endpoint/tools/endpoint_static_visualizer_exporter.cc").exists()
        recorder_src = (REPO_ROOT / "cyber/tools/cyber_recorder/main.cc").exists()
        return {
            "available": exporter_src and recorder_src,
            "actions": {"export_and_view": exporter_src and recorder_src},
            "missing": [],
        }

    def actions(self) -> List[Dict[str, Any]]:
        dag_default = _first_existing(
            ["modules/localization/dag/dag_streaming_endpoint_localization.dag"]
        )
        return [
            {
                "action_id": "export_and_view",
                "title": "Export And Visualize",
                "description": "Toolbox manages the output workspace and opens the generated static viewer.",
                "fields": [
                    {
                        "name": "data_package",
                        "label": "Data Package",
                        "type": "path",
                        "required": True,
                    },
                    {"name": "dag_config", "label": "DAG Config", "type": "path", "default": dag_default},
                    {"name": "max_full_points", "label": "Max Full Points", "type": "number", "default": 30000},
                    {"name": "max_filtered_points", "label": "Max Filtered Points", "type": "number", "default": 0},
                    {"name": "export_every_n", "label": "Export Every N", "type": "number", "default": 1},
                    {"name": "max_exports", "label": "Max Exports", "type": "number", "default": 0},
                ],
            }
        ]

    def run_action(self, action_id: str, params: Dict[str, Any], ctx: JobContext) -> None:
        del action_id
        exporter_bin = ensure_container_binary(
            "//modules/localization/endpoint/tools:endpoint_static_visualizer_exporter", ctx
        )
        recorder_bin = ensure_container_binary("//cyber/tools/cyber_recorder:cyber_recorder", ctx)
        supports_progress_file = container_binary_supports_flag(exporter_bin, "--progress_file")
        supports_expected_exports = container_binary_supports_flag(exporter_bin, "--expected_exports")
        dag_config = params.get("dag_config") or _first_existing(
            ["modules/localization/dag/dag_streaming_endpoint_localization.dag"]
        )
        if not dag_config:
            raise RuntimeError("missing dag_config")
        input_topic = _extract_first_channel_from_dag(REPO_ROOT / dag_config)
        output_dir = ctx.workspace / "viewer"
        progress_file = ctx.workspace / "endpoint_progress.json"
        inspect_result = inspect_records(params["data_package"])
        total_messages = count_topic_messages(params["data_package"], input_topic) if input_topic else 0
        export_every_n = max(1, int(params.get("export_every_n") or 1))
        expected_exports = total_messages // export_every_n + (1 if total_messages % export_every_n else 0)
        max_exports = int(params.get("max_exports") or 0)
        if max_exports > 0:
            expected_exports = min(expected_exports, max_exports)

        files = inspect_result["files"]
        play_parts = [shlex.quote(recorder_bin), "play"]
        for file_path in files:
            play_parts.append(f"-f {shlex.quote(host_to_container(Path(file_path)))}")

        exporter_parts = [
            shlex.quote(exporter_bin),
            f"--dag_config={shlex.quote(host_to_container(REPO_ROOT / dag_config))}",
            f"--output_dir={shlex.quote(host_to_container(output_dir))}",
            f"--max_full_points={int(params.get('max_full_points') or 30000)}",
            f"--max_filtered_points={int(params.get('max_filtered_points') or 0)}",
            f"--export_every_n={export_every_n}",
        ]
        if supports_expected_exports:
            exporter_parts.append(f"--expected_exports={expected_exports}")
        if supports_progress_file:
            exporter_parts.append(
                f"--progress_file={shlex.quote(host_to_container(progress_file))}"
            )

        ctx.set_progress(1.0, "prepare", f"replaying {len(files)} record file(s)")
        if progress_file.exists():
            progress_file.unlink()

        def _start_process(command: List[str], name: str) -> tuple[subprocess.Popen[str], "queue.Queue[str]"]:
            ctx.log("$ " + " ".join(shlex.quote(part) for part in command))
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            output_queue: "queue.Queue[str]" = queue.Queue()

            def _reader() -> None:
                assert process.stdout is not None
                for line in process.stdout:
                    output_queue.put(f"[{name}] {line.rstrip()}")

            threading.Thread(target=_reader, daemon=True).start()
            return process, output_queue

        exporter_process, exporter_queue = _start_process(
            container_command(" ".join(exporter_parts)), "endpoint_exporter"
        )
        time.sleep(2.0)
        play_process, play_queue = _start_process(
            container_command(" ".join(play_parts)), "cyber_recorder"
        )

        stopped_play_for_limit = False
        exporter_marked_done = False
        last_progress_mtime = 0.0

        def _drain_logs() -> None:
            for output_queue in (exporter_queue, play_queue):
                while True:
                    try:
                        ctx.log(output_queue.get_nowait())
                    except queue.Empty:
                        break

        try:
            while True:
                _drain_logs()

                if supports_progress_file and progress_file.exists():
                    mtime = progress_file.stat().st_mtime
                    if mtime != last_progress_mtime:
                        last_progress_mtime = mtime
                        ctx.read_progress_file(progress_file)
                        try:
                            payload = json.loads(progress_file.read_text(encoding="utf-8"))
                        except Exception:
                            payload = {}
                        current = int(payload.get("current", 0) or 0)
                        exporter_marked_done = bool(payload.get("done", False))
                        if expected_exports > 0 and current >= expected_exports and not stopped_play_for_limit:
                            stopped_play_for_limit = True
                            if play_process.poll() is None:
                                play_process.terminate()
                        if exporter_marked_done:
                            if play_process.poll() is None:
                                play_process.terminate()

                exporter_done = exporter_process.poll() is not None
                play_done = play_process.poll() is not None

                if exporter_done and not play_done:
                    play_process.terminate()

                if exporter_done and play_done:
                    _drain_logs()
                    break

                time.sleep(0.2)
        finally:
            for process in (play_process, exporter_process):
                if process.poll() is None:
                    process.terminate()
            time.sleep(0.2)
            for process in (play_process, exporter_process):
                if process.poll() is None:
                    process.kill()
            _drain_logs()
            if supports_progress_file and progress_file.exists():
                ctx.read_progress_file(progress_file)

        if exporter_process.returncode != 0:
            raise RuntimeError(f"endpoint exporter failed with exit code {exporter_process.returncode}")

        ctx.add_artifact("viewer", "Viewer", output_dir, "index.html")
        summary = {
            "raw_viewer_url": f"/artifacts/{ctx.job_id}/viewer/index.html",
            "viewer_url": f"/tools/jobs/{ctx.job_id}/viewer",
            "viewer_dir": str(output_dir),
            "expected_exports": expected_exports,
        }
        ctx.set_summary(summary)


class SlamVisualizationPlugin(ToolboxPlugin):
    plugin_id = "slam_visualization"
    name = "SLAM Visualization"
    description = "Single-port live SLAM visualization proxied through the toolbox."

    def __init__(self, job_manager: JobManager):
        super().__init__(job_manager)
        self._queues: set[queue.Queue] = set()
        self._lock = threading.Lock()
        self._process = None
        self._ready_payload = {"raw_map_exists": False, "downsampled_map_exists": False}

    def probe(self) -> Dict[str, Any]:
        app_src = (REPO_ROOT / "modules/slam_localization/tools/slam_visualization/app.py").exists()
        return {
            "available": app_src,
            "actions": {"live_view": app_src},
            "missing": [],
        }

    def actions(self) -> List[Dict[str, Any]]:
        return [
            {
                "action_id": "live_view",
                "title": "Open Live Viewer",
                "fields": [],
                "live_url": "/tools/slam_visualization/live",
            }
        ]

    def run_action(self, action_id: str, params: Dict[str, Any], ctx: JobContext) -> None:
        del action_id, params, ctx
        raise RuntimeError("slam_visualization is a live plugin and does not create background jobs")

    def _ensure_stream_process(self) -> None:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return
            proxy_path = host_to_container(REPO_ROOT / "modules/tools/whl_toolbox/slam_stream_proxy.py")
            command = container_command(f"python3 -u {shlex.quote(proxy_path)}")
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            thread = threading.Thread(target=self._fanout_stream, daemon=True)
            thread.start()

    def _fanout_stream(self) -> None:
        assert self._process is not None
        assert self._process.stdout is not None
        for line in self._process.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                payload = {"event": "log", "payload": {"message": line}}
            if payload.get("event") == "ready":
                self._ready_payload = payload.get("payload", self._ready_payload)
            with self._lock:
                subscribers = list(self._queues)
            for subscriber in subscribers:
                subscriber.put(payload)

    def register_routes(self, app) -> None:
        @app.get("/tools/slam_visualization/live")
        def _slam_page():
            return send_file(REPO_ROOT / "modules/tools/whl_toolbox/templates/slam.html")

        @app.get("/api/plugins/slam_visualization/stream")
        def _slam_stream():
            self._ensure_stream_process()
            subscriber: "queue.Queue[dict[str, Any]]" = queue.Queue()
            with self._lock:
                self._queues.add(subscriber)

            def generate():
                try:
                    yield "event: ready\ndata: " + json.dumps(self._ready_payload) + "\n\n"
                    while True:
                        try:
                            payload = subscriber.get(timeout=10.0)
                            yield (
                                "event: "
                                + payload.get("event", "message")
                                + "\ndata: "
                                + json.dumps(payload.get("payload", {}))
                                + "\n\n"
                            )
                        except queue.Empty:
                            yield "event: ping\ndata: {}\n\n"
                finally:
                    with self._lock:
                        self._queues.discard(subscriber)

            return Response(generate(), mimetype="text/event-stream")

        @app.get("/api/plugins/slam_visualization/assets/<path:asset_name>")
        def _slam_asset(asset_name: str):
            if asset_name not in {"three.min.js", "OrbitControls.js"}:
                abort(404)
            return send_file(REPO_ROOT / "modules/tools/whl_toolbox/static/vendor" / asset_name)

        @app.get("/api/plugins/slam_visualization/map_raw.pcd")
        def _raw_map():
            return self._container_cat("/apollo/cyber/data/slam_tf/global.pcd", "application/octet-stream")

        @app.get("/api/plugins/slam_visualization/map_downsampled.pcd")
        def _downsampled_map():
            return self._container_cat(
                "/apollo/cyber/data/slam_tf/global_downsample.pcd", "application/octet-stream"
            )

    def _container_cat(self, container_path: str, mimetype: str):
        command = container_command(f"cat {shlex.quote(container_path)}")
        process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if process.returncode != 0:
            abort(404)
        return Response(process.stdout, mimetype=mimetype)


class LivePointCloudPlugin(ToolboxPlugin):
    plugin_id = "live_pointcloud"
    name = "Live PointCloud"
    description = "Launch a CivetWeb C++ backend for realtime point cloud viewing on a selected channel."

    def __init__(self, job_manager: JobManager):
        super().__init__(job_manager)
        self._lock = threading.Lock()
        self._process: subprocess.Popen[str] | None = None
        self._channel = ""
        self._imu_frame_id = ""
        self._port = int(os.environ.get("WHL_TOOLBOX_LIVE_POINTCLOUD_PORT", "8891"))
        self._started_at = 0.0
        self._logs: list[str] = []

    def probe(self) -> Dict[str, Any]:
        backend_src = (REPO_ROOT / "modules/tools/whl_toolbox/live_pointcloud_viewer.cc").exists()
        return {
            "available": backend_src,
            "actions": {"launch_viewer": backend_src},
            "missing": [],
        }

    def actions(self) -> List[Dict[str, Any]]:
        return [
            {
                "action_id": "launch_viewer",
                "title": "Start Live Viewer",
                "description": "Toolbox starts or reuses the C++ CivetWeb backend, then opens the realtime point cloud viewer.",
                "fields": [
                    {
                        "name": "channel",
                        "label": "PointCloud Channel",
                        "type": "text",
                        "required": True,
                    },
                    {
                        "name": "imu_frame_id",
                        "label": "IMU Frame",
                        "type": "text",
                    },
                ],
            }
        ]

    def register_routes(self, app) -> None:
        @app.get("/tools/live_pointcloud/live")
        def _live_pointcloud_page():
            return send_file(REPO_ROOT / "modules/tools/whl_toolbox/templates/live_pointcloud.html")

        @app.get("/api/plugins/live_pointcloud/state")
        def _live_pointcloud_state():
            return jsonify(self._state_payload())

        @app.get("/api/plugins/live_pointcloud/assets/<path:asset_name>")
        def _live_pointcloud_asset(asset_name: str):
            if asset_name not in {"three.min.js", "OrbitControls.js"}:
                abort(404)
            return send_file(REPO_ROOT / "modules/tools/whl_toolbox/static/vendor" / asset_name)

    def run_action(self, action_id: str, params: Dict[str, Any], ctx: JobContext) -> None:
        if action_id != "launch_viewer":
            raise RuntimeError(f"unsupported action: {action_id}")
        channel = str(params.get("channel") or "").strip()
        imu_frame_id = str(params.get("imu_frame_id") or "").strip()
        if not channel:
            raise RuntimeError("channel is required")

        ctx.set_progress(5.0, "prepare", f"channel: {channel}")
        viewer_bin = ensure_container_binary(
            "//modules/tools/whl_toolbox:live_pointcloud_viewer", ctx
        )

        reused = False
        with self._lock:
            if self._process is not None and self._process.poll() is not None:
                self._process = None
                self._channel = ""
                self._imu_frame_id = ""
            if (
                self._process is not None
                and self._channel == channel
                and self._imu_frame_id == imu_frame_id
            ):
                reused = True

        if reused:
            ctx.log(f"reusing live pointcloud backend on port {self._port} for {channel}")
            ctx.set_progress(100.0, "ready", "viewer is already running")
            ctx.set_summary(
                {
                    "viewer_url": "/tools/live_pointcloud/live",
                    "channel": channel,
                    "imu_frame_id": imu_frame_id,
                    "port": self._port,
                    "backend_logs": self._logs[-50:],
                }
            )
            return

        ready_file = ctx.workspace / "live_pointcloud_ready.json"
        if ready_file.exists():
            ready_file.unlink()

        command = container_command(
            " ".join(
                [
                    shlex.quote(viewer_bin),
                    f"--channel={shlex.quote(channel)}",
                    f"--imu_frame_id={shlex.quote(imu_frame_id)}",
                    f"--port={self._port}",
                    "--max_points=30000",
                    f"--ready_file={shlex.quote(host_to_container(ready_file))}",
                ]
            )
        )
        ctx.log("$ " + " ".join(shlex.quote(part) for part in command))

        with self._lock:
            if self._process is not None and self._process.poll() is None:
                ctx.log(f"stopping previous live pointcloud backend for {self._channel}")
                self._process.terminate()
                time.sleep(0.5)
                if self._process.poll() is None:
                    self._process.kill()
                self._process = None
                self._channel = ""
                self._imu_frame_id = ""
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self._process = process
            self._channel = channel
            self._imu_frame_id = imu_frame_id
            self._started_at = time.time()
            self._logs = []
            threading.Thread(
                target=self._drain_process_logs,
                args=(process,),
                daemon=True,
                name="whl-live-pointcloud-log",
            ).start()

        ctx.set_progress(40.0, "launch", f"starting backend on port {self._port}")
        deadline = time.time() + 12.0
        logged_count = 0
        while time.time() < deadline:
            with self._lock:
                process = self._process
                pending_logs = self._logs[logged_count:]
                logged_count = len(self._logs)
            for line in pending_logs:
                ctx.log(line)
            if process is None:
                raise RuntimeError("live pointcloud backend terminated before becoming ready")
            if process.poll() is not None:
                raise RuntimeError(
                    f"live pointcloud backend failed with exit code {process.returncode}"
                )
            if wait_for_file(ready_file, timeout_sec=0.2):
                break
        else:
            raise RuntimeError("live pointcloud backend did not become ready within 12s")

        ctx.set_progress(100.0, "ready", f"viewer is ready on port {self._port}")
        ctx.set_summary(
            {
                "viewer_url": "/tools/live_pointcloud/live",
                "channel": channel,
                "imu_frame_id": imu_frame_id,
                "port": self._port,
                "backend_logs": self._logs[-50:],
            }
        )

    def _drain_process_logs(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is None:
            return
        for line in process.stdout:
            text = line.rstrip()
            with self._lock:
                self._logs.append(text)
                self._logs = self._logs[-200:]

    def _state_payload(self) -> Dict[str, Any]:
        with self._lock:
            process = self._process
            running = process is not None and process.poll() is None
            return {
                "running": running,
                "channel": self._channel,
                "imu_frame_id": self._imu_frame_id,
                "port": self._port if running else None,
                "started_at": self._started_at if running else 0.0,
                "logs": self._logs[-50:],
            }


def create_plugins(job_manager: JobManager) -> Dict[str, ToolboxPlugin]:
    plugins: List[ToolboxPlugin] = [
        PerceptionLidarPlugin(job_manager),
        EndpointStaticPlugin(job_manager),
        SlamVisualizationPlugin(job_manager),
        LivePointCloudPlugin(job_manager),
    ]
    return {plugin.plugin_id: plugin for plugin in plugins}
