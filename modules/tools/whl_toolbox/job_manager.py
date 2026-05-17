import json
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional


@dataclass
class JobState:
    job_id: str
    plugin_id: str
    action_id: str
    params: Dict[str, Any]
    workspace: Path
    status: str = "queued"
    stage: str = "queued"
    progress: float = 0.0
    message: str = ""
    error: str = ""
    summary: Dict[str, Any] = field(default_factory=dict)
    artifacts: List[Dict[str, Any]] = field(default_factory=list)
    logs: List[str] = field(default_factory=list)
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)


class JobContext:
    def __init__(self, manager: "JobManager", job_id: str):
        self._manager = manager
        self.job_id = job_id

    @property
    def state(self) -> JobState:
        return self._manager.get_state(self.job_id)

    @property
    def workspace(self) -> Path:
        return self.state.workspace

    def log(self, message: str) -> None:
        self._manager.append_log(self.job_id, message)

    def set_progress(self, progress: float, stage: str, message: str = "") -> None:
        self._manager.update_progress(self.job_id, progress, stage, message)

    def set_summary(self, summary: Dict[str, Any]) -> None:
        self._manager.set_summary(self.job_id, summary)

    def add_artifact(self, key: str, label: str, root: Path, default_entry: str = "") -> None:
        self._manager.add_artifact(self.job_id, key, label, root, default_entry)

    def read_progress_file(self, progress_file: Path) -> None:
        try:
            payload = json.loads(progress_file.read_text(encoding="utf-8"))
        except Exception:
            return
        percent = float(payload.get("percent", 0.0))
        stage = str(payload.get("stage", self.state.stage))
        message = str(payload.get("message", ""))
        self.set_progress(percent, stage, message)


class JobManager:
    def __init__(self, runtime_root: Path):
        self.runtime_root = runtime_root
        self.runtime_root.mkdir(parents=True, exist_ok=True)
        self._jobs: Dict[str, JobState] = {}
        self._lock = threading.Lock()

    def create_job(
        self,
        plugin_id: str,
        action_id: str,
        params: Dict[str, Any],
        runner: Callable[[JobContext], None],
    ) -> JobState:
        job_id = uuid.uuid4().hex[:12]
        workspace = self.runtime_root / job_id
        workspace.mkdir(parents=True, exist_ok=True)
        state = JobState(
            job_id=job_id,
            plugin_id=plugin_id,
            action_id=action_id,
            params=params,
            workspace=workspace,
        )
        with self._lock:
            self._jobs[job_id] = state

        thread = threading.Thread(
            target=self._run_job,
            args=(job_id, runner),
            name=f"whl-toolbox-job-{job_id}",
            daemon=True,
        )
        thread.start()
        return state

    def _run_job(self, job_id: str, runner: Callable[[JobContext], None]) -> None:
        self._set_status(job_id, "running")
        ctx = JobContext(self, job_id)
        try:
            runner(ctx)
            self._set_status(job_id, "completed")
            if self.get_state(job_id).progress < 100.0:
                self.update_progress(job_id, 100.0, "complete", "job finished")
        except Exception as exc:
            self._set_status(job_id, "failed", str(exc))
            self.append_log(job_id, f"[error] {exc}")

    def _set_status(self, job_id: str, status: str, error: str = "") -> None:
        with self._lock:
            state = self._jobs[job_id]
            state.status = status
            state.error = error
            state.updated_at = time.time()

    def update_progress(self, job_id: str, progress: float, stage: str, message: str) -> None:
        with self._lock:
            state = self._jobs[job_id]
            state.progress = max(0.0, min(100.0, progress))
            state.stage = stage
            state.message = message
            state.updated_at = time.time()

    def append_log(self, job_id: str, message: str) -> None:
        with self._lock:
            state = self._jobs[job_id]
            state.logs.append(message.rstrip())
            state.logs = state.logs[-500:]
            state.updated_at = time.time()

    def add_artifact(
        self, job_id: str, key: str, label: str, root: Path, default_entry: str = ""
    ) -> None:
        with self._lock:
            state = self._jobs[job_id]
            state.artifacts = [
                artifact for artifact in state.artifacts if artifact["key"] != key
            ]
            state.artifacts.append(
                {
                    "key": key,
                    "label": label,
                    "root": str(root),
                    "default_entry": default_entry,
                }
            )
            state.updated_at = time.time()

    def set_summary(self, job_id: str, summary: Dict[str, Any]) -> None:
        with self._lock:
            state = self._jobs[job_id]
            state.summary = summary
            state.updated_at = time.time()

    def get_state(self, job_id: str) -> JobState:
        with self._lock:
            return self._jobs[job_id]

    def get_job(self, job_id: str) -> Dict[str, Any]:
        state = self.get_state(job_id)
        return {
            "job_id": state.job_id,
            "plugin_id": state.plugin_id,
            "action_id": state.action_id,
            "params": state.params,
            "status": state.status,
            "stage": state.stage,
            "progress": state.progress,
            "message": state.message,
            "error": state.error,
            "summary": state.summary,
            "artifacts": state.artifacts,
            "logs": state.logs[-100:],
            "workspace": str(state.workspace),
            "created_at": state.created_at,
            "updated_at": state.updated_at,
        }

    def list_jobs(self) -> List[Dict[str, Any]]:
        with self._lock:
            job_ids = list(self._jobs.keys())
        return [self.get_job(job_id) for job_id in job_ids]
