import os
from pathlib import Path

from flask import Flask, abort, jsonify, render_template, request, send_from_directory

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

if __package__ in {None, ""}:  # pragma: no cover
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from container_runtime import REPO_ROOT
    from job_manager import JobManager
    from plugins import create_plugins
else:
    from .container_runtime import REPO_ROOT
    from .job_manager import JobManager
    from .plugins import create_plugins


RUNTIME_ROOT = REPO_ROOT / "modules/tools/whl_toolbox/runtime/jobs"


def create_app() -> Flask:
    app = Flask(
        __name__,
        static_folder=str(REPO_ROOT / "modules/tools/whl_toolbox/static"),
        template_folder=str(REPO_ROOT / "modules/tools/whl_toolbox/templates"),
    )
    job_manager = JobManager(RUNTIME_ROOT)
    plugins = create_plugins(job_manager)
    app.config["JOB_MANAGER"] = job_manager
    app.config["PLUGINS"] = plugins

    @app.get("/")
    def index():
        return render_template("index.html")

    @app.get("/api/plugins")
    def list_plugins():
        return jsonify([plugin.metadata() for plugin in plugins.values()])

    @app.get("/api/plugins/<plugin_id>")
    def get_plugin(plugin_id: str):
        plugin = plugins.get(plugin_id)
        if plugin is None:
            abort(404)
        return jsonify(plugin.metadata())

    @app.get("/api/plugins/<plugin_id>/inspect_record")
    def inspect_record_endpoint(plugin_id: str):
        plugin = plugins.get(plugin_id)
        if plugin is None:
            abort(404)
        raw_path = request.args.get("path", "")
        action_id = request.args.get("action_id", "")
        return jsonify(plugin.inspect_record(action_id, raw_path))

    @app.post("/api/jobs")
    def create_job():
        payload = request.get_json(force=True)
        plugin_id = payload["plugin_id"]
        action_id = payload["action_id"]
        params = payload.get("params", {})
        plugin = plugins.get(plugin_id)
        if plugin is None:
            abort(404)
        job = job_manager.create_job(
            plugin_id,
            action_id,
            params,
            lambda ctx: plugin.run_action(action_id, params, ctx),
        )
        return jsonify(job_manager.get_job(job.job_id))

    @app.get("/api/jobs")
    def list_jobs():
        return jsonify(job_manager.list_jobs())

    @app.get("/api/jobs/<job_id>")
    def get_job(job_id: str):
        return jsonify(job_manager.get_job(job_id))

    @app.get("/tools/jobs/<job_id>/viewer")
    def job_viewer(job_id: str):
        job = job_manager.get_job(job_id)
        artifact = next((item for item in job["artifacts"] if item["key"] == "viewer"), None)
        if artifact is None or not artifact.get("default_entry"):
            abort(404)
        iframe_url = f"/artifacts/{job_id}/viewer/{artifact['default_entry']}"
        return render_template("job_viewer.html", job=job, iframe_url=iframe_url)

    @app.get("/artifacts/<job_id>/<artifact_key>/")
    @app.get("/artifacts/<job_id>/<artifact_key>/<path:subpath>")
    def artifact(job_id: str, artifact_key: str, subpath: str = ""):
        job = job_manager.get_job(job_id)
        artifact = next((item for item in job["artifacts"] if item["key"] == artifact_key), None)
        if artifact is None:
            abort(404)
        root = Path(artifact["root"])
        entry = subpath or artifact.get("default_entry") or ""
        if root.is_dir():
            if not entry:
                abort(404)
            return send_from_directory(root, entry)
        abort(404)

    for plugin in plugins.values():
        plugin.register_routes(app)

    return app


def main() -> None:
    app = create_app()
    host = os.environ.get("WHL_TOOLBOX_HOST", "0.0.0.0")
    port = int(os.environ.get("WHL_TOOLBOX_PORT", "8080"))
    app.run(host=host, port=port, debug=False)


if __name__ == "__main__":
    main()
