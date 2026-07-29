import yaml
import os


def _deep_merge_dict(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_dict(base[key], value)
        else:
            base[key] = value


class ConfigManager:
    def __init__(self, path):
        self.data = {
            "topics": {"control": "/apollo/control", "chassis": "/apollo/chassis"},
            "limits": {"emergency_brake_val": 50.0},
            "thresholds": {"steady_error": 2.0, "chassis_wait_sec": 5.0},
        }
        if os.path.exists(path):
            try:
                with open(path, "r") as f:
                    user_config = yaml.safe_load(f)
                    if user_config:
                        _deep_merge_dict(self.data, user_config)
            except Exception as e:
                print(f"Warning: Failed to load config {path}: {e}")

    @property
    def topics(self):
        return self.data["topics"]

    @property
    def limits(self):
        return self.data["limits"]

    @property
    def thresholds(self):
        return self.data["thresholds"]
