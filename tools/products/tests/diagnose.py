"""Private diagnostic fixture; no model inference or device access."""

import os
from pathlib import Path

from wheelos_msgs.chassis_msgs import chassis_pb2

root = Path(os.environ["WHEELOS_ASSET_ROOT"])
assert (
    chassis_pb2.Chassis.FromString(
        chassis_pb2.Chassis(speed_mps=0.0).SerializeToString()
    ).speed_mps
    == 0.0
)
print((root / "models/alpha/fixture/model.bin").read_text().strip())
