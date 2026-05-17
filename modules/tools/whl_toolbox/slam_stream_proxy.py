import json
import os
import sys

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import numpy as np

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.localization_msgs.localization_pb2 import LocalizationEstimate
from modules.slam_localization.proto.visualization_pb2 import Correction, KeyFrame, ScanMatchingStatus
from modules.slam_localization.proto.odometry_pb2 import Odometry


RAW_MAP_PATH = "/apollo/cyber/data/slam_tf/global.pcd"
DOWNSAMPLED_MAP_PATH = "/apollo/cyber/data/slam_tf/global_downsample.pcd"
GNSS_OFFSET_FILE = "/apollo/cyber/data/slam_tf/gnss-map-offset.txt"


def emit(event: str, payload: dict) -> None:
    sys.stdout.write(json.dumps({"event": event, "payload": payload}) + "\n")
    sys.stdout.flush()


def pose_to_matrix(pose):
    from scipy.spatial.transform import Rotation

    pos = pose.position
    euler = pose.euler_angles
    translation = [pos.x, pos.y, pos.z]
    rotation_matrix = Rotation.from_euler(
        "zyx", [euler.z, euler.y, euler.x], degrees=False
    ).as_matrix()
    matrix = np.identity(4)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = translation
    return matrix


def parse_offset():
    try:
        with open(GNSS_OFFSET_FILE, "r", encoding="utf-8") as fin:
            values = fin.read().split("\t")
        return float(values[0]), float(values[1]), float(values[2])
    except Exception:
        return 0.0, 0.0, 0.0


def main():
    offset_x, offset_y, offset_z = parse_offset()
    cyber.init()
    node = cyber.Node("whl_toolbox_slam_stream")

    def on_loc(msg: LocalizationEstimate):
        emit(
            "pose_update",
            {
                "mode": "localization",
                "data": [
                    msg.pose.position.x - offset_x,
                    msg.pose.position.y - offset_y,
                    msg.pose.position.z - offset_z,
                    msg.pose.orientation.qx,
                    msg.pose.orientation.qy,
                    msg.pose.orientation.qz,
                    msg.pose.orientation.qw,
                ],
            },
        )

    def on_scan(msg: ScanMatchingStatus):
        points = [[point.x, point.y, point.z] for point in msg.current_scan_cloud.point]
        emit("scan_update", {"mode": "localization", "points": points})

    def on_map_pose(msg: Odometry):
        pose = msg.pose.pose
        emit(
            "pose_update",
            {
                "mode": "mapping",
                "data": [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.qx,
                    pose.orientation.qy,
                    pose.orientation.qz,
                    pose.orientation.qw,
                ],
            },
        )

    def on_keyframe(msg: KeyFrame):
        points = [
            [point.x, point.y, point.z]
            for point in msg.point_cloud.point
            if not (np.isnan(point.x) or np.isnan(point.y) or np.isnan(point.z))
        ]
        emit(
            "new_point_cloud_chunk",
            {
                "id": msg.id,
                "mode": "mapping",
                "pose": pose_to_matrix(msg.pose).flatten(order="F").tolist(),
                "points": points,
            },
        )

    def on_correction(msg: Correction):
        updates = []
        for item in msg.updated_poses:
            updates.append(
                {
                    "id": item.kf_id,
                    "pose": pose_to_matrix(item.pose_msg).flatten(order="F").tolist(),
                }
            )
        emit("map_correction", {"mode": "mapping", "updated_poses": updates})

    node.create_reader("/apollo/localization/pose", LocalizationEstimate, on_loc)
    node.create_reader("/liorf/vis/scan_matching_status", ScanMatchingStatus, on_scan)
    node.create_reader("odometry/imu", Odometry, on_map_pose)
    node.create_reader("/liorf/vis/keyframe", KeyFrame, on_keyframe)
    node.create_reader("/liorf/vis/correction", Correction, on_correction)
    emit(
        "ready",
        {
            "raw_map_exists": os.path.exists(RAW_MAP_PATH),
            "downsampled_map_exists": os.path.exists(DOWNSAMPLED_MAP_PATH),
        },
    )
    cyber.spin()


if __name__ == "__main__":
    main()
