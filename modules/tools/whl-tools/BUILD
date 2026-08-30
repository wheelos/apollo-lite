load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "save_lidar_semantic_pcd",
    srcs = ["save_lidar_semantic_pcd.py"],
    deps = [
        "@core//cyber/python/cyber_py3:record",
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_py_pb2",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:pointcloud_py_pb2",
    ],
)

py_binary(
    name = "image_message_publisher",
    srcs = ["image_message_publisher.py"],
    main = "image_message_publisher.py",
    deps = [
        "@core//cyber/python/cyber_py3:cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_py_pb2",
    ],
)

py_binary(
    name = "lane_debug_visualizer",
    srcs = ["lane_debug_visualizer.py"],
    main = "lane_debug_visualizer.py",
    deps = [
        "@core//cyber/python/cyber_py3:cyber",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_lane_py_pb2",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_py_pb2",
    ],
)
