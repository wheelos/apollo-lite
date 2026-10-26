load("@rules_python//python:defs.bzl", "py_binary", "py_test")

py_binary(
    name = "save_lidar_semantic_pcd",
    srcs = ["save_lidar_semantic_pcd.py"],
    deps = [
        "//modules/lidar_semantic_segmentation/proto:lidar_semantic_segmentation_py_pb2",
        "@core//cyber/python/cyber_py3:record",
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

py_binary(
    name = "openlane_lane_evaluator",
    srcs = ["openlane_lane_evaluator.py"],
    main = "openlane_lane_evaluator.py",
)

py_binary(
    name = "openlane_projection_accuracy",
    srcs = ["openlane_projection_accuracy.py"],
    main = "openlane_projection_accuracy.py",
)

py_test(
    name = "openlane_lane_evaluator_test",
    size = "small",
    srcs = [
        "openlane_lane_evaluator.py",
        "openlane_lane_evaluator_test.py",
    ],
    main = "openlane_lane_evaluator_test.py",
)

py_test(
    name = "openlane_projection_accuracy_test",
    size = "small",
    srcs = [
        "openlane_projection_accuracy.py",
        "openlane_projection_accuracy_test.py",
    ],
    main = "openlane_projection_accuracy_test.py",
)

py_binary(
    name = "camera_semantic_segmentation_visualizer",
    srcs = ["camera_semantic_segmentation_visualizer.py"],
    main = "camera_semantic_segmentation_visualizer.py",
    deps = [
        "//modules/camera_semantic_segmentation/proto:camera_semantic_segmentation_py_pb2",
        "@core//cyber/python/cyber_py3:cyber",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_py_pb2",
    ],
)
