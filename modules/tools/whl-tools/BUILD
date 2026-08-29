load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_python//python:defs.bzl", "py_binary")

cc_binary(
    name = "image_message_publisher",
    srcs = ["image_message_publisher.cc"],
    deps = [
        "@core//cyber",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:videoio",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:sensor_image_cc_proto",
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
