load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "publisher",
    srcs = ["publisher.py"],
    main = "publisher.py",
    deps = [
        "@core//cyber/python/cyber_py3:cyber",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_detail_py_pb2",
        "@wheelos_msgs//wheelos_msgs/chassis_msgs:chassis_py_pb2",
        "@wheelos_msgs//wheelos_msgs/control_msgs:control_cmd_py_pb2",
        "@wheelos_msgs//wheelos_msgs/control_msgs:pad_msg_py_pb2",
        "@wheelos_msgs//wheelos_msgs/dreamview_msgs:chart_py_pb2",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:localization_py_pb2",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:perception_obstacle_py_pb2",
        "@wheelos_msgs//wheelos_msgs/perception_msgs:traffic_light_detection_py_pb2",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_py_pb2",
        "@wheelos_msgs//wheelos_msgs/prediction_msgs:prediction_obstacle_py_pb2",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_py_pb2",
    ],
)
