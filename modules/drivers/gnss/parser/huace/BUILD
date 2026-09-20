load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "huace_parser",
    srcs = [
        "huace_messages.h",
        "huace_parser.cc",
    ],
    hdrs = [
        "huace_parser.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/common/util",
        "//modules/drivers/gnss/parser",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/util",
        "//modules/drivers/gnss/util:data_buffer",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/basic_msgs:pnc_point_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:heading_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:ins_cc_proto",
    ],
)

cpplint()
