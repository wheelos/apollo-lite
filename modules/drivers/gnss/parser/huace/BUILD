load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "huace_parser",
    srcs = [
        "huace_parser.cc",
        "huace_messages.h",
    ],
    hdrs = [
        "huace_parser.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/util:data_buffer",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util",
        "//modules/drivers/gnss/util",
        "//modules/drivers/gnss/parser",
        "@com_google_absl//absl/types:span",
        "@com_google_absl//absl/strings"
    ],
    visibility = ["//visibility:public"],
)

cpplint()
