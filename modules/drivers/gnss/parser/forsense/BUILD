load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "forsense_parser",
    srcs = [
        "forsense_binary_parser.cc",
        "forsense_messages.cc",
        "forsense_nmea_parser.cc",
    ],
    hdrs = [
        "forsense_binary_parser.h",
        "forsense_messages.h",
        "forsense_nmea_parser.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/drivers/gnss/parser",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/util",
        "//modules/drivers/gnss/util:data_buffer",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cpplint()
