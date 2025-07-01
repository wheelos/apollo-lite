load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "raw_stream",
    srcs = ["raw_stream.cc"],
    hdrs = ["raw_stream.h"],
    deps = [
        "//cyber",
        "//modules/drivers/hal/stream:stream_factory",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/drivers/gnss/parser:data_parser",
        "//modules/drivers/gnss/parser:rtcm_parser",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
