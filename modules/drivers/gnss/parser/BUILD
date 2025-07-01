load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parser",
    hdrs = [
        "parser.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/util:data_buffer",
    ],
)

cc_library(
    name = "rtcm_decode",
    hdrs = [
        "rtcm_decode.h",
    ],
    deps = [
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
    ],
)

cc_library(
    name = "parser_factory",
    hdrs = [
        "parser_factory.h",
    ],
    deps = [
        ":parser",
        "//cyber",
        "//modules/drivers/gnss/parser/novatel:novatel_parser",
        "//modules/drivers/gnss/parser/huace:huace_parser",
        "//modules/drivers/gnss/proto:config_cc_proto",
    ],
)

cc_library(
    name = "data_parser",
    srcs = [
        "data_parser.cc",
    ],
    hdrs = [
        "data_parser.h",
    ],
    deps = [
        ":parser",
        ":parser_factory",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/drivers/gnss/util",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/transform:transform_broadcaster",
        "@eigen",
        "@proj",
    ],
)

cc_library(
    name = "rtcm_parser",
    srcs = [
        "rtcm_parser.cc",
    ],
    hdrs = [
        "parser.h",
        "rtcm_decode.h",
        "rtcm_parser.h",
    ],
    copts = ["-Ithird_party/rtklib"],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/parser/rtcm3:rtcm3_parser",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
        "//modules/drivers/gnss/util",
        "//third_party/rtklib",
    ],
)

cpplint()
