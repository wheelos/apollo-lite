load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/hardware:defs.bzl", "enabled")

package(default_visibility = ["//visibility:public"])

GNSS_DEFINES = enabled(
    "with_gnss_forsense",
    [
        "GNSS_WITH_FORSENSE",
    ],
) + enabled(
    "with_gnss_huace",
    [
        "GNSS_WITH_HUACE",
    ],
) + enabled(
    "with_gnss_novatel",
    [
        "GNSS_WITH_NOVATEL",
    ],
)

cc_library(
    name = "parser",
    hdrs = [
        "parser.h",
    ],
    deps = [
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/util:data_buffer",
        "@core//cyber",
    ],
)

cc_library(
    name = "rtcm_decode",
    hdrs = [
        "rtcm_decode.h",
    ],
    deps = [
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
    ],
)

cc_library(
    name = "parser_factory",
    srcs = ["parser_factory.cc"],
    hdrs = [
        "parser_factory.h",
    ],
    local_defines = GNSS_DEFINES,
    deps = [
        ":parser",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "@core//cyber",
    ] + enabled(
        "with_gnss_forsense",
        [
            "//modules/drivers/gnss/parser/forsense:forsense_parser",
        ],
    ) + enabled(
        "with_gnss_huace",
        [
            "//modules/drivers/gnss/parser/huace:huace_parser",
        ],
    ) + enabled(
        "with_gnss_novatel",
        [
            "//modules/drivers/gnss/parser/novatel:novatel_parser",
        ],
    ),
)

cc_test(
    name = "parser_factory_test",
    size = "small",
    srcs = ["parser_factory_test.cc"],
    local_defines = GNSS_DEFINES,
    deps = [
        ":parser_factory",
        "@com_google_googletest//:gtest_main",
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
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/util",
        "@core//cyber",
        "@eigen",
        "@proj",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_common//modules/common/util:message_util",
        "@wheelos_common//modules/common/util:proj_helper",
        "@wheelos_common//modules/common/util:time_conversion",
        "@wheelos_common//modules/common/util:util_tool",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:gps_cc_proto",
        "@wheelos_msgs//wheelos_msgs/localization_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:heading_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:imu_cc_proto",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:ins_cc_proto",
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
        "//modules/drivers/gnss/parser/rtcm3:rtcm3_parser",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/util",
        "//third_party/rtklib",
        "@core//cyber",
        "@wheelos_common//modules/common/adapters:adapter_gflags",
        "@wheelos_msgs//wheelos_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
    ],
)

cpplint()
