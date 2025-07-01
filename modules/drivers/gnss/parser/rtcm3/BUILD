load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtcm3_parser",
    srcs = [
        "rtcm3_parser.cc",
    ],
    hdrs = [
        "rtcm3_parser.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
        "//modules/drivers/gnss/util",
        "//modules/drivers/gnss/parser",
        "//modules/drivers/gnss/parser:rtcm_decode",
        "//third_party/rtklib"
    ],
)

cpplint()
