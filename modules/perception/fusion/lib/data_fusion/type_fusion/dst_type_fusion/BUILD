load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dst_type_fusion",
    srcs = ["dst_type_fusion.cc"],
    hdrs = ["dst_type_fusion.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base:base_init_options",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/base:track",
        "//modules/perception/fusion/common:camera_util",
        "//modules/perception/fusion/common:dst_evidence",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/config_manager",
        "//modules/perception/pipeline/proto/plugin:dst_type_fusion_config_cc_proto",
        "@com_google_absl//absl/strings",
        "@boost.format",
    ],
)

cc_test(
    name = "dst_type_fusion_test",
    size = "small",
    srcs = ["dst_type_fusion_test.cc"],
    deps = [
        ":dst_type_fusion",
        "//modules/perception/base:base_type",
        "//modules/perception/base:frame",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
