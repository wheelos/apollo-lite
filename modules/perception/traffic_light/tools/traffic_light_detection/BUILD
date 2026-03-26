load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_light_detection_tool_lib",
    srcs = ["tool_runner.cc"],
    hdrs = ["tool_runner.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "//modules/perception/traffic_light:traffic_light_perception_pipeline",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_binary(
    name = "traffic_light_detection_tool",
    srcs = ["traffic_light_detection_tool_main.cc"],
    deps = [
        ":traffic_light_detection_tool_lib",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
