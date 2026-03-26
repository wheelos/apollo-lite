load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "traffic_light_detection",
    srcs = ["traffic_light_detection.cc"],
    deps = [
        "//modules/perception/traffic_light/tools/traffic_light_detection:traffic_light_detection_tool_lib",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "yolo_single_stage_dector_tool",
    srcs = ["yolo_single_stage_dector_tool.cc"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/traffic_light/detector:detection",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cpplint()
