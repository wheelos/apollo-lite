load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "traffic_light_detection",
    srcs = ["traffic_light_detection.cc"],
    deps = [
        "//modules/perception/camera/app:traffic_light_camera_perception",
        "//modules/perception/camera/tools/common",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber",
    ],
)

cc_binary(
    name = "yolo_single_stage_dector_tool",
    srcs = ["yolo_single_stage_dector_tool.cc"],
    deps = [
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/traffic_light/detector/detection",
        "//modules/perception/pipeline/proto:pipeline_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@core//cyber",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cpplint()
