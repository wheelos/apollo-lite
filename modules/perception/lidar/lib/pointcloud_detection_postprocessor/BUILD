load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_detection_postprocessor",
    srcs = [
        "pointcloud_detection_postprocessor.cc",
    ],
    hdrs = [
        "pointcloud_detection_postprocessor.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_get_objects",
        "//modules/perception/pipeline:plugin_factory",
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
