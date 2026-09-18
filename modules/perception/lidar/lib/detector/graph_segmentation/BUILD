load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "graph_segmentation",
    srcs = [
        "background_map.cc",
        "graph_segmentation.cc",
    ],
    hdrs = [
        "background_map.h",
        "graph_segmentation.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//modules/perception/common/graph:graph_segmentor",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/spp_engine:spp_cluster_list",
        "//modules/perception/lidar/lib/detector/graph_segmentation/common:lidar_segmentaion_common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/pipeline/proto/stage:graph_segmentation_config_cc_proto",
        "@core//cyber",
        "@eigen",
    ],
)

cpplint()

cc_test(
    name = "graph_segmentation_test",
    size = "small",
    srcs = ["graph_segmentation_test.cc"],
    deps = [
        ":graph_segmentation",
        "@com_google_googletest//:gtest_main",
    ],
)
