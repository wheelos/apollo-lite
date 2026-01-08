load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "graph_segmentation",
    srcs = [
        "graph_segmentation.cc",
        "background_map.cc"
    ],
    hdrs = [
        "graph_segmentation.h",
        "background_map.h"
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/common/graph:graph_segmentor",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/detector/cnn_segmentation/spp_engine:spp_cluster_list",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lidar/lib/detector/graph_segmentation/common:lidar_segmentaion_common",
        "//modules/perception/pipeline/proto/stage:graph_segmentation_config_cc_proto",
        "@eigen",
    ],
)

cpplint()
