load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spatio_temporal_ground_detector",
    srcs = ["spatio_temporal_ground_detector.cc"],
    hdrs = ["spatio_temporal_ground_detector.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/lib/registerer",
        "@eigen",
        #"//modules/perception/lib/io:protobuf_util",
        "//modules/perception/common/i_lib",
        "//modules/perception/common/i_lib/pc:i_ground",
        "//modules/perception/common/i_lib/pc:i_struct_s",
        "//modules/perception/common/i_lib/pc:i_util",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/lidar/common",
        "//modules/perception/pipeline/proto/stage:spatio_temporal_ground_detector_config_cc_proto",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "//modules/perception/pipeline:stage",
    ],
    alwayslink = True,
)

cc_test(
    name = "spatio_temporal_ground_detector_test",
    size = "small",
    srcs = ["spatio_temporal_ground_detector_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":spatio_temporal_ground_detector",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest_main",
        "@local_config_pcl//:common",
    ],
)

cpplint()
