load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "ground_analyzer",
    srcs = ["ground_analyzer.cc"],
    copts = ['-DMODULE_NAME=\\"perception\\"'],
    deps = [
        "//cyber",
        "//modules/perception/common/i_lib",
        "//modules/perception/common/i_lib/pc:i_ground",
        "//modules/perception/common/i_lib/pc:i_struct_s",
        "//modules/perception/lidar/common:pcl_util",
        "@eigen",
    ],
    visibility = ["//visibility:public"],
)
