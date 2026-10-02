load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    hdrs = [
        "extract_ground_plane.h",
        "rect2d.h",
        "voxel_grid_covariance_hdmap.h",
    ],
    deps = [
        "@boost.mpl",
        "@core//cyber",
        "@eigen",
        "@local_config_pcl//:filters",
        "@local_config_pcl//:kdtree",
        "@local_config_pcl//:sample_consensus",
    ],
)

cc_library(
    name = "base_map_cache",
    hdrs = ["base_map_cache.h"],
    deps = ["@wheelos_common//modules/common/util:util_tool"],
)

cc_library(
    name = "compression",
    srcs = ["compression.cc"],
    hdrs = ["compression.h"],
    deps = [
        "@core//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "md5",
    srcs = ["md5.cc"],
    hdrs = ["md5.h"],
)

cc_library(
    name = "file_utility",
    srcs = ["file_utility.cc"],
    hdrs = ["file_utility.h"],
    deps = [
        ":md5",
        "@boost.filesystem",
        "@com_github_google_glog//:glog",
        "@core//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "system_utility",
    srcs = ["system_utility.cc"],
    hdrs = ["system_utility.h"],
    deps = [
        "@boost.filesystem",
        "@core//cyber",
        "@eigen",
    ],
)

cpplint()
