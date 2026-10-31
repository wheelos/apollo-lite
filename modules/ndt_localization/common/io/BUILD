load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_io",
    srcs = ["velodyne_utility.cc"],
    hdrs = [
        "pcl_point_types.h",
        "velodyne_utility.h",
    ],
    deps = [
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@core//cyber",
        "@local_config_pcl//:io",
        "@wheelos_common//modules/common/util:util_tool",
    ],
)
