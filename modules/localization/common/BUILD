load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_gflags",
    srcs = ["localization_gflags.cc"],
    hdrs = [
        "localization_gflags.h",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "rigid_transform_helper",
    srcs = ["rigid_transform_helper.cc"],
    hdrs = ["rigid_transform_helper.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/transform:transform_query",
        "@eigen",
    ],
)

install(
    name = "install",
    library_dest = "localization/lib",
    targets = [
        ":localization_gflags",
        ":rigid_transform_helper",
    ],
)

cpplint()
