load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    targets = [
        ":config_gflags",
        ":vehicle_config_helper",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "config_gflags",
    srcs = [
        "config_gflags.cc",
    ],
    hdrs = ["config_gflags.h"],
    data = [
        "//modules/common/data:global_flagfile",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "vehicle_config_helper",
    srcs = [
        "vehicle_config_helper.cc",
    ],
    hdrs = ["vehicle_config_helper.h"],
    data = [
        "//modules/common/data:vehicle_config_data",
    ],
    deps = [
        ":config_gflags",
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "vehicle_config_helper_test",
    size = "small",
    srcs = ["vehicle_config_helper_test.cc"],
    deps = [
        ":vehicle_config_helper",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
