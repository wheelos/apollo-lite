load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]

install(
    name = "install",
    library_dest = "map/lib",
    targets = [
        ":hdmap",
        ":hdmap_util"
        ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdmap",
    srcs = [
        "hdmap.cc",
        "hdmap_common.cc",
        "hdmap_impl.cc",
    ],
    hdrs = [
        "hdmap.h",
        "hdmap_common.h",
        "hdmap_impl.h",
        "hdmap_util.h",
    ],
    copts = MAP_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/map/hdmap/adapter:opendrive_adapter",
        "@com_google_absl//absl/strings",
        "@com_github_google_glog//:glog",
        "@eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdmap_util",
    srcs = [
        "hdmap_util.cc",
    ],
    hdrs = ["hdmap_util.h"],
    copts = MAP_COPTS,
    deps = [
        ":hdmap",
        "//cyber",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common/configs:config_gflags",
        "@com_google_absl//absl/strings",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "test-data/*",
    ]),
)

cc_test(
    name = "hdmap_map_test",
    size = "small",
    timeout = "short",
    srcs = [
        "hdmap_common_test.cc",
        "hdmap_impl_test.cc",
    ],
    tags = ["exclude"],
    data = [
        ":testdata",
        #"//modules/common/util",
    ],
    deps = [
        ":hdmap",
        "//cyber",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "hdmap_util_test",
    size = "small",
    timeout = "short",
    srcs = ["hdmap_util_test.cc"],
    data = [
        ":testdata",
    ],
    deps = [
        ":hdmap_util",
        "@com_google_absl//absl/strings",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
