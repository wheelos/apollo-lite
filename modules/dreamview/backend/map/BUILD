load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "dreamview/lib",
    targets = [":map_service"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "map_service",
    srcs = [
        "map_service.cc",
    ],
    hdrs = ["map_service.h"],
    alwayslink = True,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/dreamview/proto:simulation_world_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "@boost.thread",
        "@com_github_nlohmann_json//:json",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "map_service_test",
    size = "small",
    srcs = ["map_service_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":map_service",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
