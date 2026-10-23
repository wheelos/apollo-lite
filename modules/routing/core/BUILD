load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

cc_library(
    name = "core",
    deps = [
        ":routing_navigator",
    ],
)

cc_library(
    name = "routing_navigator",
    srcs = ["navigator.cc"],
    hdrs = ["navigator.h"],
    copts = ROUTING_COPTS,
    deps = [
        ":directional_routing",
        ":routing_black_list_range_generator",
        ":routing_result_generator",
        "@wheelos_common//modules/common/util",
        "//modules/routing/strategy",
    ],
)

cc_library(
    name = "directional_routing",
    srcs = ["directional_routing.cc"],
    hdrs = ["directional_routing.h"],
    deps = [
        "//modules/routing/proto:topo_graph_cc_proto",
        "@core//cyber/common:log",
        "@wheelos_msgs//wheelos_msgs/routing_msgs:routing_cc_proto",
    ],
)

cc_test(
    name = "directional_routing_test",
    size = "small",
    srcs = ["directional_routing_test.cc"],
    deps = [
        ":directional_routing",
        ":routing_navigator",
        "@com_google_googletest//:gtest_main",
        "@core//cyber/common:file",
    ],
)

cc_library(
    name = "routing_black_list_range_generator",
    srcs = ["black_list_range_generator.cc"],
    hdrs = ["black_list_range_generator.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/routing/graph",
    ],
)

cc_library(
    name = "routing_result_generator",
    srcs = ["result_generator.cc"],
    hdrs = ["result_generator.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/routing/graph",
    ],
)

cpplint()
