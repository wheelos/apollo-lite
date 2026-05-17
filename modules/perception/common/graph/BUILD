load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "graph",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":conditional_clustering",
        ":connected_component_analysis",
        ":disjoint_set",
        ":gated_hungarian_bigraph_matcher",
        ":graph_segmentor",
        ":hungarian_optimizer",
        ":secure_matrix",
    ],
)

cc_library(
    name = "disjoint_set",
    srcs = ["disjoint_set.cc"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    hdrs = ["disjoint_set.h"],
)

cc_test(
    name = "disjoint_set_test",
    size = "small",
    srcs = ["disjoint_set_test.cc"],
    deps = [
        ":disjoint_set",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "graph_segmentor",
    srcs = ["graph_segmentor.cc"],
    hdrs = ["graph_segmentor.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":disjoint_set",
        "//cyber",
    ],
)

cc_test(
    name = "graph_segmentor_test",
    size = "small",
    srcs = ["graph_segmentor_test.cc"],
    deps = [
        ":graph_segmentor",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "secure_matrix",
    hdrs = ["secure_matrix.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "secure_matrix_test",
    size = "small",
    srcs = ["secure_matrix_test.cc"],
    deps = [
        ":secure_matrix",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_library(
    name = "hungarian_optimizer",
    hdrs = ["hungarian_optimizer.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "hungarian_optimizer_test",
    size = "small",
    srcs = ["hungarian_optimizer_test.cc"],
    deps = [
        ":hungarian_optimizer",
        ":secure_matrix",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "connected_component_analysis",
    srcs = ["connected_component_analysis.cc"],
    hdrs = ["connected_component_analysis.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "connected_component_analysis_test",
    size = "small",
    srcs = ["connected_component_analysis_test.cc"],
    deps = [
        ":connected_component_analysis",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_library(
    name = "conditional_clustering",
    hdrs = ["conditional_clustering.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/geometry:basic",
        "@eigen",
    ],
)

cc_test(
    name = "conditional_clustering_test",
    size = "small",
    srcs = ["conditional_clustering_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":conditional_clustering",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gated_hungarian_bigraph_matcher",
    hdrs = ["gated_hungarian_bigraph_matcher.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":connected_component_analysis",
        ":hungarian_optimizer",
        ":secure_matrix",
        "//cyber",
    ],
)

cc_test(
    name = "gated_hungarian_bigraph_matcher_test",
    size = "small",
    srcs = ["gated_hungarian_bigraph_matcher_test.cc"],
    deps = [
        ":gated_hungarian_bigraph_matcher",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
