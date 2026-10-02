load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "likelihood_field",
    srcs = ["likelihood_field.cc"],
    hdrs = ["likelihood_field.h"],
    deps = [
        "//modules/localization/core:types",
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:kdtree",
    ],
)

cc_library(
    name = "particle_relocalizer",
    srcs = ["particle_relocalizer.cc"],
    hdrs = ["particle_relocalizer.h"],
    deps = [
        ":likelihood_field",
        "//modules/localization/core:types",
        "@eigen",
        "@local_config_pcl//:common",
    ],
)

cc_library(
    name = "particle_hypothesis_tracker",
    srcs = ["particle_hypothesis_tracker.cc"],
    hdrs = ["particle_hypothesis_tracker.h"],
    deps = [
        ":particle_relocalizer",
        "//modules/localization/core:types",
        "@eigen",
    ],
)

cc_test(
    name = "particle_relocalizer_test",
    size = "small",
    srcs = ["particle_relocalizer_test.cc"],
    deps = [":particle_relocalizer", "@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "particle_hypothesis_tracker_test",
    size = "small",
    srcs = ["particle_hypothesis_tracker_test.cc"],
    deps = [
        ":particle_hypothesis_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "likelihood_field_test",
    size = "small",
    srcs = ["likelihood_field_test.cc"],
    deps = [":likelihood_field", "@com_google_googletest//:gtest_main"],
)

cc_library(
    name = "map_matcher",
    srcs = ["map_matcher.cc"],
    hdrs = ["map_matcher.h"],
    deps = [
        "//modules/localization/core:observed_subspace",
        "//modules/localization/core:types",
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:filters",
        "@local_config_pcl//:io",
        "@local_config_pcl//:kdtree",
        "@local_config_pcl//:registration",
    ],
)

cc_test(
    name = "map_matcher_test",
    size = "small",
    srcs = ["map_matcher_test.cc"],
    deps = [":map_matcher", "@com_google_googletest//:gtest_main"],
)

cpplint()
