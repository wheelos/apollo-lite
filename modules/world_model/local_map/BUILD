load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_types",
    hdrs = ["lane_types.h"],
)

cc_library(
    name = "lane_association",
    srcs = ["lane_association.cc"],
    hdrs = ["lane_association.h"],
    deps = [
        ":lane_types",
        "@wheelos_common//modules/common/status",
    ],
)

cc_library(
    name = "temporal_lane_map",
    srcs = ["temporal_lane_map.cc"],
    hdrs = ["temporal_lane_map.h"],
    deps = [
        ":lane_association",
        ":lane_types",
        "@wheelos_common//modules/common/status",
    ],
)

cc_library(
    name = "measured_lane_geometry",
    srcs = ["measured_lane_geometry.cc"],
    hdrs = ["measured_lane_geometry.h"],
    deps = [
        ":temporal_lane_map",
        "@wheelos_common//modules/common/status",
    ],
)

cc_test(
    name = "lane_association_test",
    size = "small",
    srcs = ["lane_association_test.cc"],
    deps = [
        ":lane_association",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "temporal_lane_map_test",
    size = "small",
    srcs = ["temporal_lane_map_test.cc"],
    deps = [
        ":temporal_lane_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "local_scene_contracts",
    hdrs = ["local_scene.h"],
    deps = [":lane_types"],
)

cc_library(
    name = "drivable_environment",
    srcs = ["drivable_environment.cc"],
    hdrs = ["drivable_environment.h"],
    deps = [
        ":lane_types",
        ":local_scene_contracts",
        "@wheelos_common//modules/common/status",
    ],
)

cc_library(
    name = "lane_synthesis",
    srcs = ["lane_synthesis.cc"],
    hdrs = ["lane_synthesis.h"],
    deps = [
        ":lane_types",
        ":local_scene_contracts",
        "@wheelos_common//modules/common/status",
    ],
)

cc_library(
    name = "navigation_prior",
    srcs = ["navigation_prior.cc"],
    hdrs = ["navigation_prior.h"],
    deps = [
        ":local_scene_contracts",
        "@wheelos_common//modules/common/status",
    ],
)

cc_library(
    name = "scene_assembly",
    srcs = ["scene_assembly.cc"],
    hdrs = ["scene_assembly.h"],
    deps = [
        ":drivable_environment",
        ":lane_synthesis",
        ":local_scene_contracts",
        ":navigation_prior",
        "@wheelos_common//modules/common/status",
    ],
)

cc_test(
    name = "scene_assembly_test",
    size = "small",
    srcs = ["scene_assembly_test.cc"],
    deps = [
        ":scene_assembly",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "local_scene_builder",
    srcs = ["local_scene_builder.cc"],
    hdrs = ["local_scene_builder.h"],
    deps = [
        ":drivable_environment",
        ":lane_synthesis",
        ":local_scene_contracts",
        ":scene_assembly",
        ":temporal_lane_map",
        "@wheelos_common//modules/common/status",
    ],
)

cc_test(
    name = "local_scene_test",
    size = "small",
    srcs = ["local_scene_test.cc"],
    deps = [
        ":local_scene_builder",
        ":measured_lane_geometry",
        "@com_google_googletest//:gtest_main",
    ],
)
