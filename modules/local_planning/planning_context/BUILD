load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "input_contract",
    hdrs = [
        "input_contract.h",
        "scene.h",
    ],
    deps = [
        "//modules/world_model/local_map:local_scene_contracts",
    ],
)

cc_library(
    name = "input_gate",
    srcs = ["input_gate.cc"],
    hdrs = ["input_gate.h"],
    deps = [
        ":input_contract",
        "//modules/common/status",
    ],
)

cc_library(
    name = "trajectory_geometry",
    hdrs = ["trajectory_geometry.h"],
    deps = [
        ":input_contract",
        "//modules/common/math:geometry",
    ],
)

cc_library(
    name = "environment_geometry",
    srcs = ["environment_geometry.cc"],
    hdrs = ["environment_geometry.h"],
    deps = [
        ":input_contract",
        ":trajectory_geometry",
        "//modules/common/status",
        "//modules/world_model/local_map:local_scene_contracts",
    ],
)

cc_library(
    name = "corridor_selector",
    srcs = ["corridor_selector.cc"],
    hdrs = ["corridor_selector.h"],
    deps = [
        ":input_contract",
        "//modules/common/math:geometry",
        "//modules/common/status",
        "//modules/world_model/local_map:local_scene_contracts",
    ],
)

cc_library(
    name = "boundary_geometry",
    srcs = ["boundary_geometry.cc"],
    hdrs = ["boundary_geometry.h"],
    deps = [
        ":input_contract",
        "//modules/common/status",
    ],
)

cc_library(
    name = "reference_continuity",
    srcs = ["reference_continuity.cc"],
    hdrs = ["reference_continuity.h"],
    deps = [
        ":corridor_selector",
        ":trajectory_geometry",
        "//modules/common/math:geometry",
        "//modules/common/status",
        "//modules/world_model/local_map:local_scene_contracts",
    ],
)

cc_library(
    name = "reference_line",
    srcs = ["reference_line.cc"],
    hdrs = ["reference_line.h"],
    deps = [
        ":boundary_geometry",
        ":corridor_selector",
        ":reference_continuity",
        ":trajectory_geometry",
        "//modules/common/math:geometry",
        "//modules/common/status",
        "//modules/world_model/local_map:local_scene_contracts",
    ],
)

cc_library(
    name = "planning_geometry",
    srcs = ["planning_geometry.cc"],
    hdrs = ["planning_geometry.h"],
    deps = [
        ":environment_geometry",
        ":reference_line",
        "//modules/common/status",
        "//modules/world_model/local_map:local_scene_contracts",
    ],
)

cc_library(
    name = "planning_input_builder",
    srcs = ["planning_input_builder.cc"],
    hdrs = ["planning_input_builder.h"],
    deps = [
        ":environment_geometry",
        ":input_gate",
        ":planning_geometry",
        "//modules/common/status",
        "//modules/world_model/local_map:local_scene_contracts",
    ],
)

cc_test(
    name = "planning_input_builder_test",
    size = "small",
    srcs = ["planning_input_builder_test.cc"],
    deps = [
        ":planning_input_builder",
        "//modules/common/status",
        "//modules/world_model/local_map:local_scene_contracts",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
