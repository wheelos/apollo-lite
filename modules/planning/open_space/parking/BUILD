load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "parking_slot",
    srcs = ["parking_slot.cc"],
    hdrs = ["parking_slot.h"],
    copts = PLANNING_COPTS,
    deps = [
        "@core//cyber",
        "//modules/common/math:math_utils",
        "//modules/common/math:vec2d",
    ],
)

cc_library(
    name = "parking_slot_provider",
    srcs = ["parking_slot_provider.cc"],
    hdrs = ["parking_slot_provider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":parking_slot",
        "@core//cyber",
        "//modules/map/hdmap:hdmap",
        "//modules/map/pnc_map",
    ],
)

cc_library(
    name = "parking_roi_geometry",
    srcs = ["parking_roi_geometry.cc"],
    hdrs = ["parking_roi_geometry.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":parking_slot",
        "@core//cyber",
        "//modules/common/math:geometry",
        "//modules/common/math:math_utils",
        "//modules/common/math:vec2d",
        "@wheelos_msgs//wheelos_msgs/config_msgs:vehicle_config_cc_proto",
        "@boost.geometry",
    ],
)

cc_library(
    name = "parking_roi_validator",
    srcs = ["parking_roi_validator.cc"],
    hdrs = ["parking_roi_validator.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":parking_roi_geometry",
        "@core//cyber",
        "//modules/common/math:geometry",
        "//modules/common/math:vec2d",
        "@wheelos_msgs//wheelos_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/planning/open_space/coarse_trajectory_generator:node3d",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_library(
    name = "parking_pose_selector",
    srcs = ["parking_pose_selector.cc"],
    hdrs = ["parking_pose_selector.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":parking_roi_geometry",
        ":parking_slot",
        "@core//cyber",
        "//modules/common/math:math_utils",
        "//modules/common/math:vec2d",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_test(
    name = "parking_slot_provider_test",
    size = "small",
    srcs = ["parking_slot_provider_test.cc"],
    deps = [
        ":parking_slot",
        ":parking_slot_provider",
        "@wheelos_msgs//wheelos_msgs/map_msgs:map_parking_space_cc_proto",
        "//modules/planning/testing:gtest_exit_main",
        "//modules/map/pnc_map:path",
    ],
)

cc_test(
    name = "parking_pose_selector_test",
    size = "small",
    srcs = ["parking_pose_selector_test.cc"],
    deps = [
        ":parking_pose_selector",
        "//modules/planning/testing:gtest_exit_main",
    ],
)

cc_test(
    name = "parking_roi_builder_test",
    size = "small",
    srcs = ["parking_roi_builder_test.cc"],
    deps = [
        ":parking_roi_geometry",
        "//modules/planning/testing:gtest_exit_main",
    ],
)

cc_test(
    name = "parking_roi_validator_test",
    size = "small",
    srcs = ["parking_roi_validator_test.cc"],
    deps = [
        ":parking_pose_selector",
        ":parking_roi_validator",
        "//modules/planning/testing:gtest_exit_main",
    ],
)

cpplint()
