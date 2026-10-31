load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "standing_still_trajectory1d",
    srcs = ["standing_still_trajectory1d.cc"],
    hdrs = ["standing_still_trajectory1d.h"],
    deps = [
        "//modules/planning/math/curve1d",
        "@core//cyber",
    ],
)

cc_library(
    name = "piecewise_acceleration_trajectory1d",
    srcs = ["piecewise_acceleration_trajectory1d.cc"],
    hdrs = ["piecewise_acceleration_trajectory1d.h"],
    deps = [
        "@wheelos_common//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math/curve1d",
    ],
)

cc_library(
    name = "constant_deceleration_trajectory1d",
    srcs = ["constant_deceleration_trajectory1d.cc"],
    hdrs = ["constant_deceleration_trajectory1d.h"],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math/curve1d",
        "@core//cyber",
    ],
)

cc_library(
    name = "piecewise_trajectory1d",
    srcs = ["piecewise_trajectory1d.cc"],
    hdrs = ["piecewise_trajectory1d.h"],
    deps = [
        "@wheelos_common//modules/common/math",
        "//modules/planning/math/curve1d",
    ],
)

cc_library(
    name = "constant_jerk_trajectory1d",
    srcs = ["constant_jerk_trajectory1d.cc"],
    hdrs = ["constant_jerk_trajectory1d.h"],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math/curve1d",
        "@core//cyber",
    ],
)

cc_library(
    name = "piecewise_jerk_trajectory1d",
    srcs = ["piecewise_jerk_trajectory1d.cc"],
    hdrs = ["piecewise_jerk_trajectory1d.h"],
    deps = [
        ":constant_jerk_trajectory1d",
    ],
)

filegroup(
    name = "exported_source_files",
    srcs = glob(
        ["**/*"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)

filegroup(
    name = "exported_source_headers",
    srcs = glob(
        ["**/*.h"],
        allow_empty = True,
    ),
    visibility = ["//visibility:public"],
)
