load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_planner",
    hdrs = ["trajectory_planner.h"],
    deps = [
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
    ],
)

cpplint()

