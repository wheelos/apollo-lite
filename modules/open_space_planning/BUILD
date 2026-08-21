load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_planning",
    hdrs = ["open_space_planning.h"],
    deps = [
        "//modules/open_space_planning/common:status",
        "//modules/open_space_planning/common:types",
        "//modules/open_space_planning/route:route_planner",
        "//modules/open_space_planning/runtime:open_space_planner",
        "//modules/open_space_planning/safety:fallback_planner",
        "//modules/open_space_planning/safety:trajectory_validator",
        "//modules/open_space_planning/trajectory:trajectory_planner",
    ],
)

cpplint()

