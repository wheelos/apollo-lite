load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "valet_parking",
    srcs = [
        "stage_parking.cc",
        "valet_parking_scenario.cc",
    ],
    hdrs = [
        "stage_parking.h",
        "valet_parking_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "@core//cyber",
        "@wheelos_msgs//wheelos_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
    ],
)

cc_test(
    name = "valet_parking_scenario_test",
    size = "small",
    srcs = ["valet_parking_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":valet_parking",
        "//modules/planning/common/util:util_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stage_parking_test",
    size = "small",
    srcs = ["stage_parking_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":valet_parking",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
