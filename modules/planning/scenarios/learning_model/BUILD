load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "learning_model",
    copts = PLANNING_COPTS,
    deps = [
        ":learning_model_sample_scenario",
    ],
)

cc_library(
    name = "learning_model_sample_scenario",
    srcs = [
        "learning_model_sample_scenario.cc",
        "stage_run.cc",
    ],
    hdrs = [
        "learning_model_sample_scenario.h",
        "stage_run.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/planning/common:frame",
        "//modules/planning/scenarios:scenario",
        "@core//cyber",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_test(
    name = "learning_model_sample_scenario_test",
    size = "small",
    srcs = ["learning_model_sample_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":learning_model_sample_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stage_run_test",
    size = "small",
    srcs = ["stage_run_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":learning_model_sample_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
