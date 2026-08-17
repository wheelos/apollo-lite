load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "scenario_case",
    hdrs = ["scenario_case.h"],
    deps = [
        "//modules/planning/proto:planning_config_cc_proto"
    ],
)

cc_library(
    name = "scenario_builder",
    srcs = ["scenario_builder.cc"],
    hdrs = ["scenario_builder.h"],
    deps = [
        ":scenario_case",
        "@core//cyber",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame"
    ],
)

cc_library(
    name = "scenario_loader",
    srcs = ["scenario_loader.cc"],
    hdrs = ["scenario_loader.h"],
    deps = [
        "@core//cyber",
        ":scenario_case",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp"
    ],
)

cc_test(
    name = "scenario_manager_test",
    size = "small",
    srcs = ["scenario_manager_test.cc"],
    data = [
        "scenario_cases.yaml",
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-fopenmp"],
    deps = [
        "@core//cyber",
        "//modules/planning/common:gtest_exit_main",
        ":scenario_case",
        ":scenario_builder",
        ":scenario_loader",
        "//modules/planning/scenarios:scenario_manager",
    ],
)

cpplint()
