load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "dreamview/bin",
    targets = [":sim_control"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "sim_control_manager",
    srcs = ["sim_control_manager.cc"],
    hdrs = [
        "sim_control_manager.h",
    ],
    alwayslink = True,
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_gflags",
        "@core//cyber",
        "//modules/dreamview/backend/sim_control_manager/core:dynamic_model_factory",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_binary(
    name = "sim_control",
    srcs = ["main.cc"],
    copts = ['-DMODULE_NAME=\\"sim_control\\"'],
    deps = [
        ":sim_control_manager",
        "@core//cyber",
    ],
)

# todo: add test file
# cc_test(
#     name = "sim_control_test",
#     size = "small",
#     srcs = ["sim_control_test.cc"],
#     data = [
#         "//modules/dreamview/backend/testdata",
#     ],
#     deps = [
#         ":sim_control",
#         "@core//cyber",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cpplint()
