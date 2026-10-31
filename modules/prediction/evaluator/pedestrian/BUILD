load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pedestrian_interaction_evaluator",
    srcs = ["pedestrian_interaction_evaluator.cc"],
    hdrs = ["pedestrian_interaction_evaluator.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        #"//third_party/libtorch",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
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
