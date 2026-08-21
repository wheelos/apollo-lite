load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "status",
    hdrs = ["status.h"],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
)

cc_library(
    name = "problem_validator",
    srcs = ["problem_validator.cc"],
    hdrs = ["problem_validator.h"],
    deps = [
        ":status",
        ":types",
    ],
)

cpplint()

