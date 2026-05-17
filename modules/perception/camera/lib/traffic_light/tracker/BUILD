load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "semantic_decision",
    srcs = ["semantic_decision.cc"],
    hdrs = ["semantic_decision.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cyber",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:semantic_cc_proto",
        "@boost.bind",
    ],
    alwayslink = True,
)

cc_test(
    name = "semantic_decision_test",
    size = "small",
    srcs = ["semantic_decision_test.cc"],
    deps = [
        ":semantic_decision",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
