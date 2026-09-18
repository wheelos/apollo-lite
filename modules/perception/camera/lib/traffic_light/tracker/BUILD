load("@rules_cc//cc:defs.bzl", "cc_library")
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
        "//modules/perception/camera/lib/interface",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:semantic_cc_proto",
        "@boost.bind",
        "@core//cyber",
    ],
    alwayslink = True,
)

cpplint()
