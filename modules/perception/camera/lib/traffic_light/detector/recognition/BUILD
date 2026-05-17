load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "classify",
    srcs = ["classify.cc"],
    hdrs = ["classify.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:recognition_cc_proto",
    ],
)

cc_library(
    name = "recognition",
    srcs = [
        "efficient_net_recognizer.cc",
        "recognition.cc",
    ],
    hdrs = [
        "efficient_net_recognizer.h",
        "recognition.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":classify",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference/onnx_multi_batch:onnx_multi_batch_infer",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
    ],
    alwayslink = True,
)

cpplint()
