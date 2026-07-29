load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onnx_multi_batch_infer",
    srcs = ["onnx_multi_batch_infer.cc"],
    hdrs = ["onnx_multi_batch_infer.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "@core//cyber",
        "//modules/perception/inference:inference_lib",
        "@cuda//:cuda_runtime",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cpplint()

