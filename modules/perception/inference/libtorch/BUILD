load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "torch_net",
    srcs = ["torch_net.cc"],
    hdrs = ["torch_net.h"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "@com_google_protobuf//:protobuf",
        "@cuda//:cuda_runtime",
        "@libtorch_gpu",
        "@opencv//:imgcodecs",
    ],
)

cpplint()
