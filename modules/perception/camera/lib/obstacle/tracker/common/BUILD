load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "similar_cuda",
    srcs = ["similar.cu"],
    hdrs = ["similar.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common:camera_frame",
        "//modules/perception/camera/common:util",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "@cuda//:cuda_runtime",
    ],
)

cc_library(
    name = "similar",
    srcs = [
        "similar.cc",
        ":similar_cuda",
    ],
    hdrs = ["similar.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "@eigen",
    ],
)

cc_library(
    name = "half_circle_angle",
    srcs = ["half_circle_angle.cc"],
    hdrs = ["half_circle_angle.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//modules/perception/camera/common",
    ],
)

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "@eigen",
    ],
)

cc_library(
    name = "common",
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        ":half_circle_angle",
        ":kalman_filter",
        ":similar",
    ],
)

cpplint()
