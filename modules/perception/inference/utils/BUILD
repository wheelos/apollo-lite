load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cuda//cuda:defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "inference_cuda_util_cuda_lib",
    srcs = ["cuda_util.cu"],
    hdrs = ["cuda_util.h"],
    deps = [
        "@core//cyber",
        "@boost.thread",
        "@cuda//:cuda_runtime",
    ],
)

cc_library(
    name = "inference_cuda_util_lib",
    srcs = [":inference_cuda_util_cuda_lib"],
    hdrs = ["cuda_util.h"],
    linkstatic = 1,
    deps = [
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cuda_library(
    name = "inference_util_cuda_lib",
    srcs = ["util.cu"],
    hdrs = ["util.h"],
    deps = [
        "@core//cyber",
        "//modules/perception/base",
        "@cuda//:cuda_runtime",
        "@eigen",
    ],
)

cc_library(
    name = "inference_util_lib",
    srcs = [
        "util.cc",
        ":inference_util_cuda_lib",
    ],
    hdrs = ["util.h"],
    linkstatic = 1,
    deps = [
        "//modules/perception/base:blob",
        "//modules/perception/base:box",
        "//modules/perception/base:image",
        "//modules/perception/base:point_cloud",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cuda_library(
    name = "inference_resize_cuda_lib",
    srcs = ["resize.cu"],
    hdrs = ["resize.h"],
    deps = [
        ":inference_cuda_util_lib",
        ":inference_util_lib",
        "@core//cyber",
        "//modules/perception/base:blob",
        "//modules/perception/base:image",
        "@cuda//:cublas",
        "@cuda//:cuda_runtime",
    ],
)

cc_library(
    name = "inference_resize_lib",
    srcs = [":inference_resize_cuda_lib"],
    hdrs = ["resize.h"],
    linkstatic = 1,
    deps = [
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cuda_library(
    name = "inference_gemm_cuda_lib",
    srcs = ["gemm.cu"],
    hdrs = ["gemm.h"],
    deps = [
        ":inference_cuda_util_lib",
        ":inference_util_lib",
        "@core//cyber",
        "//modules/perception/base:blob",
        "//modules/perception/base:image",
        "@cuda//:cuda_runtime",
    ],
)

cc_library(
    name = "inference_gemm_lib",
    srcs = [":inference_gemm_cuda_lib"],
    hdrs = ["gemm.h"],
    linkstatic = 1,
    deps = [
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "inference_util_test",
    size = "small",
    srcs = ["util_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    linkstatic = True,
    deps = [
        ":inference_util_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inference_binary_data_lib",
    srcs = ["binary_data.cc"],
    hdrs = ["binary_data.h"],
    deps = [
        "//modules/perception/base:blob",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "binary_data_test",
    size = "small",
    srcs = ["binary_data_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    linkstatic = True,
    deps = [
        ":inference_binary_data_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
