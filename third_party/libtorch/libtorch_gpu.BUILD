load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_gpu",
    includes = [
        ".",
        "torch/csrc/api/include",
    ],
    linkopts = [
        "-Wl,-rpath,/usr/local/libtorch/lib",
        "-L/usr/local/libtorch/lib",
        "-lc10",
        "-lc10_cuda",
        "-ltorch",
        "-ltorch_cpu",
        "-ltorch_cuda",
    ],
    linkstatic = False,
    deps = [
        "@cuda//:cuda_runtime",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)
