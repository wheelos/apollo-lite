load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_cpu",
    includes = [
        ".",
        "torch/csrc/api/include",
    ],
    linkopts = [
        "-Wl,-rpath,/usr/local/libtorch/lib",
        "-L/usr/local/libtorch/lib",
        "-ltorch_cpu",
        "-ltorch",
        "-lc10",
    ],
    linkstatic = False,
    deps = [
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)
