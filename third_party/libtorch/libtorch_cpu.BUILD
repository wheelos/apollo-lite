load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_cpu",
    hdrs = glob([
        "include/**/*.h",
        "include/**/*.hpp",
        "include/**/*.cuh",
        "include/**/*.c",
        "include/**/*.cpp",
        "include/**/*.cxx",
    ]),
    includes = [
        "include",
        "include/torch/csrc/api/include",
    ],
    linkopts = [
        "-L/usr/local/libtorch/lib",
        "-Wl,-rpath,/usr/local/libtorch/lib",
        "-ltorch_cpu",
        "-ltorch",
        "-lc10",
    ],
    linkstatic = False,
    deps = [
        "@rules_python//python/cc:current_py_cc_headers",
        "@rules_python//python/cc:current_py_cc_libs",
    ],
)
