load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "paddleinference_lib",
    srcs = [
        "paddle/lib/libpaddle_inference.so",
        # "third_party/install/gflags/lib/libgflags.so.2.2",
        "third_party/install/mkldnn/lib/libdnnl.so.2",
        "third_party/install/mkldnn/lib/libmkldnn.so.0",
        "third_party/install/mklml/lib/libiomp5.so",
        "third_party/install/mklml/lib/libmklml_intel.so",
    ],
    hdrs = glob([
        "paddle/include/*.h",
        "paddle/include/**/*.h",
        "paddle/include/**/**/*.h",
        "third_party/install/**/include/*.h",
        "third_party/install/mkldnn/include/oneapi/dnnl/*.h",
        # "third_party/install/gflags/include/gflags/*.h",
    ]),
    includes = [
        "include",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
    deps = [
        # Note: use gflgas from bazel instead of the pre-compiled one
        "@com_github_gflags_gflags//:gflags",
    ],
)
